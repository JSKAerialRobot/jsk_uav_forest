// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include "jsk_uav_forest_perception/vision_detector.h"

namespace vision_detection
{
  VisionDetector::VisionDetector():
    camera_fx_(-1), camera_fy_(-1), camera_cx_(-1), camera_cy_(-1)
  {
  }

  VisionDetector::~VisionDetector()
  {
  }

  void VisionDetector::onInit()
  {
    pnh_ = this->getPrivateNodeHandle();

    /* ros param */
    pnh_.param("image_topic_name", image_topic_name_, std::string("/camera/image_rect"));
    pnh_.param("camera_info_topic_name", camera_info_topic_name_, std::string("camera_info"));
    pnh_.param("laser_scan_topic_name", laser_scan_topic_name_, std::string("/scan"));
    pnh_.param("target_image_topic_name", target_image_topic_name_, std::string("/target"));
    pnh_.param("detection_start_topic_name", detection_start_topic_name_, string("/detection_start"));
    pnh_.param("target_image_center_topic_name", target_image_center_topic_name_, std::string("/object_image_center"));
    pnh_.param("target_direction_topic_name", target_direction_topic_name_, std::string("/object_direction"));

    pnh_.param("detect_once", detect_once_, true);
    pnh_.param("verbose", verbose_, true);

    pnh_.param("color_region_tree_clustering_angle_diff_thre", color_region_tree_clustering_angle_diff_thre_, 0.10);
    pnh_.param("first_detection_depth_thre", first_detection_depth_thre_, 9.0); // [m]

    pnh_.param("h_min", hsv_lower_bound_[0], 0);
    pnh_.param("s_min", hsv_lower_bound_[1], 0);
    pnh_.param("v_min", hsv_lower_bound_[2], 0);
    pnh_.param("h_max", hsv_upper_bound_[0], 255);
    pnh_.param("s_max", hsv_upper_bound_[1], 255);
    pnh_.param("v_max", hsv_upper_bound_[2], 255);

    pnh_.param("contour_color_r", contour_color_r_, 255);
    pnh_.param("contour_color_g", contour_color_g_, 255);
    pnh_.param("contour_color_b", contour_color_b_, 255);
    contour_color_ = cv::Scalar(contour_color_r_, contour_color_g_, contour_color_b_);

    /* ros pub and sub */
    sub_image_.subscribe(pnh_, image_topic_name_, 1);
    sub_scan_.subscribe(pnh_, laser_scan_topic_name_, 1);
    unsubscribe();
    sync_ = boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10)));
    sync_->connectInput(sub_image_, sub_scan_);
    sync_->registerCallback(&VisionDetector::cameraScanCallback, this);

    sub_detection_start_ = pnh_.subscribe(detection_start_topic_name_, 1, &VisionDetector::startDetectionCallback, this);
    sub_camera_info_ = pnh_.subscribe(camera_info_topic_name_, 1, &VisionDetector::cameraInfoCallback, this);

    pub_target_image_ = pnh_.advertise<sensor_msgs::Image>(target_image_topic_name_, 1);
    pub_target_direction_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("/object_direction", 1);
    pub_target_image_center_ = pnh_.advertise<geometry_msgs::PointStamped>("/object_image_center", 1);

    /* ros service */
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&VisionDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void VisionDetector::configCallback(Config &new_config, uint32_t level)
  {
    if(!new_config.enable) return;

    ROS_WARN("New config");

    hsv_lower_bound_[0] = new_config.color_hsv_h_min;
    hsv_upper_bound_[0] = new_config.color_hsv_h_max;
    hsv_lower_bound_[1] = new_config.color_hsv_s_min;
    hsv_upper_bound_[1] = new_config.color_hsv_s_max;
    hsv_lower_bound_[2] = new_config.color_hsv_v_min;
    hsv_upper_bound_[2] = new_config.color_hsv_v_max;
  }

  void VisionDetector::startDetectionCallback(const std_msgs::BoolConstPtr& msg)
  {
    if(msg->data)
      {
        ROS_INFO("start detection");
        subscribe();
      }
    else
      {
        ROS_INFO("stop perception");
        unsubscribe();
      }
  }

  void VisionDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
  {
    camera_fx_ = camera_info->K[0];
    camera_fy_ = camera_info->K[4];
    camera_cx_ = camera_info->K[2];
    camera_cy_ = camera_info->K[5];
  }

  void VisionDetector::cameraScanCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg)
  {
    /* debug */
    ROS_WARN("camera scan vs laser scan time diff: %f", (image_msg->header.stamp.toSec() - scan_msg->header.stamp.toSec()));

    /* laser part */
    vector<int> cluster_index;
    /* extract the cluster */
    for (size_t i = 0; i < scan_msg->ranges.size(); i++)
      if(scan_msg->ranges[i] > 0) cluster_index.push_back(i);

    /* vision part */
    /* check whether the camera info is updated */
    if(camera_fx_ < -1) return;

    cv::Mat src_image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

    /* independent hsv filter */
    cv::Mat hsv_image, hsv_image_mask;
    if(image_msg->encoding == sensor_msgs::image_encodings::RGB8)
        cv::cvtColor(src_image, hsv_image, CV_RGB2HSV);
    else if(image_msg->encoding == sensor_msgs::image_encodings::BGR8)
        cv::cvtColor(src_image, hsv_image, CV_BGR2HSV);

    if(hsv_lower_bound_[0] < hsv_upper_bound_[0])
      {
        cv::inRange(hsv_image,
                    cv::Scalar(hsv_lower_bound_[0], hsv_lower_bound_[1], hsv_lower_bound_[2]),
                    cv::Scalar(hsv_upper_bound_[0], hsv_upper_bound_[1], hsv_upper_bound_[2]),
                    hsv_image_mask);
      }
    else
      {
        cv::Scalar lower_color_range_0 = cv::Scalar(0, hsv_lower_bound_[1], hsv_lower_bound_[2]);
        cv::Scalar upper_color_range_0 = cv::Scalar(hsv_upper_bound_[0], hsv_upper_bound_[1], hsv_upper_bound_[2]);
        cv::Scalar lower_color_range_360 = cv::Scalar(hsv_lower_bound_[0], hsv_lower_bound_[1], hsv_lower_bound_[2]);
        cv::Scalar upper_color_range_360 = cv::Scalar(180, hsv_upper_bound_[1], hsv_upper_bound_[2]);
        cv::Mat output_image_0, output_image_360;
        cv::inRange(hsv_image, lower_color_range_0, upper_color_range_0, output_image_0);
        cv::inRange(hsv_image, lower_color_range_360, upper_color_range_360, output_image_360);
        hsv_image_mask = output_image_0 | output_image_360;
      }

    /* find countour */
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(hsv_image_mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    if(contours.size() == 0) return;

    /* find the biggest contour */
    float max_contour_area = 0;
    float max_contour = 0;
    for(int i = 0; i < contours.size(); ++i)
      {
        if (cv::contourArea(contours[i]) > max_contour_area)
          {
            max_contour_area = cv::contourArea(contours[i]);
            max_contour = i;
          }
      }
    cv::Moments contour_moments = cv::moments(contours[max_contour], true);

    /* publish the base information */
    geometry_msgs::PointStamped contour_center;
    contour_center.header = image_msg->header;
    contour_center.point.x = contour_moments.m10 / contour_moments.m00;
    contour_center.point.y = contour_moments.m01 / contour_moments.m00;
    pub_target_image_center_.publish(contour_center);
    /* draw */
    cv::circle(src_image, cv::Point(contour_center.point.x, contour_center.point.y), 3, cv::Scalar(255, 0, 0), 5);
    cv::drawContours(src_image, contours, max_contour, contour_color_, 3);

    /* publish */
    pub_target_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                 image_msg->encoding,
                                                 src_image).toImageMsg());


    /* check whether the contour is satisfied the condistion */

    /* we calculate the angle between the x axis(forward) and the line to the color region */
    float color_region_direction = atan2(-contour_center.point.x + camera_cx_, camera_fx_);
    if(verbose_) cout << "vision detection direction"  << color_region_direction << endl;

    /* Find the most closed tree to the color region, record the target tree index */
    float fov = atan2(camera_cx_, camera_fx_); //proximate the width of image with camera_cx_
    float min_diff = 1e6;
    int target_tree_index = 0;
    for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
    {
      float diff = 0;

      float laser_direction = *it * scan_msg->angle_increment + scan_msg->angle_min;
      if(fabs(laser_direction) > fov)
        {
          if(verbose_) cout << "eliminate: laser_direction:" << laser_direction << "; " << "fov: " << fov << endl;
          continue;
        }

      if(scan_msg->ranges[*it] > first_detection_depth_thre_)
        {
          if(verbose_) cout << "eliminate: depth: " << scan_msg->ranges[*it] << endl;
          continue;
        }

      diff = fabs(laser_direction - color_region_direction);

      if(verbose_)
        {
          cout << "tree index: " << *it << endl;
          cout << "tree angle: " << *it * scan_msg->angle_increment + scan_msg->angle_min << endl;
          cout << "diff: " << diff << endl;
        }

      if(diff < min_diff)
        {
          min_diff = diff;
          target_tree_index = *it;
        }
    }

    /* Check whether the tree and color region is closed enough */
    if(min_diff > color_region_tree_clustering_angle_diff_thre_)
      {
        ROS_INFO_THROTTLE(1,"can not find the target tree, diff: %f", min_diff);
        return;
      }


    /* find the target tree */
    ROS_WARN("vision  detectoion: find the target tree, angle_diff: %f", min_diff);

    geometry_msgs::Vector3Stamped object_direction;
    object_direction.header = scan_msg->header;
    object_direction.vector.x = target_tree_index; //index
    object_direction.vector.y = target_tree_index * scan_msg->angle_increment + scan_msg->angle_min; //direction
    object_direction.vector.z = scan_msg->ranges[target_tree_index]; //distance
    pub_target_direction_.publish(object_direction);

    /* stop the subscribe if necessary */
    if(detect_once_) unsubscribe();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (vision_detection::VisionDetector, nodelet::Nodelet);
