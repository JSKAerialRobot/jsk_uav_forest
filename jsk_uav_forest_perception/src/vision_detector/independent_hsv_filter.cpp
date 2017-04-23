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

/* base class */
#include <jsk_uav_forest_perception/vision_detector/independent_hsv_filter.h>

using namespace std;

namespace vision_detection
{
  void IndependentHsvFilter::initialize(ros::NodeHandle nh, ros::NodeHandle pnh)
  {
    vision_detection::DetectorBase::initialize(nh, pnh);

    pnh_.param("color_region_tree_clustering_angle_diff_thre", color_region_tree_clustering_angle_diff_thre_, 0.10);

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


    /* ros service */
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&IndependentHsvFilter::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void IndependentHsvFilter::configCallback(Config &new_config, uint32_t level)
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

  void IndependentHsvFilter::hsvFilter(cv::Mat src_image, string encoding, cv::Mat& hsv_image_mask)
  {
    cv::Mat hsv_image;
    if(encoding == sensor_msgs::image_encodings::RGB8)
      cv::cvtColor(src_image, hsv_image, CV_RGB2HSV);
    else if(encoding == sensor_msgs::image_encodings::BGR8)
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
  }

  bool IndependentHsvFilter::filter(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg, int& target_tree_index)
  {
    cv::Mat src_image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

    /* general hsv filter */
    cv::Mat hsv_image_mask;
    hsvFilter(src_image, image_msg->encoding, hsv_image_mask);

    /* find countour */
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(hsv_image_mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    if(contours.size() == 0) return false;

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

    vector<int> cluster_index;
    laserClustering(*scan_msg, cluster_index, src_image);

    for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
      {
        float diff = 0;
        float laser_direction = *it * scan_msg->angle_increment + scan_msg->angle_min;

        /*
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
        */
        if(!commonFiltering(laser_direction, fov, scan_msg->ranges[*it])) continue;

        diff = fabs(laser_direction - color_region_direction);

        if(verbose_)
          {
            cout << "tree index: " << *it << endl;
            cout << "tree angle: " << laser_direction << endl;
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
        ROS_INFO_THROTTLE(1,"closest tree is far from the color region, can not find the target tree, diff: %f", min_diff);
        return false;
      }

    /* find the target tree */
    ROS_WARN("independent hsv filter based vision detectoion: find the target tree, angle_diff: %f", min_diff);
    return true;
  }

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_detection::IndependentHsvFilter, vision_detection::DetectorBase);

