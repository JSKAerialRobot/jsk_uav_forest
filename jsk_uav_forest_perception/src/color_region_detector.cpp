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


#include "jsk_uav_forest_perception/color_region_detector.h"

namespace color_region_detection
{
  ColorRegionDetector::~ColorRegionDetector()
  {

  }

  void ColorRegionDetector::onInit()
  {
    pnh_ = this->getPrivateNodeHandle();

    /* ros param */
    pnh_.param("image_topic_name", image_topic_name_, std::string("/camera/image_rect"));
    pnh_.param("out_image_topic_name", out_image_topic_name_, std::string("/target"));
    pnh_.param("camera_info_topic_name", camera_info_topic_name_, std::string("/camera_info"));

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
    pub_target_image_ = pnh_.advertise<sensor_msgs::Image>(out_image_topic_name_, 1);
    pub_target_image_center_ = pnh_.advertise<geometry_msgs::PointStamped>("/object_image_center", 1);

    sub_camera_image_ = pnh_.subscribe<sensor_msgs::Image>(image_topic_name_, 1, &ColorRegionDetector::cameraDataCallback, this);
    sub_camera_info_ = pnh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_name_, 1, &ColorRegionDetector::cameraInfoCallback, this);

    /* ros service */
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ColorRegionDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void ColorRegionDetector::configCallback(Config &new_config, uint32_t level)
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

  void ColorRegionDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
  {
    if(camera_info_update_) return;
    camera_fx_ = camera_info->K[0];
    camera_fy_ = camera_info->K[4];

    if(camera_info->K[0] > 0)
      {
        camera_info_update_ = true;
      }
  }

  void ColorRegionDetector::cameraDataCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    cv::Mat src_image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

    /* hsv filter */
    cv::Mat hsv_image, hsv_image_mask;
    if(image_msg->encoding == sensor_msgs::image_encodings::RGB8)
      {
	cv::cvtColor(src_image, hsv_image, CV_RGB2HSV);
      }
    else if(image_msg->encoding == sensor_msgs::image_encodings::BGR8)
      {
	cv::cvtColor(src_image, hsv_image, CV_BGR2HSV);
      }

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
    if(contours.size() > 0)
      {
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
      }
    else
      pub_target_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                 sensor_msgs::image_encodings::MONO8,
                                                 hsv_image_mask).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (color_region_detection::ColorRegionDetector, nodelet::Nodelet);
