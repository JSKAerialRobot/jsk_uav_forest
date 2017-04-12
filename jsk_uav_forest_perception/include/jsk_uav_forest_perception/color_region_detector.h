// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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


#ifndef COLOR_REGION_DETECTOR_H_
#define COLOR_REGION_DETECTOR_H_

/* ros */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

/* cfg */
#include <dynamic_reconfigure/server.h>
#include <jsk_uav_forest_perception/ColorRegionDetectorConfig.h>

/* ros msg */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

/* cv */
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/* standard */
#include <iostream>
#include <vector>

namespace color_region_detection
{
  class ColorRegionDetector: public nodelet::Nodelet
  {
  public:
    ColorRegionDetector(){}
    ~ColorRegionDetector();
    typedef jsk_uav_forest_perception::ColorRegionDetectorConfig Config;

  private:
    ros::NodeHandle pnh_;
    ros::Subscriber sub_camera_image_;
    ros::Subscriber sub_camera_info_;
    ros::Publisher pub_target_image_;
    ros::Publisher pub_target_image_center_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    std::string image_topic_name_;
    std::string out_image_topic_name_;
    std::string camera_info_topic_name_;

    int contour_color_r_, contour_color_g_, contour_color_b_;
    cv::Scalar contour_color_;

    int hsv_lower_bound_[3];
    int hsv_upper_bound_[3];
    bool camera_info_update_;
    double camera_fx_, camera_fy_;
    void configCallback(Config &new_config, uint32_t level);
    virtual void onInit();
    void cameraDataCallback(const sensor_msgs::ImageConstPtr& image_msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
  };
}

#endif
