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


#ifndef VISION_DETECTOR_H_
#define VISION_DETECTOR_H_

/* ros */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

/* message filter */
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* cfg */
#include <dynamic_reconfigure/server.h>
#include <jsk_uav_forest_perception/HsvFilterConfig.h>

/* ros msg */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>

/* cv */
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/* standard */
#include <iostream>
#include <vector>

using namespace std;

namespace vision_detection
{
  class VisionDetector: public nodelet::Nodelet
  {
  public:
    VisionDetector();
    ~VisionDetector();
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> SyncPolicy;
    typedef jsk_uav_forest_perception::HsvFilterConfig Config;

  private:
    ros::NodeHandle pnh_;
    boost::shared_ptr< message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_;

    ros::Subscriber sub_camera_info_;
    ros::Subscriber sub_detection_start_;
    ros::Publisher pub_target_image_;
    ros::Publisher pub_target_direction_;
    ros::Publisher pub_target_image_center_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    /* rosparam */
    string image_topic_name_;
    string camera_info_topic_name_;
    string laser_scan_topic_name_;
    string target_image_topic_name_;
    string target_direction_topic_name_;
    string target_image_center_topic_name_;
    string detection_start_topic_name_;

    bool detect_once_;
    bool verbose_;

    int hsv_lower_bound_[3];
    int hsv_upper_bound_[3];
    int contour_color_r_, contour_color_g_, contour_color_b_;
    cv::Scalar contour_color_;
    double color_region_tree_clustering_angle_diff_thre_;
    double first_detection_depth_thre_;

    /* base var */
    double camera_fx_, camera_fy_, camera_cx_, camera_cy_;

    virtual void onInit();
    void configCallback(Config &new_config, uint32_t level);

    void cameraScanCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
    void startDetectionCallback(const std_msgs::BoolConstPtr& msg);

    void subscribe()
    {
      sub_image_.subscribe(); //start
      sub_scan_.subscribe(); //start
    }

    void unsubscribe()
    {
      sub_image_.unsubscribe(); //stop
      sub_scan_.unsubscribe(); //stop
    }

  };
}

#endif
