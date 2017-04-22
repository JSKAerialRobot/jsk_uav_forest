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


#ifndef VISION_DETECTOR_BASE_PLUGIN_H_
#define VISION_DETECTOR_BASE_PLUGIN_H_

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
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> SyncPolicy;

  class DetectorBase
  {
  public:
    DetectorBase():
      camera_fx_(-1), camera_fy_(-1), camera_cx_(-1), camera_cy_(-1),
      scan_angle_increment_(0), scan_angle_min_(0)
    {
    }

    ~DetectorBase(){}

    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle pnh)
    {
      nh_ = nh;
      pnh_ = pnh;

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

      pnh_.param("first_detection_depth_thre", first_detection_depth_thre_, 9.0); // [m]

      /* ros pub and sub */
      sub_image_.subscribe(nh_, image_topic_name_, 1);
      sub_scan_.subscribe(nh_, laser_scan_topic_name_, 1);
      unsubscribe();
      sync_ = boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10)));
      sync_->connectInput(sub_image_, sub_scan_);
      sync_->registerCallback(&DetectorBase::cameraScanCallback, this);

      sub_detection_start_ = nh_.subscribe(detection_start_topic_name_, 1, &DetectorBase::startDetectionCallback, this);
      sub_camera_info_ = nh_.subscribe(camera_info_topic_name_, 1, &DetectorBase::cameraInfoCallback, this);

      pub_target_image_ = nh_.advertise<sensor_msgs::Image>(target_image_topic_name_, 1);
      pub_target_direction_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/object_direction", 1);
      pub_target_image_center_ = nh_.advertise<geometry_msgs::PointStamped>("/object_image_center", 1);
    }

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    boost::shared_ptr< message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_;

    ros::Subscriber sub_camera_info_;
    ros::Subscriber sub_detection_start_;
    ros::Publisher pub_target_image_;
    ros::Publisher pub_target_direction_;
    ros::Publisher pub_target_image_center_;

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

    double first_detection_depth_thre_;

    /* base var */
    float scan_angle_increment_, scan_angle_min_;
    double camera_fx_, camera_fy_, camera_cx_, camera_cy_;

    void cameraScanCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg)
    {
      /* debug */
      ROS_WARN("camera scan vs laser scan time diff: %f", (image_msg->header.stamp.toSec() - scan_msg->header.stamp.toSec()));

      /* check whether the camera info is updated */
      if(camera_fx_ < -1) return;

      /* do the specific filter process */
      int target_tree_index = 0;
      if(!filter(image_msg, scan_msg, target_tree_index)) return;

      /* publish the result */
      geometry_msgs::Vector3Stamped object_direction;
      object_direction.header = scan_msg->header;
      object_direction.vector.x = target_tree_index; //index
      object_direction.vector.y = target_tree_index * scan_msg->angle_increment + scan_msg->angle_min; //direction
      object_direction.vector.z = scan_msg->ranges[target_tree_index]; //distance
      pub_target_direction_.publish(object_direction);

      /* stop the subscribe if necessary */
      if(detect_once_) unsubscribe();
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
      camera_fx_ = camera_info->K[0];
      camera_fy_ = camera_info->K[4];
      camera_cx_ = camera_info->K[2];
      camera_cy_ = camera_info->K[5];
    }
    void startDetectionCallback(const std_msgs::BoolConstPtr& msg)
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

    void laserClustering(sensor_msgs::LaserScan scan_in, vector<int>& cluster_index)
    {
      cluster_index.resize(0);
      /* extract the cluster */
      for (size_t i = 0; i < scan_in.ranges.size(); i++)
        if(scan_in.ranges[i] > 0) cluster_index.push_back(i);
    }

    virtual bool filter(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg, int& target_tree_index) = 0;
  };
}

#endif
