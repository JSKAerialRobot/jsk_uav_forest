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

#ifndef TREE_DETECTOR_H_
#define TREE_DETECTOR_H_

/* ros */
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/* ros msg/srv */
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class TreeDetector
{
public:
  TreeDetector(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~TreeDetector(){}

private:
  ros::NodeHandle nh_, nhp_;
  ros::Subscriber sub_uav_odom_;
  ros::Subscriber sub_color_region_center_;
  ros::Subscriber sub_camera_info_;
  ros::Subscriber sub_clustered_laser_scan_;
  ros::Subscriber sub_laser_scan_;
  ros::Publisher pub_tree_location_;
  ros::Publisher pub_tree_cluster_;
  ros::ServiceServer sub_ctrl_srv_;

  string uav_odom_topic_name_;
  string color_region_center_topic_name_;
  string camera_info_topic_name_;
  string laser_scan_topic_name_;
  string tree_location_topic_name_;
  string tree_cluster_topic_name_;
  string sub_ctrl_srv_topic_name_;
  bool tree_cluster_pub_;
  double color_region_tree_clustering_angle_diff_thre_;
  double target_tree_drift_thre_;
  double uav_tilt_thre_;
  bool verbose_;
  
  bool camera_info_update_;
  bool color_region_update_;
  bool detector_from_image_;

  double camera_fx_, camera_fy_, camera_cx_, camera_cy_;
  double color_region_direction_;
  double clurstering_max_radius_;
  int clurstering_min_points_;
  int target_tree_index_;
  double target_theta_, target_dist_;
  tf::Vector3 uav_odom_;
  float uav_roll_, uav_pitch_, uav_yaw_;
  tf::Vector3 target_tree_global_location_;

  void subscribe();
  void unsubscribe();

  void treeClustering(const sensor_msgs::LaserScan& scan_in, vector<int>& cluster_index);
  void circleFitting();

  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_msg);
  void colorRegionCallback(const geometry_msgs::PointStampedConstPtr& color_region_center_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
  void uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg);
  bool subControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
};

#endif
