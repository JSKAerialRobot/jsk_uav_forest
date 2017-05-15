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

#ifndef TREE_TRACKING_H_
#define TREE_TRACKING_H_

/* ros */
#include <ros/ros.h>

/* ros msg/srv */
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

/* tree database */
#include <jsk_uav_forest_perception/tree_database.h>

using namespace std;

class TreeTracking
{
public:
  TreeTracking(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~TreeTracking(){}

private:
  ros::NodeHandle nh_, nhp_;

  ros::Subscriber sub_vision_detection_;
  ros::Subscriber sub_uav_odom_;
  ros::Subscriber sub_laser_scan_;
  ros::Subscriber sub_tracking_control_;

  ros::Publisher pub_stop_vision_detection_;
  ros::Publisher pub_tree_location_;
  ros::Publisher pub_tree_global_location_;

  ros::ServiceServer update_target_tree_srv_;
  ros::ServiceServer set_first_tree_srv_;

  string uav_odom_topic_name_;
  string laser_scan_topic_name_;
  string vision_detection_topic_name_;
  string tree_location_topic_name_;
  string tree_global_location_topic_name_;
  string tree_cluster_topic_name_;
  string stop_detection_topic_name_;
  string tracking_control_topic_name_;
  string update_target_tree_srv_name_;
  string set_first_tree_srv_name_;

  double uav_tilt_thre_;
  double search_radius_;
  double max_orthogonal_dist_; /* for deep searching method */
  bool only_target_;
  bool verbose_;
  bool visualization_;
  bool tree_circle_fitting_;
  double tree_scan_angle_thre_;
  double tree_circle_regulation_thre_;

  TreeDataBase tree_db_;
  vector<TreeHandlePtr> target_trees_;

  tf::Vector3 search_center_;
  tf::Vector3 uav_odom_;
  float uav_roll_, uav_pitch_, uav_yaw_;
  tf::Vector3 initial_target_tree_direction_vec_;

  //temp
  double tree_radius_max_, tree_radius_min_;

  void subscribe();
  void unsubscribe();

  void visionDetectionCallback(const geometry_msgs::Vector3StampedConstPtr& vision_detection_msg);
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
  void uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg);
  void trackingControlCallback(const std_msgs::BoolConstPtr& msg);
  bool updateTargetTreeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool setFirstTreeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool searchTargetTreeFromDatabase();
};

#endif
