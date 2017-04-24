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

#include "jsk_uav_forest_perception/tree_tracking.h"

TreeTracking::TreeTracking(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
  tree_db_(nh, nhp),
  search_center_(0,0,0), uav_odom_(0,0,0),
  uav_yaw_(0), uav_roll_(0), uav_pitch_(0)
{
  /* ros param */
  nhp_.param("uav_odom_topic_name", uav_odom_topic_name_, string("uav_odom"));
  nhp_.param("vision_detection_topic_name", vision_detection_topic_name_, string("/object_direction"));
  nhp_.param("laser_scan_topic_name", laser_scan_topic_name_, string("scan"));
  nhp_.param("tree_location_topic_name", tree_location_topic_name_, string("tree_location"));
  nhp_.param("tree_global_location_topic_name", tree_global_location_topic_name_, string("tree_global_location"));
  nhp_.param("stop_detection_topic_name", stop_detection_topic_name_, string("/detection_start"));

  nhp_.param("uav_tilt_thre", uav_tilt_thre_, 0.17); //[rad] = 10[deg]
  nhp_.param("search_radius", search_radius_, 10.0); // 12[m]
  nhp_.param("only_target", only_target_, false);
  nhp_.param("verbose", verbose_, false);

  sub_vision_detection_ = nh_.subscribe(vision_detection_topic_name_, 1, &TreeTracking::visionDetectionCallback, this);
  sub_uav_odom_ = nh_.subscribe(uav_odom_topic_name_, 1, &TreeTracking::uavOdomCallback, this);

  pub_tree_location_ = nh_.advertise<geometry_msgs::PointStamped>(tree_location_topic_name_, 1);
  pub_tree_global_location_ = nh_.advertise<geometry_msgs::PointStamped>(tree_global_location_topic_name_, 1);
  pub_stop_vision_detection_ = nh_.advertise<std_msgs::Bool>(stop_detection_topic_name_, 1);
}

void TreeTracking::visionDetectionCallback(const geometry_msgs::Vector3StampedConstPtr& vision_detection_msg)
{
  ROS_WARN("tree tracking: start tracking");
  /* stop the vision detection */
  std_msgs::Bool stop_msg;
  stop_msg.data = false;
  pub_stop_vision_detection_.publish(stop_msg);

  /*
    ROS_WARN("tree tracking: start tracking, angle_diff: %f, vision index: %d, laser index: %d, vision dist: %f, laser dist: %f", min_diff, (int)vision_detection_msg->vector.x, target_tree_index, vision_detection_msg->vector.z, laser_msg->ranges[target_tree_index]);
  */

  tf::Matrix3x3 rotation;
  rotation.setRPY(0, 0, vision_detection_msg->vector.y + uav_yaw_);
  tf::Vector3 target_tree_global_location = uav_odom_ + rotation * tf::Vector3(vision_detection_msg->vector.z, 0, 0);

  /* start laesr-only subscribe */
  sub_laser_scan_ = nh_.subscribe(laser_scan_topic_name_, 1, &TreeTracking::laserScanCallback, this);

  /* add target tree to the tree data base */
  TreeHandlePtr new_tree = TreeHandlePtr(new TreeHandle(target_tree_global_location));
  tree_db_.add(new_tree);
  target_tree_ = new_tree;

  /* set the search center as the first target tree(with color marker) pos */
  search_center_ = target_tree_global_location;

  sub_vision_detection_.shutdown(); //stop

}


void TreeTracking::uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg)
{
  tf::Quaternion uav_q(uav_msg->pose.pose.orientation.x,
                       uav_msg->pose.pose.orientation.y,
                       uav_msg->pose.pose.orientation.z,
                       uav_msg->pose.pose.orientation.w);
  tf::Matrix3x3  uav_orientation_(uav_q);
  tfScalar r,p,y;
  uav_orientation_.getRPY(r, p, y);
  uav_odom_.setX(uav_msg->pose.pose.position.x);
  uav_odom_.setY(uav_msg->pose.pose.position.y);
  uav_odom_.setZ(uav_msg->pose.pose.position.z);
  uav_roll_ = r; uav_pitch_ = p; uav_yaw_ = y;
}

void TreeTracking::laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  if(verbose_) ROS_INFO("receive new laser scan");

  /* extract the cluster */
  vector<int> cluster_index;
  for (size_t i = 0; i < scan_msg->ranges.size(); i++)
      if(scan_msg->ranges[i] > 0) cluster_index.push_back(i);

  /* find the tree most close to the previous target tree */
  int target_tree_index = 0;
  bool target_update = false;
  for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
    {
      /* we do not update trees pos if there is big tilt */
      if(fabs(uav_pitch_) > uav_tilt_thre_)
        {
          ROS_WARN("Too much tilt: %f", uav_pitch_);
          break;
        }

      tf::Vector3 tree_global_location;

      /* calculate the distance  */
      tf::Matrix3x3 rotation;
      rotation.setRPY(0, 0, *it * scan_msg->angle_increment + scan_msg->angle_min + uav_yaw_);
      tree_global_location = uav_odom_ + rotation * tf::Vector3(scan_msg->ranges[*it], 0, 0);

      /* omit, if the tree is not within the search area */
      if((search_center_ - tree_global_location).length() > search_radius_)
        {
          if(verbose_)
            cout << "tree [" << tree_global_location.x() << ", " << tree_global_location.y() << "] is to far from the search center [" << search_center_.x() << ", " << search_center_.y() << "]" << endl;
          continue;
        }

      /* add tree to the database */
      if(verbose_) cout << "Scan input tree No." << distance(cluster_index.begin(), it) << ": start update" << endl;
      target_update += tree_db_.update(tree_global_location, only_target_);
    }

  tf::Vector3 target_tree_global_location = target_tree_->getPos();
  tf::Matrix3x3 rotation;
  rotation.setRPY(0, 0, -uav_yaw_);
  tf::Vector3 target_tree_local_location = rotation * (target_tree_global_location - uav_odom_);
  if(!target_update && only_target_)
    ROS_WARN("lost the target tree: [%f, %f]", target_tree_global_location.x(), target_tree_global_location.y());

  if(verbose_) ROS_INFO("target_tree_global_location: [%f, %f]", target_tree_global_location.x(), target_tree_global_location.y());

  /* publish the location of the target tree */
  geometry_msgs::PointStamped target_msg;
  target_msg.header = scan_msg->header;
  target_msg.point.x = target_tree_local_location.x();
  target_msg.point.y = target_tree_local_location.y();
  pub_tree_location_.publish(target_msg);
  geometry_msgs::PointStamped target_global_msg;
  target_global_msg.header = scan_msg->header;
  target_global_msg.point.x = target_tree_global_location.x();
  target_global_msg.point.y = target_tree_global_location.y();
  pub_tree_global_location_.publish(target_global_msg);
}

