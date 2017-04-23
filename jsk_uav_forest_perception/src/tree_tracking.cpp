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
  target_theta_(0),
  target_dist_(0),
  uav_odom_(0,0,0),
  target_tree_global_location_(0,0,0),
  uav_yaw_(0), uav_roll_(0), uav_pitch_(0)
{
  /* ros param */
  nhp_.param("uav_odom_topic_name", uav_odom_topic_name_, string("uav_odom"));
  nhp_.param("vision_detection_topic_name", vision_detection_topic_name_, string("/object_direction"));
  nhp_.param("laser_scan_topic_name", laser_scan_topic_name_, string("scan"));
  nhp_.param("tree_location_topic_name", tree_location_topic_name_, string("tree_location"));
  nhp_.param("tree_global_location_topic_name", tree_global_location_topic_name_, string("tree_global_location"));

  nhp_.param("target_tree_drift_thre", target_tree_drift_thre_, 0.5);
  nhp_.param("uav_tilt_thre", uav_tilt_thre_, 0.17);
  nhp_.param("verbose", verbose_, false);

  sub_sync_scan_.subscribe(nhp_, laser_scan_topic_name_, 20, ros::TransportHints().tcpNoDelay());
  sub_sync_vision_detection_.subscribe(nhp_, vision_detection_topic_name_, 10, ros::TransportHints().tcpNoDelay());

  sync_ = boost::shared_ptr<SyncPolicy >(new SyncPolicy(sub_sync_scan_, sub_sync_vision_detection_, 100));
  sync_->registerCallback(&TreeTracking::visionDetectionCallback, this);

  sub_uav_odom_ = nh_.subscribe(uav_odom_topic_name_, 1, &TreeTracking::uavOdomCallback, this);

  pub_tree_location_ = nh_.advertise<geometry_msgs::PointStamped>(tree_location_topic_name_, 1);
  pub_tree_global_location_ = nh_.advertise<geometry_msgs::PointStamped>(tree_global_location_topic_name_, 1);
}


void TreeTracking::visionDetectionCallback(const sensor_msgs::LaserScanConstPtr& laser_msg, const geometry_msgs::Vector3StampedConstPtr& vision_detection_msg)
{
  ROS_INFO("receive  vision detection result, time diff: %f", laser_msg->header.stamp.toSec() - vision_detection_msg->header.stamp.toSec());

  //ROS_WARN("tree tracking: start tracking, angle_diff: %f, vision index: %d, laser index: %d, vision dist: %f, laser dist: %f", min_diff, (int)vision_detection_msg->vector.x, target_tree_index, vision_detection_msg->vector.z, laser_msg->ranges[target_tree_index]);
  tf::Matrix3x3 rotation;
  rotation.setRPY(0, 0, vision_detection_msg->vector.y + uav_yaw_);
  target_tree_global_location_ = uav_odom_ + rotation * tf::Vector3(vision_detection_msg->vector.z, 0, 0);

  /* stop the synchronized subscribe and start laesr-only subscribe */
  sub_laser_scan_ = nh_.subscribe(laser_scan_topic_name_, 1, &TreeTracking::laserScanCallback, this);

  sub_sync_vision_detection_.unsubscribe(); //stop
  sub_sync_scan_.unsubscribe(); //stop
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

/* 1. Do the tree clustering based on the simple radius checking algorithm */
/* 2. Find the most closed tree to previos target tree index, update the target tree index */
void TreeTracking::laserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_msg)
{
  if(verbose_) ROS_INFO("receive new laser scan");

  /* tree clustering */
  vector<int> cluster_index;
  /* extract the cluster */
  for (size_t i = 0; i < laser_msg->ranges.size(); i++)
    if(laser_msg->ranges[i] > 0) cluster_index.push_back(i);

  /* find the tree most close to the previous target tree */
  float min_diff = 1e6;
  int target_tree_index = 0;
  tf::Vector3 target_tree_global_location;

  for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
    {
      float diff = 0;
      tf::Vector3 tree_global_location;

      /* calculate the distance  */
      tf::Matrix3x3 rotation;
      rotation.setRPY(0, 0, *it * laser_msg->angle_increment + laser_msg->angle_min + uav_yaw_);
      tree_global_location = uav_odom_ + rotation * tf::Vector3(laser_msg->ranges[*it], 0, 0);
      diff = (target_tree_global_location_ - tree_global_location).length();

      if(verbose_)
        {
          cout << "tree index: " << *it << endl;
          cout << "tree angle: " << *it * laser_msg->angle_increment + laser_msg->angle_min << endl;
          cout << "diff: " << diff << endl;
        }

      if(diff < min_diff)
        {
          min_diff = diff;
          target_tree_index = *it;
          target_tree_global_location = tree_global_location;
        }
    }

  /* update */
  if(fabs(uav_pitch_) < uav_tilt_thre_)
    {
      float drift = (target_tree_global_location_ - target_tree_global_location).length();
      if(verbose_) cout << "drift: " << drift << endl;
      if(drift < target_tree_drift_thre_)
        {
          target_tree_global_location_ = target_tree_global_location;
        }
      else
        {
          ROS_WARN("lost the target tree, drift: %f, the nearest target location: [%f, %f], prev target location: [%f, %f]", drift, target_tree_global_location.x(), target_tree_global_location.y(), target_tree_global_location_.x(), target_tree_global_location_.y());
        }
    }
  else
    {
      ROS_WARN("Too much tilt: %f", uav_pitch_);
    }


  if(verbose_)
    ROS_INFO("target_tree_global_location_: [%f, %f]", target_tree_global_location_.x(), target_tree_global_location_.y());

  /* publish the location of the target tree */
  geometry_msgs::PointStamped target_msg;
  target_msg.header = laser_msg->header;
  tf::Matrix3x3 rotation;
  rotation.setRPY(0, 0, -uav_yaw_);
  tf::Vector3 target_tree_local_location = rotation * (target_tree_global_location_ - uav_odom_);
  target_msg.point.x = target_tree_local_location.x();
  target_msg.point.y = target_tree_local_location.y();
  pub_tree_location_.publish(target_msg);
  geometry_msgs::PointStamped target_global_msg;
  target_global_msg.header = laser_msg->header;
  target_global_msg.point.x = target_tree_global_location_.x();
  target_global_msg.point.y = target_tree_global_location_.y();
  pub_tree_global_location_.publish(target_global_msg);

  /* TODO: we need record method for multiple trees using the odom of UAV */
}

