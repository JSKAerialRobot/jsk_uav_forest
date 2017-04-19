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

#include "jsk_uav_forest_perception/tree_detector.h"

TreeDetector::TreeDetector(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
  camera_info_update_(false),
  camera_fx_(0), camera_fy_(0), camera_cx_(0), camera_cy_(0),
  color_region_update_(false),
  color_region_direction_(0),
  detector_from_image_(false),
  target_tree_index_(0),
  target_theta_(0),
  target_dist_(0),
  uav_odom_(0,0,0),
  target_tree_global_location_(0,0,0),
  uav_yaw_(0), uav_roll_(0), uav_pitch_(0)
{
  /* ros param */
  nhp_.param("uav_odom_topic_name", uav_odom_topic_name_, string("uav_odom"));
  nhp_.param("color_region_center_topic_name", color_region_center_topic_name_, string("color_region_center"));
  nhp_.param("camera_info_topic_name", camera_info_topic_name_, string("camera_info"));
  nhp_.param("laser_scan_topic_name", laser_scan_topic_name_, string("scan"));
  nhp_.param("tree_location_topic_name", tree_location_topic_name_, string("tree_location"));
  nhp_.param("tree_global_location_topic_name", tree_global_location_topic_name_, string("tree_global_location"));
  nhp_.param("tree_cluster_topic_name", tree_cluster_topic_name_, string("tree_cluster"));
  nhp_.param("sub_ctrl_srv_topic_name", sub_ctrl_srv_topic_name_, string("sub_control"));

  nhp_.param("clurstering_max_radius", clurstering_max_radius_, 0.1);
  nhp_.param("clurstering_min_points", clurstering_min_points_, 3);
  nhp_.param("tree_cluster_pub", tree_cluster_pub_, true);
  nhp_.param("color_region_tree_clustering_angle_diff_thre", color_region_tree_clustering_angle_diff_thre_, 0.10);
  nhp_.param("target_tree_drift_thre", target_tree_drift_thre_, 0.5);
  nhp_.param("first_detection_depth_thre", first_detection_depth_thre_, 8.0); // [m]
  nhp_.param("uav_tilt_thre", uav_tilt_thre_, 0.17);
  nhp_.param("verbose", verbose_, false);

  pub_tree_location_ = nh_.advertise<geometry_msgs::PointStamped>(tree_location_topic_name_, 1);
  pub_tree_global_location_ = nh_.advertise<geometry_msgs::PointStamped>(tree_global_location_topic_name_, 1);
  pub_tree_cluster_ = nh_.advertise<sensor_msgs::LaserScan>(tree_cluster_topic_name_, 1);
  sub_ctrl_srv_ = nh_.advertiseService(sub_ctrl_srv_topic_name_, &TreeDetector::subControlCallback, this);
}

bool TreeDetector::subControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data)
    {
      ROS_INFO("start subscribe");
      subscribe();
    }
  else
    {
      ROS_INFO("stop subscribe");
      unsubscribe();
    }
  return true;
}

void TreeDetector::subscribe()
{
  sub_laser_scan_ = nh_.subscribe(laser_scan_topic_name_, 1, &TreeDetector::laserScanCallback, this);
  sub_color_region_center_ = nh_.subscribe(color_region_center_topic_name_, 1, &TreeDetector::colorRegionCallback, this);
  sub_camera_info_ = nh_.subscribe(camera_info_topic_name_, 1, &TreeDetector::cameraInfoCallback, this);
  sub_uav_odom_ = nh_.subscribe(uav_odom_topic_name_, 1, &TreeDetector::uavOdomCallback, this);
}

void TreeDetector::unsubscribe()
{
  sub_clustered_laser_scan_.shutdown();
  sub_laser_scan_.shutdown();
  sub_color_region_center_.shutdown();
  sub_camera_info_.shutdown();
  sub_uav_odom_.shutdown();
}

void TreeDetector::colorRegionCallback(const geometry_msgs::PointStampedConstPtr& color_region_center_msg)
{
  if(!camera_info_update_ || color_region_update_) return;

  /* TODO: we need a more robust color region selection method, i.e. histogram with a certaion motion */
  /* we calculate the angle between the x axis(forward) and the line to the color region */
  color_region_direction_ = atan2(-color_region_center_msg->point.x + camera_cx_, camera_fx_);
  color_region_update_ = true;

  if(verbose_)
    {
      cout << "camera_cx"  << camera_cx_ << endl;
      cout << "color_region_center_msg->point.x"  << color_region_center_msg->point.x << endl;
      cout << "color regio direction"  << color_region_direction_ << endl;
    }
}

void TreeDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  camera_info_update_ = true;
  camera_fx_ = camera_info->K[0];
  camera_fy_ = camera_info->K[4];
  camera_cx_ = camera_info->K[2];
  camera_cy_ = camera_info->K[5];
}

void TreeDetector::uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg)
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

void TreeDetector::laserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_msg)
{
  /* 1. Do the tree clustering based on the simple radius checking algorithm */
  /* 2-a. Find the most closed tree to the color region, record the target tree index */
  /* 2-b. Find the most closed tree to previos target tree index, update the target tree index */
  /* TODO: we need a more tree detection method, i.e. particle filter */
  /* tree clustering */
  vector<int> cluster_index;
  treeClustering(*laser_msg, cluster_index);

  if(!color_region_update_) return;

  if(verbose_)
    ROS_INFO("receive new laser scan");

  /* find the tree most close to the color region */
  float min_diff = 1e6;
  int target_tree_index = target_tree_index_;
  tf::Vector3 target_tree_global_location;

  for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
    {
      float diff = 0;
      tf::Vector3 tree_global_location;

      if(!detector_from_image_)
        {/* we only use the result of color_region_direction at begin, and focus within the fov of the camera image */
          float laser_direction = *it * laser_msg->angle_increment + laser_msg->angle_min;
          float fov = atan2(camera_cx_, camera_fx_); //proximate the width of image with camera_cx_
          if(fabs(laser_direction) > fov)
            {
              if(verbose_) cout << "eliminate: laser_direction:" << laser_direction << "; " << "fov: " << fov << endl;
              continue;
            }

          if(laser_msg->ranges[*it] > first_detection_depth_thre_)
            {
              if(verbose_) cout << "eliminate: depth: " << laser_msg->ranges[*it] << endl;
              continue;
            }

          diff = fabs(laser_direction - color_region_direction_);
        }
      else
        {/* calculate the distance  */
          tf::Matrix3x3 rotation;
          rotation.setRPY(0, 0, *it * laser_msg->angle_increment + laser_msg->angle_min + uav_yaw_);
          tree_global_location = uav_odom_ + rotation * tf::Vector3(laser_msg->ranges[*it], 0, 0);
          diff = (target_tree_global_location_ - tree_global_location).length();
        }

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
          if(detector_from_image_) target_tree_global_location = tree_global_location;
        }
    }

  /* update */
  if(!detector_from_image_)
    {
      if(min_diff < color_region_tree_clustering_angle_diff_thre_)
        {
          ROS_WARN("tree detector: find the target tree, angle_diff: %f", min_diff);
          detector_from_image_ = true;
          tf::Matrix3x3 rotation;
          rotation.setRPY(0, 0, target_tree_index * laser_msg->angle_increment + laser_msg->angle_min + uav_yaw_);
          target_tree_global_location_ = uav_odom_ + rotation * tf::Vector3(laser_msg->ranges[target_tree_index], 0, 0);
        }
      else
        {
          ROS_INFO_THROTTLE(1,"can not find the target tree, diff: %f", min_diff);
        }
    }
  else
    {
      /* outlier check method */
      /* we only update the tree location, if the metric drift compared with previous location is within certain threshold, that is target_tree_drift_thre_ and uav_tilt_thre_. This can avoid, to some degree, the dramatic laser scan change due to the rapid attitude tilting of uav, especially the forward and backward movement by ptich tilting, along with the case that target tree hides behind other objects. We also believe that the laser scan can recover after being back to level, and the new target(tree) location has a small offset with the previous valid target location */
      /* TODO: we need a more robust outlier check method, i.e. the dynamic threshold based on the tilt angle and distance to the target */

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


void TreeDetector::treeClustering(const sensor_msgs::LaserScan& scan_in, vector<int>& cluster_index)
{
  /* copy across all data first */
  sensor_msgs::LaserScan scan_out = scan_in;
  cluster_index.resize(0);

  std::set<int> indices_to_publish;
  /* assume that all points is pass thorugh shadow filter, so each blob is separeted by invalide scan data */
  std::vector<std::vector<int> > range_blobs;
  std::vector<int> range_blob;
  for (unsigned int i = 0; i < scan_in.ranges.size (); i++)
    {
      scan_out.ranges[i] = -1.0 * fabs(scan_in.ranges[i]); // set all ranges to invalid (*)
      if ( scan_in.ranges[i] < 0 || std::isnan(scan_in.ranges[i])) {
        if ( range_blob.size() > clurstering_min_points_ ) {
          range_blobs.push_back(range_blob);
        }
        range_blob.clear();
      }else{
        range_blob.push_back(i);
      }
    }
  if ( range_blob.size() > clurstering_min_points_ ) {
    range_blobs.push_back(range_blob);
  }

  /* for each blob calculate center and radius */
  for (unsigned int i = 0; i < range_blobs.size(); i++) {
    int size = range_blobs[i].size();
#if 0 // previous radisu calculation, has some problem
    /* check center of blob */
    double center_x = 0, center_y = 0;
    for (unsigned int j = 0; j < size; j++) {
      double x = scan_in.ranges[range_blobs[i][j]];
      double y = scan_in.ranges[range_blobs[i][j]] * scan_in.angle_increment;
      center_x += x;
      center_y += y;
    }
    center_x /= size;
    center_y /= size;

    /* check range of blob */
    double radius = 0;
    for (unsigned int j = 0; j < size; j++) {
      double x = scan_in.ranges[range_blobs[i][j]];
      double y = scan_in.ranges[range_blobs[i][j]] * scan_in.angle_increment;
      if ( radius < fabs(center_x - x) ) radius = fabs(center_x - x) ;
      if ( radius < fabs(center_y - y) ) radius = fabs(center_y - y) ;
    }
#else // very rough proximation
    float angle = (size - 1) * scan_in.angle_increment;
    float length = 0;
    float radius = 0;
    for (unsigned int j = 0; j < size - 1; j++) {
      float x_curr = scan_in.ranges[range_blobs[i][j]];
      float x_next = scan_in.ranges[range_blobs[i][j + 1]]; // cos(scan_in.angle_increment) = 1
      float y_next = scan_in.ranges[range_blobs[i][j + 1]] * scan_in.angle_increment; // sin(scan_in.angle_increment) = scan_in.angle_increment
      length += sqrt((x_curr - x_next) * (x_curr - x_next) + y_next * y_next);
    }
    radius = length / (M_PI - angle);
    if(verbose_)
      {
        cout << i << ": direction: " << (range_blobs[i][0] + size/2) * scan_in.angle_increment + scan_in.angle_min
             << "[rad]; distance: " << scan_in.ranges[range_blobs[i][0] + size/2]
             << "[m]; size: " << size << "; radius: "
             << radius << "[m]; length: " << length << "[m]; angle: " << angle << "[rad]" << endl;
      }
#endif

    if ( radius < clurstering_max_radius_ ) {
      indices_to_publish.insert(range_blobs[i][0] + size/2);
    }
  }

  for ( std::set<int>::iterator it = indices_to_publish.begin(); it != indices_to_publish.end(); ++it)
    {
      scan_out.ranges[*it] = fabs(scan_in.ranges[*it]); // valid only the ranges that passwd the test (*)
      cluster_index.push_back(*it);
    }

  if(tree_cluster_pub_) pub_tree_cluster_.publish(scan_out);
}
