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
#include "jsk_uav_forest_perception/util/circle_detection.h"

TreeTracking::TreeTracking(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
  tree_db_(nh, nhp),
  search_center_(0,0,0), uav_odom_(0,0,0),
  uav_yaw_(0), uav_roll_(0), uav_pitch_(0),
  start_database_(false)
{
  /* ros param */
  nhp_.param("uav_odom_topic_name", uav_odom_topic_name_, string("uav_odom"));
  nhp_.param("vision_detection_topic_name", vision_detection_topic_name_, string("/object_direction"));
  nhp_.param("database_control_topic_name", database_control_topic_name_, string("/database_control"));
  nhp_.param("laser_scan_topic_name", laser_scan_topic_name_, string("scan"));
  nhp_.param("tree_location_topic_name", tree_location_topic_name_, string("tree_location"));
  nhp_.param("tree_global_location_topic_name", tree_global_location_topic_name_, string("tree_global_location"));
  nhp_.param("tracking_control_topic_name", tracking_control_topic_name_, string("/tracking_control"));
  nhp_.param("stop_detection_topic_name", stop_detection_topic_name_, string("/detection_start"));
  nhp_.param("update_target_tree_srv_name", update_target_tree_srv_name_, string("/update_target_tree"));
  nhp_.param("set_first_tree_srv_name", set_first_tree_srv_name_, string("/set_first_tree"));

  nhp_.param("uav_tilt_thre", uav_tilt_thre_, 0.17); //[rad] = 10[deg]
  nhp_.param("only_target", only_target_, false);
  nhp_.param("verbose", verbose_, false);
  nhp_.param("visualization", visualization_, false);
  nhp_.param("tree_circle_fitting", tree_circle_fitting_, true);
  nhp_.param("urg_yaw_offset", urg_yaw_offset_, 0.0);

  nhp_.param("first_tree_pos_margin", first_tree_pos_margin_, 0.3); //should be positive
  nhp_.param("first_tree_dist_thresh", first_tree_dist_thresh_, 3.0); // 3[m]

  /* tree circle fitting */
  nhp_.param("tree_radius_max", tree_radius_max_, 0.3);
  nhp_.param("tree_radius_min", tree_radius_min_, 0.08);
  nhp_.param("tree_scan_angle_thre", tree_scan_angle_thre_, 0.1);
  nhp_.param("tree_circle_regulation_thre", tree_circle_regulation_thre_, 0.005);

  nhp_.param("searching_method", searching_method_, 1); //1 narrow 2 deep
  /* narrow searching method */
  nhp_.param("narrow_searching_radius", narrow_searching_radius_, 5.0);
  nhp_.param("narrow_angle_diff_min", narrow_angle_diff_min_, 0.1);
  /* deep searching method */
  nhp_.param("max_orthogonal_dist", max_orthogonal_dist_, 3.0);

  sub_vision_detection_ = nh_.subscribe(vision_detection_topic_name_, 1, &TreeTracking::visionDetectionCallback, this);
  sub_uav_odom_ = nh_.subscribe(uav_odom_topic_name_, 1, &TreeTracking::uavOdomCallback, this);
  sub_tracking_control_ = nh_.subscribe(tracking_control_topic_name_, 1, &TreeTracking::trackingControlCallback, this);
  update_target_tree_srv_ = nh_.advertiseService(update_target_tree_srv_name_, &TreeTracking::updateTargetTreeCallback, this);
  set_first_tree_srv_ = nh_.advertiseService(set_first_tree_srv_name_, &TreeTracking::setFirstTreeCallback, this);

  pub_database_control_ = nh_.advertise<std_msgs::Bool>(database_control_topic_name_, 1);
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

  tf::Matrix3x3 rotation;
  rotation.setRPY(0, 0, vision_detection_msg->vector.y + uav_yaw_ + urg_yaw_offset_);
  tf::Vector3 target_tree_global_location = uav_odom_ + rotation * tf::Vector3(vision_detection_msg->vector.z, 0, 0);
  initial_target_tree_direction_vec_ = rotation * tf::Vector3(vision_detection_msg->vector.z, 0, 0);
  initial_target_tree_direction_vec_ /= initial_target_tree_direction_vec_.length(); //normalize

  /* start laesr-only subscribe */
  sub_laser_scan_ = nh_.subscribe(laser_scan_topic_name_, 1, &TreeTracking::laserScanCallback, this);

  /* add target tree to the tree data base */
  TreeHandlePtr new_tree = TreeHandlePtr(new TreeHandle(nh_, nhp_, target_tree_global_location));
  tree_db_.add(new_tree);
  tree_db_.setCenterTree(new_tree);
  target_trees_.resize(0);
  target_trees_.push_back(new_tree);

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

  /* we only consider in the 2D space */
  //uav_odom_.setZ(uav_msg->pose.pose.position.z);
  uav_odom_.setZ(0);
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
  bool target_update = false;
  int prev_vote = target_trees_.back()->getVote();
  for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
    {
      /* we do not update trees pos if there is big tilt */
      if(fabs(uav_pitch_) > uav_tilt_thre_)
        {
          ROS_WARN("Too much tilt: %f", uav_pitch_);
          break;
        }

      tf::Vector3 tree_global_location;

      /* calculate the distance */
      tf::Matrix3x3 rotation;
      rotation.setRPY(0, 0, *it * scan_msg->angle_increment + scan_msg->angle_min + uav_yaw_);
      tree_global_location = uav_odom_ + rotation * tf::Vector3(scan_msg->ranges[*it], 0, 0);

      /* add tree to the database */
      if(verbose_) cout << "Scan input tree No." << distance(cluster_index.begin(), it) << ": start update" << endl;

      /* calc radius with circle fitting */
      if (tree_circle_fitting_)
        {
          vector<tf::Vector3> points;
	  int scan_point_num = 0;
          for (int i = *it; !isnan(scan_msg->ranges[i]) && i < scan_msg->ranges.size(); i++)
            {
              double r = fabs(scan_msg->ranges[i]);
              double theta = scan_msg->angle_min + (scan_msg->angle_increment) * i;
              tf::Vector3 point(r * cos(theta), r * sin(theta), 0);
              points.push_back(point);
	      scan_point_num++;
            }
          for (int i = (*it) - 1; !isnan(scan_msg->ranges[i]) && i > 0; i--)
            {
              double r = fabs(scan_msg->ranges[i]);
              double theta = scan_msg->angle_min + (scan_msg->angle_increment) * i;
              tf::Vector3 point(r * cos(theta), r * sin(theta), 0);
              points.push_back(point);
	      scan_point_num++;
            }
	  /* calc position and radius */
	  tf::Vector3 tree_center_pos; double tree_radius, regulation;
	  CircleDetection::circleFitting(points, tree_center_pos, tree_radius, regulation);
	  /* angle filter */
	  double scan_angle_real = scan_point_num * scan_msg->angle_increment;
	  double scan_angle_virtual = M_PI - 2 * acos(tree_radius / tree_center_pos.length());
	  /* tree position filter */
	  tf::Matrix3x3 rotation; rotation.setRPY(0, 0, uav_yaw_ + urg_yaw_offset_);
	  tf::Vector3 tree_center_global_location = uav_odom_ + rotation * tf::Vector3(tree_center_pos.x(), tree_center_pos.y(), 0);
	  tf::Vector3 initial_target_tree_pos = target_trees_.at(0)->getPos();
	  double projected_length_from_initial_target = initial_target_tree_direction_vec_.dot(tree_center_global_location - initial_target_tree_pos);	  

	  if (tree_radius > tree_radius_min_ && tree_radius < tree_radius_max_ && fabs((scan_angle_real - scan_angle_virtual) / scan_angle_real) < tree_scan_angle_thre_ && regulation < tree_circle_regulation_thre_ && projected_length_from_initial_target > -first_tree_pos_margin_)
            {
               target_update += tree_db_.updateSingleTree(tree_center_global_location, tree_radius, only_target_ || (!start_database_));
            }
          else
            {
              if(verbose_)
                ROS_INFO("radius: %f, min: %f, max: %f", tree_radius, tree_radius_min_, tree_radius_max_);
            }
        }
      else
        {
          target_update += tree_db_.updateSingleTree(tree_global_location, 0.2, only_target_);
        }
    }

  /* update the whole database(sorting) */
  tree_db_.update();

  tf::Vector3 target_tree_global_location = target_trees_.back()->getPos();
  tf::Matrix3x3 rotation;
  rotation.setRPY(0, 0, -uav_yaw_ - urg_yaw_offset_);
  tf::Vector3 target_tree_local_location = rotation * (target_tree_global_location - uav_odom_);
  target_tree_local_location.setZ(0);
  if(prev_vote ==  target_trees_.back()->getVote())
    ROS_WARN_THROTTLE(0.5, "lost the target tree: [%f, %f], prev vote: %d, curr vote: %d", target_tree_global_location.x(), target_tree_global_location.y(), prev_vote, target_trees_.back()->getVote());

  if(verbose_) ROS_INFO("target_tree_global_location: [%f, %f]", target_tree_global_location.x(), target_tree_global_location.y());


  /* check whether close to the target tree for the trigger to start database update */
  if(!start_database_)
    {
      if(target_tree_local_location.length() < first_tree_dist_thresh_)
        {
          ROS_WARN("start update database");
          start_database_ = true;
          std_msgs::Bool start_database;
          start_database.data = true;
          pub_database_control_.publish(start_database);
        }
    }

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

  if (visualization_) {
    tree_db_.visualization(scan_msg->header);
  }
}

bool TreeTracking::searchTargetTreeFromDatabase()
{
  TreeHandlePtr target_tree = NULL; 
  vector<TreeHandlePtr> trees;
  tree_db_.getTrees(trees);
  tf::Matrix3x3 rot_mat_; rot_mat_.setRPY(0, 0, -M_PI / 2);
  tf::Vector3 orthogonal_vec = rot_mat_ * initial_target_tree_direction_vec_;

  if (searching_method_ == 1)
    {
      float min_angle = 1e6;
      tf::Vector3 initial_target_tree_pos = target_trees_.at(0)->getPos() - initial_target_tree_direction_vec_ * first_tree_pos_margin_;
      float previous_target_angle = -0.5;
      if (target_trees_.size() != 1)
        {
          tf::Vector3 previous_target_from_initial_target_vec = target_trees_.back()->getPos() - initial_target_tree_pos;
          previous_target_angle = acos(orthogonal_vec.dot(previous_target_from_initial_target_vec) / previous_target_from_initial_target_vec.length());
        }

      for(vector<TreeHandlePtr>::iterator it = trees.begin(); it != trees.begin() + tree_db_.validTreeNum(); ++it)
	{
	  if (find(target_trees_.begin(), target_trees_.end(), *it)  != target_trees_.end())
	    {
	      //cout << "ignore: previous targe tree" << endl;
	      continue;
	    }
	  tf::Vector3 from_initial_target_vec = (*it)->getPos() - initial_target_tree_pos;
	  float distance_from_initial_target = initial_target_tree_direction_vec_.dot(from_initial_target_vec);
	  float angle = acos(orthogonal_vec.dot(from_initial_target_vec) / from_initial_target_vec.length());
          ROS_WARN("angle:%f dfit:%f len:%f ", angle, distance_from_initial_target, from_initial_target_vec.length());
          if(angle < min_angle && distance_from_initial_target > 0 && from_initial_target_vec.length() < narrow_searching_radius_ && angle > previous_target_angle + narrow_angle_diff_min_)
	    {
	      min_angle = angle;
	      target_tree = *it;
	    }
	}
    }
  else if (searching_method_ == 2)
    {
      float min_dist = 1e6;
      tf::Vector3 previous_target_tree_pos = target_trees_.back()->getPos();
      for(vector<TreeHandlePtr>::iterator it = trees.begin(); it != trees.begin() + tree_db_.validTreeNum(); ++it)
	{
	  if (find(target_trees_.begin(), target_trees_.end(), *it)  != target_trees_.end())
	    {
	      //cout << "ignore: previous targe tree" << endl;
	      continue;
	    }
	  tf::Vector3 from_previous_target_vec = (*it)->getPos() - previous_target_tree_pos;
	  float distance_from_prev = initial_target_tree_direction_vec_.dot(from_previous_target_vec);
	  float orthogonal_dist = orthogonal_vec.dot((*it)->getPos());
	  float dist = (previous_target_tree_pos - (*it)->getPos()).length();
	  
	  if(dist < min_dist && distance_from_prev > 0 && fabs(orthogonal_dist) < max_orthogonal_dist_)
	    {
	      min_dist = dist;
	      target_tree = *it;
	    }
	}
    }
  /* can not find next target tree */
  if(target_tree == NULL) return false;
  
  /* find the next target tree and set */
  target_trees_.push_back(target_tree);
  ROS_INFO("find new target tree: [%.3f, %.3f]", target_tree->getPos().x(), target_tree->getPos().y());
  return true;
}

bool TreeTracking::updateTargetTreeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if(searchTargetTreeFromDatabase()) res.success = true;
  else res.success = false;

  return true;
}

void TreeTracking::trackingControlCallback(const std_msgs::BoolConstPtr& msg)
{
  if(msg->data)
    {
      sub_laser_scan_ = nh_.subscribe(laser_scan_topic_name_, 1, &TreeTracking::laserScanCallback, this); //restart
      ROS_INFO("restart tree tracking");
    }
  else
    {
      sub_laser_scan_.shutdown(); //stop
      tree_db_.save(); //save the tree data to the file

      /* publish the stop signal */
      std_msgs::Bool stop_database;
      stop_database.data = false;
      pub_database_control_.publish(stop_database);

      ROS_INFO("stop tree tracking");
    }
}

bool TreeTracking::setFirstTreeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  target_trees_.erase(target_trees_.begin() + 1, target_trees_.end());
  res.success = true;
  return true;
}
