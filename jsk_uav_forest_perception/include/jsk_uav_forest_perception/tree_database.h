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

#ifndef TREE_DATABASE_H_
#define TREE_DATABASE_H_

/* ros */
#include <ros/ros.h>

/* ros msg/srv */
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

/* uitls */
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cstdlib>

using namespace std;

class TreeHandle
{
public:
  TreeHandle(): pos_(0,0,0), vote_(0), radius_(0) {}
  TreeHandle(ros::NodeHandle nh, ros::NodeHandle nhp, tf::Vector3 pos);
  ~TreeHandle(){}

  boost::shared_ptr<TreeHandle> getHandle() { return boost::shared_ptr<TreeHandle>(this); }
  void updatePos(const tf::Vector3& pos, bool lpf = true);
  void setRadius(double radius, bool lpf = true);
  const double getRadius(){ return radius_; }
  const tf::Vector3 getPos() { return pos_; }
  inline void setVote(int vote) {vote_ = vote; }
  inline int getVote() { return vote_; }

private:
  ros::NodeHandle nh_, nhp_;

  double filter_rate_;

  tf::Vector3 pos_;
  double radius_;
  int vote_;
};

typedef boost::shared_ptr<TreeHandle>  TreeHandlePtr;

bool operator>(const TreeHandlePtr& left, const TreeHandlePtr& right);

class TreeDataBase
{
public:
  TreeDataBase(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~TreeDataBase(){}

  void add(const TreeHandlePtr new_tree);
  bool updateSingleTree(const tf::Vector3& tree_pos, const double& tree_radius, const bool only_target = false);

  void update();
  void visualization(std_msgs::Header header);
  void save();
  bool load(string file_name);

  int validTreeNum()
  {
    return (trees_.size()>valid_num_)?valid_num_:trees_.size();
  }

  inline void getTrees(vector<TreeHandlePtr>& trees) { trees = trees_; }
private:
  ros::NodeHandle nh_, nhp_;

  /* ros param */
  double min_distance_; /* the min distance between two tree  */
  double max_radius_;  /* the max radius to use for tree update  */
  int valid_num_; /* the number of valid tree to  */
  bool verbose_;
  bool visualization_;
  string visualization_marker_topic_name_;

  ros::Publisher pub_visualization_marker_;

  /* the set for trees */
  vector<TreeHandlePtr> trees_;
};
#endif
