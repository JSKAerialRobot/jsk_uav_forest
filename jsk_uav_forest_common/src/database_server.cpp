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
/* ros */
#include <ros/ros.h>

/* ros msg/srv */
#include <tf/transform_broadcaster.h>

/* tree database */
#include <jsk_uav_forest_perception/tree_database.h>

/* util */
#include <vector>

using namespace std;

class DatabaseServer
{
public:
  DatabaseServer(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh), nhp_(nhp),
    tree_db_(nh, nhp)
  {
    nhp_.param ("main_rate", main_rate_, 40.0);

    /* tf */
    vector<double> trans, rot;
    if(nhp_.getParam("trans", trans))
      transform_.setOrigin(tf::Vector3(trans[0], trans[1], trans[2]));
    else
      {
        main_rate_ = 0;
        ROS_ERROR("no tf trans parameter");
      }

    if(nhp_.getParam("rot", rot))
      transform_.setRotation(tf::Quaternion(rot[0], rot[1], rot[2], rot[3]));
    else
      {
        main_rate_ = 0;
        ROS_ERROR("no tf rot parameter");
      }

    if(nhp_.getParam("tree_database_file_name", tree_database_file_name_))
      {
        if(!tree_db_.load(tree_database_file_name_)) main_rate_ = 0;
      }
    else
      {
        main_rate_ = 0;
        ROS_ERROR("no tree dtatbase file");
      }

    nhp_.param("map_frame", map_frame_, string("/map"));
    nhp_.param("odom_frame", odom_frame_, string("/world"));

    if(main_rate_ > 0)
      {
        main_timer_ = nhp_.createTimer(ros::Duration(1.0 / main_rate_), &DatabaseServer::mainFunc,this);
      }
  }

  ~DatabaseServer(){}

private:
  ros::NodeHandle nh_, nhp_;
  ros::Timer  main_timer_;
  tf::TransformBroadcaster br_;
  double main_rate_;

  string tree_database_file_name_;
  string tf_trans_;
  string tf_rot_;
  string map_frame_;
  string odom_frame_;

  TreeDataBase tree_db_;
  tf::Transform transform_;

  void mainFunc(const ros::TimerEvent & e)
  {
    /* broadcast tf */
    br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), map_frame_, odom_frame_));

    /* visulization of tree data */
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    tree_db_.visualization(header);
  }

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "tree_database_server");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  DatabaseServer*  server = new DatabaseServer(nh, nh_private);
  ros::spin ();
  delete server;
  return 0;
}


