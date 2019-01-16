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

/* base class */
#include <jsk_uav_forest_perception/vision_detector/no_image_filter.h>

using namespace std;

namespace vision_detection
{
  void NoImageFilter::initialize(ros::NodeHandle nh, ros::NodeHandle pnh)
  {
    vision_detection::DetectorBase::initialize(nh, pnh);

    pnh_.param("max_distance", max_distance_, 6.0);
  }

  bool NoImageFilter::filter(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg, int& target_tree_index)
  {
    vector<int> cluster_index;
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
      {
        if(scan_msg->ranges[i] > 0)
            cluster_index.push_back(i);
      }

    if(cluster_index.empty())
      return false;

    float min_direction = 1e6;
    float target_tree_laser_distance = 0.0;
    for (auto it = cluster_index.begin(); it != cluster_index.end(); ++it)
      {
        float laser_direction = *it * scan_msg->angle_increment + scan_msg->angle_min;
        float laser_distance = scan_msg->ranges[*it];

        if(laser_distance < max_distance_)
          {
            if(fabs(laser_direction) < min_direction)
              {
                target_tree_index = *it;
                min_direction = fabs(laser_direction);
                target_tree_laser_distance = laser_distance;
              }
          }
      }

    ROS_ERROR("find the tree %f, %f", min_direction, target_tree_laser_distance);
    return true;
  }

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_detection::NoImageFilter, vision_detection::DetectorBase);
