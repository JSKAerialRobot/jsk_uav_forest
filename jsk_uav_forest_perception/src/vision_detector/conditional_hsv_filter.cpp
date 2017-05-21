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
#include <jsk_uav_forest_perception/vision_detector/conditional_hsv_filter.h>

using namespace std;

namespace vision_detection
{
  void ConditionalHsvFilter::initialize(ros::NodeHandle nh, ros::NodeHandle pnh)
  {
    vision_detection::IndependentHsvFilter::initialize(nh, pnh);

    pnh_.param("tree_diameter", tree_diameter_, 0.3); // should be smaller than real size

  }


  bool ConditionalHsvFilter::filter(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg, int& target_tree_index)
  {
    cv::Mat src_image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

    /* general hsv filter */
    cv::Mat hsv_image_mask;
    hsvFilter(src_image, image_msg->encoding, hsv_image_mask);

    /* labeling, find the max contour inside each tree area, and choose the biggest one among the max contours. */
    vector<int> cluster_index;
    laserClustering(*scan_msg, cluster_index, src_image);

    float fov = atan2(camera_cx_, camera_fx_); //proximate the width of image with camera_cx_
    float max_contour_area = 0;
    std::vector<std::vector<cv::Point> > max_contours;
    int max_contour_id = 0;
    cv::Mat target_tree_area;
    int tree_index = 0;
#if 0
    for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
      {
        float laser_direction = *it * scan_msg->angle_increment + scan_msg->angle_min;
        float laser_distance = scan_msg->ranges[*it];

        if(!commonFiltering(laser_direction, fov, laser_distance)) continue;

        float x1 = laser_distance * cos(laser_direction) + tree_diameter_ / 2 * cos(laser_direction + M_PI/2);
        float x2 = laser_distance * cos(laser_direction) + tree_diameter_ / 2 * cos(laser_direction - M_PI/2);

        float y1 = laser_distance * sin(laser_direction) + tree_diameter_ / 2  * sin(laser_direction - M_PI/2);
        float y2 = laser_distance * sin(laser_direction) + tree_diameter_ / 2  * sin(laser_direction + M_PI/2);
        float x_l = camera_cx_ + camera_offset_x_ - camera_fx_ * y2 / x2;
        float x_r = camera_cx_ + camera_offset_x_ - camera_fx_ * y1 / x1;
        if(x_l < 0) x_l = 0;
        if(x_r > src_image.cols) x_r = src_image.cols;
        if(verbose_)
          ROS_INFO("roi calculation: x1:%f, x2:%f, y1:%f, y2:%f, x_l:%f, x_r:%f", x1, x2, y1, y2,x_l,x_r);
        cv::Rect roi(x_l, 0, x_r - x_l, src_image.rows);
        cv::Mat tree_area = hsv_image_mask(roi);
        /* draw roi */
        cv::rectangle(src_image, roi, cv::Scalar(0, 0, 255), 3,8,0);

        float contour_area = 0;
        std::vector<std::vector<cv::Point> > contours;
        int contour_id = 0;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(tree_area, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
        if(contours.size() == 0)
          {
            ROS_INFO("eliminate: no contour, tree index: %d, tree direction: %f", *it, laser_direction);
            continue;
          }

        /* find the max contour in roi */
        for(int i = 0; i < contours.size(); ++i)
          {
            if (cv::contourArea(contours[i]) > contour_area)
              {
                contour_area = cv::contourArea(contours[i]);
                contour_id = i;
              }
          }

        /* update the biggest countour in all trees */
        if(contour_area > max_contour_area )
          {
            max_contour_area = contour_area;
            max_contour_id = contour_id;
            max_contours = contours;
            target_tree_index = *it;
            tree_index = distance(cluster_index.begin(), it);
            target_tree_area = src_image(roi);
          }

        if(verbose_)
          {
            cout << "tree index: " << *it << endl;
            cout << "tree angle: " << laser_direction << endl;
            cout << "contour_area: " << contour_area << endl;
          }
      }

    if(max_contour_area == 0) return false;

    /* draw */
    cv::drawContours(target_tree_area, max_contours, max_contour_id, contour_color_, 5);

    /* publish */
    pub_target_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                 image_msg->encoding,
                                                 src_image).toImageMsg());


    /* find the target tree */
    ROS_WARN("conditional hsv filter based vision detectoion: find the target tree.");
    return true;
#else
    float min_direction = 1e6;
    for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
      {
        float laser_direction = *it * scan_msg->angle_increment + scan_msg->angle_min;
        float laser_distance = scan_msg->ranges[*it];

        if(laser_distance < 6) // 5[m]
          {
            if(fabs(laser_direction) < min_direction )
              {
                target_tree_index = *it;
                min_direction = fabs(laser_direction);
                ROS_INFO("update the tree, %f, %f",laser_direction, laser_distance);
              }
          }
      }
    ROS_ERROR("find the tree");
    return true;
#endif
  }
};


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_detection::ConditionalHsvFilter, vision_detection::DetectorBase);

