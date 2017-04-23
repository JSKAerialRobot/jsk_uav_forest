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
#include <jsk_uav_forest_perception/vision_detector_base_plugin.h>

using namespace std;

namespace vision_detection
{
  typedef jsk_uav_forest_perception::HsvFilterConfig Config;
  class IndependentHsvFilter :public vision_detection::DetectorBase
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle pnh);
  protected:
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    int hsv_lower_bound_[3];
    int hsv_upper_bound_[3];
    int contour_color_r_, contour_color_g_, contour_color_b_;
    cv::Scalar contour_color_;
    double color_region_tree_clustering_angle_diff_thre_;

    void configCallback(Config &new_config, uint32_t level);

    void hsvFilter(cv::Mat src_image, string encoding, cv::Mat& hue_image_mask);

    virtual bool filter(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::LaserScanConstPtr& scan_msg, int& target_tree_index);
  };

};

