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
#include <pluginlib/class_loader.h>
#include <jsk_uav_forest_perception/vision_detector_base_plugin.h>

namespace vision_detection
{
  class VisionDetection
  {
  public:
    VisionDetection(ros::NodeHandle nh, ros::NodeHandle pnh):
      nh_(nh), pnh_(pnh)
    {

      detector_plugin_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<vision_detection::DetectorBase> >( new pluginlib::ClassLoader<vision_detection::DetectorBase>("jsk_uav_forest_perception", "vision_detection::DetectorBase"));

      std::string plugin_name;
      if(pnh_.getParam("plugin_name", plugin_name))
        {
          detector_ = detector_plugin_ptr_->createInstance(plugin_name);
          detector_->initialize(nh, pnh);
        }
      else
        ROS_ERROR("%s: can not get the plugin name", pnh_.getNamespace().c_str());
    }

    ~VisionDetection (){};

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    boost::shared_ptr<vision_detection::DetectorBase>  detector_;
    boost::shared_ptr< pluginlib::ClassLoader<vision_detection::DetectorBase> > detector_plugin_ptr_;

  };
};


int main (int argc, char **argv)
{
  ros::init (argc, argv, "vision_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  vision_detection::VisionDetection*  vision_detection_node = new vision_detection::VisionDetection(nh, pnh);
  ros::spin ();
  delete vision_detection_node;
  return 0;
}
