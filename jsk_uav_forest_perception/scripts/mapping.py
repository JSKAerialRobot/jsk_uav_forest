#!/usr/bin/env python

import time
import sys
import rospy
import math
import signal
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import subprocess
from datetime import datetime

class Mapping:
    def init(self):
        rospy.init_node('mapping', anonymous=True)
        self.mapping = False
        self.first_tree_dist_thresh = rospy.get_param("~first_tree_dist_thresh", 3) # [m]
        self.tree_location_sub_topic_name = rospy.get_param("~tree_location_sub_topic_name", "tree_location")
        self.mapping_control_sub_topic_name = rospy.get_param("~mapping_control_sub_topic_name", "/tracking_control")

        self.mapping_process = None
        self.saving_process = None
        self.tree_location_sub = rospy.Subscriber(self.tree_location_sub_topic_name, PointStamped, self.treeLocationCallback)
        self.tree_location_sub = rospy.Subscriber(self.mapping_control_sub_topic_name, Bool, self.mappingControlCallback)

    def treeLocationCallback(self, msg):
        # search the start trigger for mapping
        if not self.mapping:
            # special condition
            dist = np.linalg.norm(np.array([msg.point.x, msg.point.y]))
            if dist < self.first_tree_dist_thresh:
                rospy.logwarn("Start mapping, first tree local location: [%f, %f]", msg.point.x, msg.point.y);
                self.mapping = True;
                self.mapping_process = subprocess.Popen(["roslaunch", "jsk_uav_forest_common", "2d_mapping.launch"])

    def mappingControlCallback(self, msg):

        if msg.data:
            rospy.loginfo("Restart mapping");
        else:
            rospy.logwarn("Stop mapping");
            # save map
            dt_obj = datetime.fromtimestamp(rospy.get_time())
            date_str = dt_obj.strftime("%Y-%m-%d-%H-%M")
            self.saving_process = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", date_str])
            time.sleep(1)
            self.mapping_process.send_signal(signal.SIGINT)
            self.mapping = False;

if __name__ == '__main__':
    try:
        mapping = Mapping()
        mapping.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
