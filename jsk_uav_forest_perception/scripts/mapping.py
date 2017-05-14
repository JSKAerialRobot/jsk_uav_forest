#!/usr/bin/env python

import time
import sys
import os
import rospy
import math
import signal
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import subprocess
from datetime import datetime
from std_msgs.msg import Bool
import tf
import pytz

class Mapping:
    def init(self):
        rospy.init_node('mapping', anonymous=True)
        self.mapping = False
        self.map_frame = rospy.get_param("~map_frame", "/map")
        self.odom_frame = rospy.get_param("~odom_frame", "/world")
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

            # get time
            dt_obj = datetime.fromtimestamp(rospy.get_time(), pytz.utc)
            date_str = dt_obj.strftime("%Y-%m-%d") + '-' + str(dt_obj.hour) + '-' + str(dt_obj.minute)

            # get tf
            trans = None
            rot = None
            listener = tf.TransformListener()
            while not rospy.is_shutdown():
                try:
                    (trans,rot) = listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                    rospy.logwarn("get tf from %s to %s", self.map_frame, self.odom_frame)
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #rospy.logerr("can not get tf from %s to %s", self.map_frame, self.odom_frame)
                    continue

            # save map
            self.saving_process = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", date_str])

            time.sleep(1)
            self.mapping_process.send_signal(signal.SIGINT)

            #save tf
            f = open(os.environ['HOME'] + '/.ros/' + date_str + '.yaml', 'a')
            f.write('trans: [' + str(trans[0]) + ', '  + str(trans[1]) + ', ' + str(trans[2]) +']\n')
            f.write('rot: [' + str(rot[0]) + ', '  + str(rot[1]) + ', ' + str(rot[2]) + ', ' + str(rot[3]) + ']\n')
            f.close()
            self.mapping = False;

if __name__ == '__main__':
    try:
        mapping = Mapping()
        mapping.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
