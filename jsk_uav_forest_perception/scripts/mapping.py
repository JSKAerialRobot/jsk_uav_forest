#!/usr/bin/env python

import sys
import os
import rospy
import math
import signal
import time
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import subprocess
from datetime import datetime
from std_msgs.msg import Bool, Empty
import tf
import pytz

class Mapping:
    def init(self):
        rospy.init_node('mapping', anonymous=True)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "world")
        self.mapping_control_sub_topic_name = rospy.get_param("~mapping_control_sub_topic_name", "/database_control")

        self.mapping_process = None
        self.saving_process = None

        self.mapping_control_sub = rospy.Subscriber(self.mapping_control_sub_topic_name, Bool, self.mappingControlCallback)

    def mappingControlCallback(self, msg):
        if msg.data:
            rospy.loginfo("Start mapping");
            self.mapping_process = subprocess.Popen(["roslaunch", "jsk_uav_forest_common", "2d_mapping.launch"])

        else:
            rospy.logwarn("Stop mapping");

            # get time
            dt_obj = datetime.fromtimestamp(time.time())
            date_str = dt_obj.strftime("%Y-%m-%d") + '-' + str(dt_obj.hour) + '-' + str(dt_obj.minute) + '-map'

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

            time.sleep(2)
            #save tf
            f = open(os.environ['HOME'] + '/.ros/' + date_str + '.yaml', 'a')
            f.write('trans: [' + str(trans[0]) + ', '  + str(trans[1]) + ', ' + str(trans[2]) +']\n')
            f.write('rot: [' + str(rot[0]) + ', '  + str(rot[1]) + ', ' + str(rot[2]) + ', ' + str(rot[3]) + ']\n')
            f.close()

            self.mapping_process.send_signal(signal.SIGINT)

if __name__ == '__main__':
    try:
        mapping = Mapping()
        mapping.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
