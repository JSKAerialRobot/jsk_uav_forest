#!/usr/bin/env python

import roslib
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry

def odometryCallback(msg):
    br = tf.TransformBroadcaster()
    pos = np.array([msg.pose.pose.position.x, -msg.pose.pose.position.y, msg.pose.pose.position.z])
    ori = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, -msg.pose.pose.orientation.w])
    br.sendTransform(pos, ori, msg.header.stamp, "/uav", "/world")
    br.sendTransform(pos, ori, msg.header.stamp, "/laser", "/world")

if __name__ == '__main__':
    rospy.init_node("tf_broadcaster")
    uav_odom_topic_name = rospy.get_param("~uav_odom_topic_name", "/modified_odom")
    rospy.Subscriber(uav_odom_topic_name, Odometry, odometryCallback)
    rospy.spin()
