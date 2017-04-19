#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
import time

class OdomGenerator:
    def init(self):
        rospy.init_node("odom_gen", anonymous=True)
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/dji_sdk/odometry")
        self.uav_odom_pub_topic_name_ = rospy.get_param("~uav_odom_pub_topic_name", "modified_odom")
        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)
        self.uav_odom_pub_ = rospy.Publisher(self.uav_odom_pub_topic_name_, Odometry, queue_size = 10)
        self.modified_odom_ = Odometry()
        self.modified_odom_.pose.pose.position.x = 0.0
        self.modified_odom_.pose.pose.position.y = 0.0
        self.modified_odom_.pose.pose.position.z = 0.0
        self.modified_odom_old_stamp_ = rospy.get_rostime()
        self.first_call_flag_ = True

    def uavOdomCallback(self, msg):
        if self.first_call_flag_ == True:
            self.first_call_flag_ = False
            self.modified_odom_old_stamp_ = msg.header.stamp
            return

        time = (msg.header.stamp - self.modified_odom_old_stamp_).to_sec()
        self.modified_odom_.header = msg.header
        self.modified_odom_.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.modified_odom_.pose.pose.orientation.y = -msg.pose.pose.orientation.y
        self.modified_odom_.pose.pose.orientation.z = -msg.pose.pose.orientation.z
        self.modified_odom_.pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.modified_odom_.pose.pose.position.x += (msg.twist.twist.linear.x * time)
        self.modified_odom_.pose.pose.position.y += (-msg.twist.twist.linear.y * time)
        self.modified_odom_.pose.pose.position.z = msg.pose.pose.position.z
        self.uav_odom_pub_.publish(self.modified_odom_)
        self.modified_odom_old_stamp_ = msg.header.stamp

if __name__ == '__main__':
    try:
        odom_gen = OdomGenerator()
        odom_gen.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

