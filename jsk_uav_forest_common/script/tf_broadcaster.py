#!/usr/bin/env python

import roslib
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry

class odom_tf:
    def init(self):
        self.uav_odom_topic_name = rospy.get_param("~uav_odom_topic_name", "/modified_odom")
        self.use_mapping = rospy.get_param("~use_mapping", False)
        self.map_frame_name = rospy.get_param("~map_frame_name", "/map")
        self.world_frame_name = rospy.get_param("~world_frame_name", "/world")
        self.base_frame_name = rospy.get_param("~base_name", "/base_link")
        self.base_footprint_frame_name = rospy.get_param("~base_footprint_name", "/base_footprint")
        self.laser_frame_name = rospy.get_param("~laser_frame_name", "/laser")

        rospy.Subscriber(self.uav_odom_topic_name, Odometry, self.odometryCallback)

        self.map_init = False

    def odometryCallback(self, msg):
        br = tf.TransformBroadcaster()

        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 0])
        ori = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        euler = tf.transformations.euler_from_quaternion(ori)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, euler[2])

        # /world -> /base
        br.sendTransform(pos, quaternion, msg.header.stamp, self.base_footprint_frame_name, self.world_frame_name)
        pos = np.array([0, 0, msg.pose.pose.position.z])
        quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], 0)
        br.sendTransform(pos, quaternion, msg.header.stamp, self.base_frame_name, self.base_footprint_frame_name)
        br.sendTransform(pos, quaternion, msg.header.stamp, self.laser_frame_name, self.base_footprint_frame_name)



if __name__ == '__main__':
    rospy.init_node("tf_broadcaster")
    odomTF = odom_tf()
    odomTF.init()
    rospy.spin()
