#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
import time

class IntegrationOdom:
	def init(self):
		rospy.init_node("integration_odom", anonymous=True)
		self.uav_odom_sub_ = rospy.Subscriber("/dji_sdk/odometry", Odometry, self.uavOdomCallback)
		self.integrated_odom_pub_ = rospy.Publisher("integrated_odom", Odometry, queue_size = 10)
		self.odom_update_rate_ = 100 #hz
		self.integrated_odom_ = Odometry()
		self.integrated_odom_.pose.pose.position.x = 0.0		
		self.integrated_odom_.pose.pose.position.y = 0.0		
		self.integrated_odom_.pose.pose.position.z = 0.0		
		self.old_stamp_ = rospy.get_rostime() 
		self.first_call_flag_ = True

	def uavOdomCallback(self, msg):
		if self.first_call_flag_ == True:
			self.first_call_flag_ = False
			self.old_stamp_ = msg.header.stamp
			return
		
		time = (msg.header.stamp - self.old_stamp_).to_sec()
		quaternion = np.array([msg.pose.pose.orientation.x,
							   msg.pose.pose.orientation.y,
							   msg.pose.pose.orientation.z,
							   msg.pose.pose.orientation.w])
		#yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
		self.integrated_odom_.header = msg.header
		self.integrated_odom_.pose.pose.orientation.x = msg.pose.pose.orientation.x	
		self.integrated_odom_.pose.pose.orientation.y = msg.pose.pose.orientation.y	
		self.integrated_odom_.pose.pose.orientation.z = msg.pose.pose.orientation.z	
		self.integrated_odom_.pose.pose.orientation.w = msg.pose.pose.orientation.w	
		self.integrated_odom_.pose.pose.position.z = msg.pose.pose.position.z		
		#rot_mat = np.array([[math.cos(yaw), -math.sin(yaw)],[math.sin(yaw), math.cos(yaw)]])
		vel_vec = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
		#vel_vec = np.dot(rot_mat, vel_vec)
		self.integrated_odom_.pose.pose.position.x += (vel_vec[0] * time)
		self.integrated_odom_.pose.pose.position.y += (vel_vec[1] * time)
		
		
		#rospy.loginfo("%f, %f", vel_vec[0], vel_vec[1])

		self.integrated_odom_pub_.publish(self.integrated_odom_)
		self.old_stamp_ = msg.header.stamp		

if __name__ == '__main__':
	try:
		integration_odom = IntegrationOdom()
		integration_odom.init()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
