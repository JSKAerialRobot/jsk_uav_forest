#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan

class CircleMotion:
    
    def init(self):
        rospy.init_node('circle_motion', anonymous=True)
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / 20), self.controlCallback)
        
        self.vel_pub_topic_name_ = rospy.get_param("~vel_pub_topic_name", "cmd_vel")
        self.odom_sub_topic_name_ = rospy.get_param("~odom_sub_topic_name", "ground_truth/state")
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "state_machine")
        self.scan_filtered_wood_topic_name_ = rospy.get_param("~scan_filtered_wood_topic_name", "scan_filtered_foot")
        self.nav_pos_pgain_ = rospy.get_param("~nav_pos_pgain", 1.0)
        self.nav_yaw_pgain_ = rospy.get_param("~nav_yaw_pgain", 1.0)
        self.nav_vel_thresh_ = rospy.get_param("~nav_vel_thresh", 2.0)
        self.nav_yaw_vel_thresh_ = rospy.get_param("~nav_yaw_vel_thresh", 1.0)
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.1)
        self.nav_vel_convergence_thresh_ = rospy.get_param("~nav_vel_convergence_thresh", 0.1)
    
        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        self.odom_sub_ = rospy.Subscriber(self.odom_sub_topic_name_, Odometry, self.odomCallback)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, UInt8, queue_size = 10)
        self.scan_filtered_wood_topic_name_ = rospy.Subscriber(self.scan_filtered_wood_topic_name_, LaserScan, self.scanFilteredWoodCallback)
        self.debug_pub_ = rospy.Publisher("scan_debug", LaserScan, queue_size = 10)

        self.control_velocity_msg_ = Twist()
        self.control_velocity_msg_.linear.x = 0
        self.control_velocity_msg_.linear.y = 0
        self.control_velocity_msg_.angular.z = 0
         
        self.cnt_ = 0
        self.odom_ = Odometry()
        self.target_pos_ = np.array([0, 0, 0])
        self.target_yaw_ = 0

        self.INITIAL_STATE_ = 0
        self.HOVERING_STATE_ = 1
        self.APPROACHING_TO_WOOD_STATE_ = 2
        self.CIRCLE_MOTION_STATE_ = 3
        self.state_machine_ = self.INITIAL_STATE_

        self.odom_update_flag_ = False
    
    def odomCallback(self, msg):
        self.odom_ = msg
        self.odom_update_flag_ = True
   
    def scanFilteredWoodCallback(self, msg):
        pub_msg = msg
        l = list(msg.ranges)
        index = 0
        for i in msg.ranges:
            if i < 0:
                l[index] = 0.0
            index += 1
        pub_msg.ranges = l
        self.debug_pub_.publish(pub_msg)

    def isConvergent(self, target_pos):
        current_pos = np.array([self.odom_.pose.pose.position.x, 
                                self.odom_.pose.pose.position.y, 
                                self.odom_.pose.pose.position.z])
        delta_pos = target_pos - current_pos
        current_vel = np.array([self.odom_.twist.twist.linear.x, 
                                self.odom_.twist.twist.linear.y, 
                                self.odom_.twist.twist.linear.z])
        if np.linalg.norm(delta_pos) < self.nav_pos_convergence_thresh_ and np.linalg.norm(current_vel) < self.nav_vel_convergence_thresh_:
            return True
        else:
            return False


    def goPos(self, target_pos, target_yaw):
        current_pos = np.array([self.odom_.pose.pose.position.x, 
                                self.odom_.pose.pose.position.y, 
                                self.odom_.pose.pose.position.z])
        quaternion = np.array([self.odom_.pose.pose.orientation.x,
                   self.odom_.pose.pose.orientation.y,
                   self.odom_.pose.pose.orientation.z,
                   self.odom_.pose.pose.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)

        delta_pos = target_pos - current_pos
        delta_yaw = target_yaw - euler[2]
        
        nav_vel = delta_pos * self.nav_pos_pgain_
        nav_yaw_vel = delta_yaw * self.nav_yaw_pgain_
        if np.linalg.norm(nav_vel) > self.nav_vel_thresh_:
            nav_vel *= (self.nav_vel_thresh_ / np.linalg.norm(nav_vel))
        if nav_yaw_vel > self.nav_yaw_vel_thresh_:
            nav_yaw_vel = self.nav_yaw_vel_thresh_
        
        self.control_velocity_msg_.linear.x = nav_vel[0]
        self.control_velocity_msg_.linear.y = nav_vel[1]
        self.control_velocity_msg_.linear.z = nav_vel[2]
        self.control_velocity_msg_.angular.z = nav_yaw_vel
        self.vel_pub_.publish(self.control_velocity_msg_)
   
    def goCircle(self, local_circle_center_pos, radius):
        self.control_velocity_msg_.linear.x = (local_circle_center_pos[0] - radius) * 1.0
        self.control_velocity_msg_.linear.y = 0.3
        self.control_velocity_msg_.linear.z = (self.target_pos_[2] - local_circle_center_pos[2]) * 1.0
        self.control_velocity_msg_.angular.z = (local_circle_center_pos[1]) * 2.0
        self.vel_pub_.publish(self.control_velocity_msg_)

    def controlCallback(self, event):
        if not self.odom_update_flag_:
            return
        if self.state_machine_ == self.INITIAL_STATE_:
            self.target_pos_ = np.array([self.odom_.pose.pose.position.x, 
                                          self.odom_.pose.pose.position.y, 
                                          self.odom_.pose.pose.position.z])
            quaternion = np.array([self.odom_.pose.pose.orientation.x,
                                   self.odom_.pose.pose.orientation.y,
                                   self.odom_.pose.pose.orientation.z,
                                   self.odom_.pose.pose.orientation.w])
            self.target_yaw_ = tf.transformations.euler_from_quaternion(quaternion)[2]
            self.target_pos_[2] = 1.0 #hover
            self.state_machine_ = self.HOVERING_STATE_

        elif self.state_machine_ == self.HOVERING_STATE_:
            self.goPos(self.target_pos_, self.target_yaw_)
            if self.isConvergent(self.target_pos_):
                self.target_pos_[0] = -1.0
                self.state_machine_ = self.APPROACHING_TO_WOOD_STATE_
        
        elif self.state_machine_ == self.APPROACHING_TO_WOOD_STATE_:
            self.goPos(self.target_pos_, self.target_yaw_)
            if self.isConvergent(self.target_pos_):
                self.state_machine_ = self.CIRCLE_MOTION_STATE_

        elif self.state_machine_ == self.CIRCLE_MOTION_STATE_:
            quaternion = np.array([self.odom_.pose.pose.orientation.x,
                                   self.odom_.pose.pose.orientation.y,
                                   self.odom_.pose.pose.orientation.z,
                                   self.odom_.pose.pose.orientation.w])
            yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
            
            local_x = math.cos(yaw) * -self.odom_.pose.pose.position.x + math.sin(yaw) * -self.odom_.pose.pose.position.y
            local_y = -math.sin(yaw) * -self.odom_.pose.pose.position.x + math.cos(yaw) * -self.odom_.pose.pose.position.y
            local_circle_center_pos = np.array([local_x, local_y, self.target_pos_[2]])
            self.goCircle(local_circle_center_pos, 1.0)

        self.state_machine_pub_.publish(self.state_machine_)

if __name__ == '__main__':
    try:
        circle_motion = CircleMotion()
        circle_motion.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
