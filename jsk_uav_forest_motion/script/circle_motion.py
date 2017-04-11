#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
from dji_sdk.dji_drone import DJIDrone
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class CircleMotion:
    
    def init(self):
        if sys.argv[1] == 'True':
            self.use_dji_ = True
        else:
            self.use_dji_ = False

        if self.use_dji_ == True:
            self.drone_ = DJIDrone()
            self.drone_.request_sdk_permission_control()
            rospy.loginfo("Use DJI")
        else:
            rospy.init_node('circle_motion', anonymous=True)
        
        self.vel_pub_topic_name_ = rospy.get_param("~vel_pub_topic_name", "cmd_vel")
        self.odom_sub_topic_name_ = rospy.get_param("~odom_sub_topic_name", "ground_truth/state")
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "state_machine")
        self.scan_filtered_wood_topic_name_ = rospy.get_param("~scan_filtered_wood_topic_name", "scan_filtered_foot")
        self.control_rate_ = rospy.get_param("~control_rate", 20)
        
        self.nav_pos_pgain_ = rospy.get_param("~nav_pos_pgain", 1.0)
        self.takeoff_height_ = rospy.get_param("~takeoff_height", 1.0)
        self.nav_yaw_pgain_ = rospy.get_param("~nav_yaw_pgain", 1.0)
        self.nav_vel_thresh_ = rospy.get_param("~nav_vel_thresh", 2.0)
        self.nav_yaw_vel_thresh_ = rospy.get_param("~nav_yaw_vel_thresh", 1.0)
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.1)
        self.nav_vel_convergence_thresh_ = rospy.get_param("~nav_vel_convergence_thresh", 0.1)
        self.circle_radius_ = rospy.get_param("~circle_radius", 1.0);
        self.circle_x_gain_ = rospy.get_param("~circle_x_gain", 1.0);
        self.circle_y_vel_ = rospy.get_param("~circle_y_vel", 0.5);
        self.circle_yaw_gain_ = rospy.get_param("~circle_yaw_gain", 2.0)
        
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        self.odom_sub_ = rospy.Subscriber(self.odom_sub_topic_name_, Odometry, self.odomCallback)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)

        self.control_velocity_msg_ = Twist()
        self.control_velocity_msg_.linear.x = 0
        self.control_velocity_msg_.linear.y = 0
        self.control_velocity_msg_.linear.z = 0
        self.control_velocity_msg_.angular.z = 0
         
        self.odom_ = Odometry()
        self.target_pos_ = np.zeros(3)
        self.target_yaw_ = 0
        self.initial_pos_ = np.zeros(3)

        #state machine
        self.INITIAL_STATE_ = 0
        self.TAKEOFF_STATE_ = 1
        self.APPROACHING_TO_WOOD_STATE_ = 2
        self.CIRCLE_MOTION_STATE_ = 3
        self.RETURN_HOME_STATE_ = 4
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "takeoff", "approaching to wood", "circle motion", "return home"]

        self.odom_update_flag_ = False
        self.uav_pos_ = np.zeros(3)
        self.uav_yaw_ = 0.0
        self.uav_yaw_old_ = 0.0
        self.uav_yaw_overflow_ = 0
        self.accumulated_yaw_ = 0.0
        self.circle_initial_yaw_ = 0.0

    def odomCallback(self, msg):
        self.odom_ = msg
        self.uav_pos_ = np.array([self.odom_.pose.pose.position.x, self.odom_.pose.pose.position.y, self.odom_.pose.pose.position.z])
        quaternion = np.array([self.odom_.pose.pose.orientation.x, self.odom_.pose.pose.orientation.y, self.odom_.pose.pose.orientation.z, self.odom_.pose.pose.orientation.w])
        self.uav_yaw_ = tf.transformations.euler_from_quaternion(quaternion)[2]
        if self.odom_update_flag_ == True:
            if self.uav_yaw_ - self.uav_yaw_old_ > 5.0:
                self.uav_yaw_overflow_ -= 1 
            elif self.uav_yaw_old_ - self.uav_yaw_old_ < -5.0:
                self.uav_yaw_overflow_ += 1
        self.accumulated_yaw_ = self.uav_yaw_ + 2 * math.pi * self.uav_yaw_overflow_
        self.uav_yaw_old_ = self.uav_yaw_
        self.odom_update_flag_ = True
    
    def isConvergent(self, target_pos):
        delta_pos = target_pos - self.uav_pos_ 
        current_vel = np.array([self.odom_.twist.twist.linear.x, 
                                self.odom_.twist.twist.linear.y, 
                                self.odom_.twist.twist.linear.z])
        if np.linalg.norm(delta_pos) < self.nav_pos_convergence_thresh_ and np.linalg.norm(current_vel) < self.nav_vel_convergence_thresh_:
            return True
        else:
            return False

    def goPos(self, target_pos, target_yaw):
        delta_pos = target_pos - self.uav_pos_
        delta_yaw = target_yaw - self.uav_yaw_
        
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
        if self.use_dji_ == True:
            self.drone_.velocity_control(0, nav_vel[0], nav_vel[1], nav_vel[2], nav_yaw_vel) #machine frame

    def goCircle(self, local_circle_center_pos, radius):
        
        self.control_velocity_msg_.linear.x = (math.sqrt(local_circle_center_pos[0] ** 2 + local_circle_center_pos[1] ** 2) - self.circle_radius_) * self.circle_x_gain_
        self.control_velocity_msg_.linear.y = self.circle_y_vel_
        self.control_velocity_msg_.linear.z = (self.target_pos_[2] - local_circle_center_pos[2]) * self.nav_pos_pgain_
        self.control_velocity_msg_.angular.z = (local_circle_center_pos[1]) * self.circle_yaw_gain_ - self.control_velocity_msg_.linear.y / radius
        self.vel_pub_.publish(self.control_velocity_msg_)
        if self.use_dji_ == True:
            self.drone_.velocity_control(0, self.control_velocity_msg_.linear.x ,self.control_velocity_msg_.linear.y, self.control_velocity_msg_.linear.z, self.control_velocity_msg_.angular.z) #machine frame

    def controlCallback(self, event):
        if not self.odom_update_flag_:
            return

        #state machine
        if self.state_machine_ == self.INITIAL_STATE_:
            self.target_pos_ = self.uav_pos_
            self.initial_pos_ = np.array(self.target_pos_)
            self.target_yaw_ = self.uav_yaw_
            self.target_pos_[2] = self.takeoff_height_ #takeoff
            self.goPos(self.target_pos_, self.target_yaw_)
            if self.use_dji_ == True:
                self.drone_.takeoff()
            self.state_machine_ = self.TAKEOFF_STATE_
            rospy.loginfo("take off")

        elif self.state_machine_ == self.TAKEOFF_STATE_:
            self.goPos(self.target_pos_, self.target_yaw_)
            if self.isConvergent(self.target_pos_):
                self.target_pos_[0] = -1.0
                self.state_machine_ = self.APPROACHING_TO_WOOD_STATE_
        
        elif self.state_machine_ == self.APPROACHING_TO_WOOD_STATE_:
            self.goPos(self.target_pos_, self.target_yaw_)
            if self.isConvergent(self.target_pos_):
                self.state_machine_ = self.CIRCLE_MOTION_STATE_
                self.circle_initial_yaw_ = self.accumulated_yaw_        

        elif self.state_machine_ == self.CIRCLE_MOTION_STATE_:
            local_x = math.cos(self.uav_yaw_) * -self.odom_.pose.pose.position.x + math.sin(self.uav_yaw_) * -self.odom_.pose.pose.position.y
            local_y = -math.sin(self.uav_yaw_) * -self.odom_.pose.pose.position.x + math.cos(self.uav_yaw_) * -self.odom_.pose.pose.position.y
            local_circle_center_pos = np.array([local_x, local_y, self.target_pos_[2]])
            self.goCircle(local_circle_center_pos, 1.0)
            if abs(self.circle_initial_yaw_ - self.accumulated_yaw_) > 2 * math.pi:
                self.state_machine_ = self.RETURN_HOME_STATE_

        elif self.state_machine_ == self.RETURN_HOME_STATE_:
            self.target_pos_ = self.initial_pos_
            print(self.initial_pos_)
            self.goPos(self.target_pos_, 0)

        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])

if __name__ == '__main__':
    try:
        circle_motion = CircleMotion()
        circle_motion.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
