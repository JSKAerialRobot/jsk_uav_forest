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
from std_srvs.srv import SetBool
from geometry_msgs.msg import PointStamped

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
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "ground_truth/state")
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "state_machine")
        self.tree_detection_start_service_name_ = rospy.get_param("~tree_detection_start_service_name", "sub_control")
        self.tree_location_sub_topic_name_ = rospy.get_param("~tree_location_sub_topic_name", "tree_location")
        self.control_rate_ = rospy.get_param("~control_rate", 20)
        self.takeoff_height_ = rospy.get_param("~takeoff_height", 1.5)
        self.nav_xy_pos_pgain_ = rospy.get_param("~nav_xy_pos_pgain", 1.0)
        self.nav_z_pos_pgain_ = rospy.get_param("~nav_z_pos_pgain", 1.0)
        self.nav_yaw_pgain_ = rospy.get_param("~nav_yaw_pgain", 1.0)
        self.nav_xy_vel_thresh_ = rospy.get_param("~nav_xy_vel_thresh", 2.0)
        self.nav_z_vel_thresh_ = rospy.get_param("~nav_z_vel_thresh", 2.0)
        self.nav_yaw_vel_thresh_ = rospy.get_param("~nav_yaw_vel_thresh", 1.0)
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.1)
        self.nav_vel_convergence_thresh_ = rospy.get_param("~nav_vel_convergence_thresh", 0.1)
        self.circle_radius_ = rospy.get_param("~circle_radius", 1.0)
        self.circle_x_gain_ = rospy.get_param("~circle_x_gain", 1.0) #not used
        self.circle_y_vel_ = rospy.get_param("~circle_y_vel", 0.5)
        self.circle_yaw_pgain_ = rospy.get_param("~circle_yaw_pgain", 2.0) #not used
        
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)
        self.odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.odomCallback)
        self.tree_location_sub_ = rospy.Subscriber(self.tree_location_sub_topic_name_, PointStamped, self.treeLocationCallback)
        
        self.odom_ = Odometry()
        self.target_xy_pos_ = np.zeros(2)
        self.target_z_pos_ = 0.0
        self.target_yaw_ = 0.0
        self.initial_xy_pos_ = np.zeros(2)
        self.initial_z_pos_ = 0.0

        #state machine
        self.INITIAL_STATE_ = 0
        self.TAKEOFF_STATE_ = 1
        self.APPROACHING_TO_TREE_STATE_ = 2
        self.CIRCLE_MOTION_STATE_ = 3
        self.RETURN_HOME_STATE_ = 4
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "takeoff", "approaching to tree", "circle motion", "return home"]

        self.odom_update_flag_ = False
        self.uav_xy_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_yaw_ = 0.0
        self.uav_yaw_old_ = 0.0
        self.uav_yaw_overflow_ = 0
        self.accumulated_yaw_ = 0.0
        self.circle_initial_yaw_ = 0.0
        
        self.GLOBAL_FRAME_ = 0
        self.LOCAL_FRAME_ = 1
        
        self.cycle_count_ = 0
        
        self.tree_location_ = np.zeros(3)

    def odomCallback(self, msg):
        self.odom_ = msg
        self.uav_xy_pos_ = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.uav_yaw_ = tf.transformations.euler_from_quaternion(quaternion)[2]
        if self.odom_update_flag_ == True:
            if self.uav_yaw_ - self.uav_yaw_old_ > 5.0:
                self.uav_yaw_overflow_ -= 1 
            elif self.uav_yaw_old_ - self.uav_yaw_old_ < -5.0:
                self.uav_yaw_overflow_ += 1
        self.accumulated_yaw_ = self.uav_yaw_ + 2 * math.pi * self.uav_yaw_overflow_
        self.uav_yaw_old_ = self.uav_yaw_
        self.odom_update_flag_ = True
    
    def treeLocationCallback(self, msg):
        self.tree_location_ = np.array([msg.point.x, msg.point.y])

    def isConvergent(self, frame, target_xy_pos, target_z_pos):
        if frame == self.GLOBAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0] - self.uav_xy_pos_[0], target_xy_pos[1] - self.uav_xy_pos_[1], target_z_pos - self.uav_z_pos_]) 
        elif frame == self.LOCAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0], target_xy_pos[1], target_z_pos - self.uav_z_pos_]) 
        else:
            return

        current_vel = np.array([self.odom_.twist.twist.linear.x, self.odom_.twist.twist.linear.y, self.odom_.twist.twist.linear.z])
        if np.linalg.norm(delta_pos) < self.nav_pos_convergence_thresh_ and np.linalg.norm(current_vel) < self.nav_vel_convergence_thresh_:
            return True
        else:
            return False

    def goPos(self, frame, target_xy, target_z, target_yaw):
        if frame == self.GLOBAL_FRAME_:
            rot_mat = np.array([[math.cos(self.uav_yaw_), math.sin(self.uav_yaw_)],[-math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
            local_target_xy = np.dot(rot_mat, target_xy - self.uav_xy_pos_)
            delta_xy = local_target_xy
            delta_z = target_z - self.uav_z_pos_
            delta_yaw = target_yaw - self.uav_yaw_
        elif frame == self.LOCAL_FRAME_:
            delta_xy = target_xy
            delta_z = target_z - self.uav_z_pos_
            delta_yaw = target_yaw
        else:
            return

        nav_xy_vel = delta_xy * self.nav_xy_pos_pgain_
        nav_z_vel = delta_z * self.nav_z_pos_pgain_
        nav_yaw_vel = delta_yaw * self.nav_yaw_pgain_
        if np.linalg.norm(nav_xy_vel) > self.nav_xy_vel_thresh_:
            nav_xy_vel *= (self.nav_xy_vel_thresh_ / np.linalg.norm(nav_xy_vel))
        if abs(nav_z_vel) > self.nav_z_vel_thresh_:
            nav_z_vel *= self.nav_z_vel_thresh_ / abs(nav_z_vel)
        if abs(nav_yaw_vel) > self.nav_yaw_vel_thresh_:
            nav_yaw_vel *= self.nav_yaw_vel_thresh_ / abs(nav_yaw_vel)
        
        vel_msg = Twist() 
        vel_msg.linear.x = nav_xy_vel[0]
        vel_msg.linear.y = nav_xy_vel[1]
        vel_msg.linear.z = nav_z_vel
        vel_msg.angular.z = nav_yaw_vel
        self.vel_pub_.publish(vel_msg)
        if self.use_dji_ == True:
            self.drone_.velocity_control(0, nav_xy_vel[0], nav_xy_vel[1], nav_z_vel, nav_yaw_vel) #machine frame

    def waitCycle(self, cycle, reset):
        if reset == 'True':
            self.cycle_count_ = 0
            return False
        else:
            self.cycle_count_ += 1
            if self.cycle_count_ >= cycle:
                return True
            else:
                return False

    def controlCallback(self, event):
        if not self.odom_update_flag_:
            return

        #state machine
        if self.state_machine_ == self.INITIAL_STATE_:
            self.target_xy_pos_ = self.uav_xy_pos_
            self.initial_xy_pos_ = np.array(self.target_xy_pos_)
            self.target_z_pos_ = self.takeoff_height_ #takeoff
            self.target_yaw_ = self.uav_yaw_
            if self.use_dji_ == True:
                self.drone_.takeoff()
            self.state_machine_ = self.TAKEOFF_STATE_
            rospy.loginfo("take off")

        elif self.state_machine_ == self.TAKEOFF_STATE_:
            self.goPos(self.GLOBAL_FRAME_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_)
            if self.isConvergent(self.GLOBAL_FRAME_, self.target_xy_pos_, self.target_z_pos_):
                rospy.wait_for_service(self.tree_detection_start_service_name_)
                tree_detection_start = rospy.ServiceProxy(self.tree_detection_start_service_name_, SetBool)
                tree_detection_start(True)
                self.state_machine_ = self.APPROACHING_TO_TREE_STATE_
                self.waitCycle(0, True)

        elif self.state_machine_ == self.APPROACHING_TO_TREE_STATE_:
            if self.waitCycle(self.control_rate_ * 3, False) == False:
                self.goPos(self.GLOBAL_FRAME_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_)
            else:
                self.target_xy_pos_[0] = self.tree_location_[0] - self.circle_radius_
                self.target_xy_pos_[1] = self.tree_location_[1]
            
                self.target_yaw_ = math.atan2(self.tree_location_[1], self.tree_location_[0])
                self.goPos(self.LOCAL_FRAME_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_)
                if self.isConvergent(self.LOCAL_FRAME_, self.target_xy_pos_, self.target_z_pos_):
                    self.state_machine_ = self.CIRCLE_MOTION_STATE_
                    self.circle_initial_yaw_ = self.accumulated_yaw_        

        elif self.state_machine_ == self.CIRCLE_MOTION_STATE_:
            #local_x = math.cos(self.uav_yaw_) * -self.odom_.pose.pose.position.x + math.sin(self.uav_yaw_) * -self.odom_.pose.pose.position.y
            #local_y = -math.sin(self.uav_yaw_) * -self.odom_.pose.pose.position.x + math.cos(self.uav_yaw_) * -self.odom_.pose.pose.position.y
            #local_circle_center_pos = np.array([local_x, local_y, self.target_pos_[2]])
            self.target_xy_pos_[0] = self.tree_location_[0] - self.circle_radius_
            self.target_xy_pos_[1] = 0.2
            self.target_yaw_ = math.atan2(self.tree_location_[1], self.tree_location_[0]) - math.atan2(self.target_xy_pos_[1], self.circle_radius_) #feedback+feedforward
            self.goPos(self.LOCAL_FRAME_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_)
            
            if abs(self.circle_initial_yaw_ - self.accumulated_yaw_) > 2 * math.pi:
                self.state_machine_ = self.RETURN_HOME_STATE_

        elif self.state_machine_ == self.RETURN_HOME_STATE_:
            self.target_xy_pos_ = np.array(self.initial_xy_pos_)
            self.goPos(self.GLOBAL_FRAME_, self.target_xy_pos_, self.target_z_pos_, 0)

        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])

if __name__ == '__main__':
    try:
        circle_motion = CircleMotion()
        circle_motion.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
