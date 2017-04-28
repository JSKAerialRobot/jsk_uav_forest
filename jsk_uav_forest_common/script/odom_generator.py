#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import LaserScan
import time

class OdomGenerator:
    def init(self):
        rospy.init_node("odom_gen", anonymous=True)
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/dji_sdk/odometry")
        self.lidar_sub_topic_name_ = rospy.get_param("~lidar_sub_topic_name", "/guidance/ultrasonic")
        self.guidance_vel_sub_topic_name_ = rospy.get_param("~guidance_vel_topic_name", "/guidance/velocity")

        self.use_lidar_ = rospy.get_param("~use_lidar", True)
        self.lidar_tc_ = rospy.get_param("~lidar_tc", 0.5) #0.0~1.0
        self.lidar_noise_cut_thresh_ = rospy.get_param("~lidar_noise_cut_thresh", 0.5)
        self.use_guidance_vel_ = rospy.get_param("~use_guidance_vel", True)
        self.guidance_vel_weight_ = rospy.get_param("~guidance_vel_weight", 0.9)

        self.modified_odom_ = Odometry()
        self.modified_odom_.pose.pose.position.x = 0.0
        self.modified_odom_.pose.pose.position.y = 0.0
        self.modified_odom_.pose.pose.position.z = 0.0
        self.uav_roll_ = 0.0
        self.uav_pitch_ = 0.0
        self.modified_odom_old_stamp_ = rospy.get_rostime()
        self.odom_update_flag_ = True

        self.lidar_update_flag_ = False
        self.uav_lidar_z_ = 0.0
        self.uav_lidar_z_old_ = 0.0
        self.uav_lidar_old_stamp_ = rospy.get_rostime()

        self.guidance_vel_z_ = 0.0

        self.uav_odom_pub_topic_name_ = rospy.get_param("~uav_odom_pub_topic_name", "modified_odom")
        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)
        self.uav_odom_pub_ = rospy.Publisher(self.uav_odom_pub_topic_name_, Odometry, queue_size = 10)
        self.lidar_sub_ = rospy.Subscriber(self.lidar_sub_topic_name_, LaserScan, self.lidarCallback)
        self.guidance_vel_sub_ = rospy.Subscriber(self.guidance_vel_sub_topic_name_, Vector3Stamped, self.guidanceVelCallback)

    def guidanceVelCallback(self, msg):
        self.guidance_vel_z_ = -msg.vector.z #minus is necessary due to guidance sdk

    def lidarCallback(self, msg):
        if (self.odom_update_flag_ == True) and (self.lidar_update_flag_ == True):
            
            lidar_val = msg.ranges[0] * math.cos(self.uav_roll_) * math.cos(self.uav_pitch_)
            time = (msg.header.stamp - self.uav_lidar_old_stamp_).to_sec()        

            if self.use_guidance_vel_ == True:
                self.uav_lidar_z_ = self.uav_lidar_z_old_ + self.guidance_vel_z_ * math.cos(self.uav_roll_) * math.cos(self.uav_pitch_) * time #update by guidance velocity integration
                if lidar_val > self.lidar_noise_cut_thresh_: #if reliable
                    self.uav_lidar_z_ = lidar_val * (1.0 - self.guidance_vel_weight_) + self.uav_lidar_z_ * self.guidance_vel_weight_ #fusion
                self.uav_lidar_z_ = self.lidar_tc_ * self.uav_lidar_z_old_ + (1.0 - self.lidar_tc_) * self.uav_lidar_z_ #LPF
            else:
                if lidar_val > self.lidar_noise_cut_thresh_: #if reliable
                    self.uav_lidar_z_ = self.lidar_tc_ * self.uav_lidar_z_old_ + (1.0 - self.lidar_tc_) * lidar_val #LPF
                else:
                    pass #no update
            self.uav_lidar_z_old_ = self.uav_lidar_z_ #update
        
        self.uav_lidar_old_stamp_ = msg.header.stamp
        self.lidar_update_flag_ = True

    def uavOdomCallback(self, msg):
        if (self.odom_update_flag_ == True) and (self.lidar_update_flag_ == True):
            self.modified_odom_.header = msg.header
            self.modified_odom_.pose.pose.orientation.x = msg.pose.pose.orientation.x
            self.modified_odom_.pose.pose.orientation.y = -msg.pose.pose.orientation.y
            self.modified_odom_.pose.pose.orientation.z = -msg.pose.pose.orientation.z
            self.modified_odom_.pose.pose.orientation.w = msg.pose.pose.orientation.w
            quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            self.uav_roll_ = tf.transformations.euler_from_quaternion(quaternion)[0]
            self.uav_pitch_ = tf.transformations.euler_from_quaternion(quaternion)[1]
            
            time = (msg.header.stamp - self.modified_odom_old_stamp_).to_sec()        
            self.modified_odom_.pose.pose.position.x += (msg.twist.twist.linear.x * time)
            self.modified_odom_.pose.pose.position.y += (-msg.twist.twist.linear.y * time)
            if self.use_lidar_ == True:
                self.modified_odom_.pose.pose.position.z = self.uav_lidar_z_
            else:
                self.modified_odom_.pose.pose.position.z = msg.pose.pose.position.z
            self.uav_odom_pub_.publish(self.modified_odom_)
        
        self.modified_odom_old_stamp_ = msg.header.stamp
        self.odom_update_flag_ = True
            
if __name__ == '__main__':
    try:
        odom_gen = OdomGenerator()
        odom_gen.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

