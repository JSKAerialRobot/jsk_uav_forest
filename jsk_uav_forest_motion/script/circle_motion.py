#!/usr/bin/env python

import time
import sys
import rospy
from geometry_msgs.msg import Twist

class CircleMotion:
    
    def init(self):
        rospy.init_node('circle_motion', anonymous=True)
        
        self.vel_pub_topic_name_ = rospy.get_param("~vel_pub_topic_name", "cmd_vel")
        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / 20), self.controlCallback)
    
        self.control_velocity_msg_ = Twist()
        self.control_velocity_msg_.linear.x = 0
        self.control_velocity_msg_.linear.y = 0
        self.control_velocity_msg_.angular.z = 0
    
        self.cnt_ = 0

    def takeOff(self):
        rospy.loginfo("take off")
        self.control_velocity_msg_.linear.z = 1.0
        self.vel_pub_.publish(self.control_velocity_msg_)

    def controlCallback(self, event):
        self.cnt_+=1
        if self.cnt_ == 100:
            self.takeOff()


if __name__ == '__main__':
    try:
        circle_motion = CircleMotion()
        circle_motion.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
