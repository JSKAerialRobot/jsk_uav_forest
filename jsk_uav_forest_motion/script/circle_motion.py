#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
import numpy as np
from dji_sdk.dji_drone import DJIDrone
from geometry_msgs.msg import Twist, Quaternion, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolResponse

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

        self.odom_ = Odometry()
        self.GLOBAL_FRAME_ = 0
        self.LOCAL_FRAME_ = 1
        self.target_xy_pos_ = np.zeros(2)
        self.target_z_pos_ = 0.0
        self.target_yaw_ = 0.0
        self.target_frame_ = self.GLOBAL_FRAME_
        self.initial_xy_pos_ = np.zeros(2)
        self.initial_z_pos_ = 0.0
        self.initial_yaw_ = 0.0

        #state machine
        self.INITIAL_STATE_ = 0
        self.TAKEOFF_STATE_ = 1
        self.TREE_DETECTION_START_STATE_ = 2
        self.APPROACHING_TO_TREE_STATE_ = 3
        self.START_CIRCLE_MOTION_STATE_ = 4
        self.FINISH_CIRCLE_MOTION_STATE_ = 5
        self.RETURN_HOME_STATE_ = 6
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "takeoff", "tree detection start", "approaching to tree", "circle motion", "finish circle motion", "return home"]
        self.tree_detection_wait_count_ = 0

        self.odom_update_flag_ = False
        self.uav_xy_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_roll_ = 0.0
        self.uav_pitch_ = 0.0
        self.uav_yaw_ = 0.0
        self.uav_yaw_old_ = 0.0
        self.uav_yaw_overflow_ = 0
        self.uav_accumulated_yaw_ = 0.0
        
        self.lidar_update_flag_ = False
        self.uav_lidar_z_ = 0.0
        
        self.cycle_count_ = 0
        self.tree_xy_pos_ = np.zeros(2)
        
        self.task_start_ = False;
    
        self.vel_pub_topic_name_ = rospy.get_param("~vel_pub_topic_name", "cmd_vel")
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "state_machine")
        self.target_pos_pub_topic_name_ = rospy.get_param("~target_pos_pub_topic_name", "uav_target_pos")
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "ground_truth/state")
        self.tree_location_sub_topic_name_ = rospy.get_param("~tree_location_sub_topic_name", "tree_location")
        self.lidar_sub_topic_name_ = rospy.get_param("~lidar_sub_topic_name", "/guidance/ultrasonic")
        self.tree_detection_start_service_name_ = rospy.get_param("~tree_detection_start_service_name", "sub_control")
        self.task_start_service_name_ = rospy.get_param("~task_start_service_name", "task_start")
        self.control_rate_ = rospy.get_param("~control_rate", 20)
        self.takeoff_height_ = rospy.get_param("~takeoff_height", 1.5)
        self.nav_xy_pos_pgain_ = rospy.get_param("~nav_xy_pos_pgain", 1.0)
        self.nav_z_pos_pgain_ = rospy.get_param("~nav_z_pos_pgain", 1.0)
        self.nav_yaw_pgain_ = rospy.get_param("~nav_yaw_pgain", 1.0)
        self.nav_xy_vel_thresh_ = rospy.get_param("~nav_xy_vel_thresh", 2.0)
        self.nav_z_vel_thresh_ = rospy.get_param("~nav_z_vel_thresh", 2.0)
        self.nav_yaw_vel_thresh_ = rospy.get_param("~nav_yaw_vel_thresh", 1.0)
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.1)
        self.nav_yaw_convergence_thresh_ = rospy.get_param("~nav_yaw_convergence_thresh", 0.05)
        self.nav_vel_convergence_thresh_ = rospy.get_param("~nav_vel_convergence_thresh", 0.1)
        self.circle_radius_ = rospy.get_param("~circle_radius", 1.0)
        self.circle_y_vel_ = rospy.get_param("~circle_y_vel", 0.5)
        self.tree_detection_wait_ = rospy.get_param("~tree_detection_wait", 1.0)
        self.task_kind_ = rospy.get_param("~task_kind", 1) #1 yosen 2 honsen 3 kesshou
        self.use_lidar_ = False

        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)
        self.target_pos_pub_ = rospy.Publisher(self.target_pos_pub_topic_name_, Odometry, queue_size = 10)
        self.odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.odomCallback)
        self.tree_location_sub_ = rospy.Subscriber(self.tree_location_sub_topic_name_, PointStamped, self.treeLocationCallback)
        self.lidar_sub_ = rospy.Subscriber(self.lidar_sub_topic_name_, LaserScan, self.lidarCallback)
        self.task_start_service_ = rospy.Service(self.task_start_service_name_, SetBool, self.taskStartCallback)

        ###debug###
        self.debug_pub_ = rospy.Publisher("/lidar_debug", Float32, queue_size = 10)

    def odomCallback(self, msg):
        self.odom_ = msg
        self.uav_xy_pos_ = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        self.uav_roll_ = tf.transformations.euler_from_quaternion(quaternion)[0]
        self.uav_pitch_ = tf.transformations.euler_from_quaternion(quaternion)[1]
        self.uav_yaw_ = tf.transformations.euler_from_quaternion(quaternion)[2]
       
        if self.odom_update_flag_ == True:
            if self.uav_yaw_ - self.uav_yaw_old_ > 5.0:
                self.uav_yaw_overflow_ -= 1
            elif self.uav_yaw_old_ - self.uav_yaw_old_ < -5.0:
                self.uav_yaw_overflow_ += 1
        self.uav_accumulated_yaw_ = self.uav_yaw_ + 2 * math.pi * self.uav_yaw_overflow_
        self.uav_yaw_old_ = self.uav_yaw_
        self.odom_update_flag_ = True

    def treeDetectionStart(self):
        rospy.wait_for_service(self.tree_detection_start_service_name_)
        tree_detection_start = rospy.ServiceProxy(self.tree_detection_start_service_name_, SetBool)
        tree_detection_start(True)

    def treeLocationCallback(self, msg):
        self.tree_xy_pos_ = np.array([msg.point.x, msg.point.y])

    def taskStartCallback(self, req):
        res = SetBoolResponse()        
        if req.data == True:
            self.task_start_ = True
            res.success = True
            res.message = "Task Start"
            rospy.loginfo("Task Start")
        else:
            self.task_start_ = False
            res.success = False
            res.message = "Task does not start"
            rospy.loginfo("Task does not start")
        return res

    def lidarCallback(self, msg):
        if odom_update_flag_ == False:
            return
        self.uav_lidar_z_ = msg.ranges[0] * math.cos(uav_roll_) * math.cos(uav_pitch_)
        self.lidar_update_flag_ = True
        pub_msg = Float32()
        pub_msg.data = self.uav_lidar_z_
        self.debug_pub_.publish(pub_msg)

    def isConvergent(self, frame, target_xy_pos, target_z_pos, target_yaw):
        if frame == self.GLOBAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0] - self.uav_xy_pos_[0], target_xy_pos[1] - self.uav_xy_pos_[1], target_z_pos - self.uav_z_pos_]) 
        elif frame == self.LOCAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0], target_xy_pos[1], target_z_pos - self.uav_z_pos_]) 
        else:
            return

        delta_yaw = target_yaw - self.uav_yaw_
        current_vel = np.array([self.odom_.twist.twist.linear.x, self.odom_.twist.twist.linear.y, self.odom_.twist.twist.linear.z])
        if np.linalg.norm(delta_pos) < self.nav_pos_convergence_thresh_ and abs(delta_yaw) < self.nav_yaw_convergence_thresh_ and np.linalg.norm(current_vel) < self.nav_vel_convergence_thresh_:
            return True
        else:
            return False

    def saturateVelocity(self, xy_vel, z_vel, yaw_vel):
        ret = [xy_vel, z_vel, yaw_vel]
        if np.linalg.norm(xy_vel) > self.nav_xy_vel_thresh_:
            ret[0] = xy_vel * (self.nav_xy_vel_thresh_ / np.linalg.norm(xy_vel))
        if abs(z_vel) > self.nav_z_vel_thresh_:
            ret[1] = z_vel * self.nav_z_vel_thresh_ / abs(z_vel)
        if abs(yaw_vel) > self.nav_yaw_vel_thresh_:
            ret[2] = yaw_vel * self.nav_yaw_vel_thresh_ / abs(yaw_vel)
        return ret

    #go pos function: z and yaw are always in global frame
    def goPos(self, frame, target_xy, target_z, target_yaw):
        if frame == self.GLOBAL_FRAME_:
            rot_mat = np.array([[math.cos(self.uav_yaw_), math.sin(self.uav_yaw_)],[-math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
            delta_xy = np.dot(rot_mat, target_xy - self.uav_xy_pos_)
        elif frame == self.LOCAL_FRAME_:
            delta_xy = target_xy
        else:
            return
        
        delta_z = target_z - self.uav_z_pos_
        delta_yaw = target_yaw - self.uav_yaw_
       
        nav_xy_vel = delta_xy * self.nav_xy_pos_pgain_
        nav_z_vel = delta_z * self.nav_z_pos_pgain_
        nav_yaw_vel = delta_yaw * self.nav_yaw_pgain_
      
        nav_xy_vel, nav_z_vel, nav_yaw_vel = self.saturateVelocity(nav_xy_vel, nav_z_vel, nav_yaw_vel)
        vel_msg = Twist() 
        vel_msg.linear.x = nav_xy_vel[0]
        vel_msg.linear.y = nav_xy_vel[1]
        vel_msg.linear.z = nav_z_vel
        vel_msg.angular.z = nav_yaw_vel
        #for debug
        self.target_xy_pos_ = target_xy
        self.target_z_pos_ = target_z
        self.target_yaw_ = target_yaw
        self.target_frame_ = frame

        return vel_msg

    def goCircle(self, circle_center_xy, target_z, target_y_vel, radius):
        nav_xy_vel = np.zeros(2)
        nav_xy_vel[0] = -(radius - circle_center_xy[0]) * self.nav_xy_pos_pgain_
        nav_xy_vel[1] = target_y_vel
        nav_z_vel = target_z - self.uav_z_pos_
        nav_yaw_vel = math.atan2(circle_center_xy[1], circle_center_xy[0]) * self.nav_yaw_pgain_ - target_y_vel / radius #feedback+feedforward
        nav_xy_vel, nav_z_vel, nav_yaw_vel = self.saturateVelocity(nav_xy_vel, nav_z_vel, nav_yaw_vel)

        vel_msg = Twist()
        vel_msg.linear.x = nav_xy_vel[0]
        vel_msg.linear.y = nav_xy_vel[1]
        vel_msg.linear.z = nav_z_vel
        vel_msg.angular.z = nav_yaw_vel

        return vel_msg

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
        if (not self.odom_update_flag_) or (not self.lidar_update_flag_) or (not self.task_start_):
            return

    #navigation
        vel_msg = Twist()
        if self.state_machine_ == self.INITIAL_STATE_:
            vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.z = 0.0
        if self.state_machine_ == self.TAKEOFF_STATE_ or self.state_machine_ == self.TREE_DETECTION_START_STATE_:
            vel_msg = self.goPos(self.GLOBAL_FRAME_, self.initial_xy_pos_, self.takeoff_height_, self.initial_yaw_) #hover
        if self.state_machine_ == self.APPROACHING_TO_TREE_STATE_:
            vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([self.tree_xy_pos_[0] - self.circle_radius_, self.tree_xy_pos_[1]]),
                                                    self.takeoff_height_,
                                                    self.uav_yaw_ + math.atan2(self.tree_xy_pos_[1], self.tree_xy_pos_[0]))
        if self.state_machine_ == self.START_CIRCLE_MOTION_STATE_:
            vel_msg = self.goCircle(self.tree_xy_pos_, self.takeoff_height_, self.circle_y_vel_, self.circle_radius_)
        if self.state_machine_ == self.FINISH_CIRCLE_MOTION_STATE_:
        #use global frame
            #vel_msg = self.goPos(self.GLOBAL_FRAME_, self.circle_initial_xy_, self.takeoff_height_, self.circle_initial_yaw_)
        #use local frame
            vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([self.tree_xy_pos_[0] - self.circle_radius_, self.tree_xy_pos_[1]]), 
                                                    self.takeoff_height_,
                                                    self.uav_yaw_ + math.atan2(self.tree_xy_pos_[1], self.tree_xy_pos_[0]))
            
        if self.state_machine_ == self.RETURN_HOME_STATE_:
        #use global frame
            #vel_msg = self.goPos(self.GLOBAL_FRAME_, self.initial_xy_pos_, self.takeoff_height_, self.initial_yaw_)
        #use local frame
            vel_msg = self.goPos(self.LOCAL_FRAME_, self.tree_xy_pos_ - self.initial_target_tree_xy_pos_, self.takeoff_height_, self.initial_yaw_)
    #end navigation

    #state machine
        if self.state_machine_ == self.INITIAL_STATE_:
            self.initial_xy_pos_ = np.array(self.uav_xy_pos_)
            self.initial_yaw_ = self.uav_yaw_
            if self.use_dji_ == True:
                self.drone_.takeoff()
            self.state_machine_ = self.TAKEOFF_STATE_
            rospy.loginfo("take off")

        elif self.state_machine_ == self.TAKEOFF_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_):
               self.treeDetectionStart()
               self.state_machine_ = self.TREE_DETECTION_START_STATE_

        elif self.state_machine_ == self.TREE_DETECTION_START_STATE_:
            self.tree_detection_wait_count_ += 1
            if self.tree_detection_wait_count_ > self.control_rate_ * self.tree_detection_wait_:
                self.state_machine_ = self.APPROACHING_TO_TREE_STATE_
                self.initial_target_tree_xy_pos_ = np.array(self.tree_xy_pos_)

        elif self.state_machine_ == self.APPROACHING_TO_TREE_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_):
                if self.task_kind_ == 1:
                    self.state_machine_ = self.RETURN_HOME_STATE_
                elif self.task_kind_ == 2:
                    self.state_machine_ = self.START_CIRCLE_MOTION_STATE_
                    self.circle_initial_yaw_ = self.uav_accumulated_yaw_
                    self.circle_initial_xy_ = np.array(self.uav_xy_pos_)

        elif self.state_machine_ == self.START_CIRCLE_MOTION_STATE_:
            if abs(self.circle_initial_yaw_ - self.uav_accumulated_yaw_) > 2 * math.pi:
                self.state_machine_ = self.FINISH_CIRCLE_MOTION_STATE_
                        
        elif self.state_machine_ == self.FINISH_CIRCLE_MOTION_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_):
                self.state_machine_ = self.RETURN_HOME_STATE_
        
        elif self.state_machine_ == self.RETURN_HOME_STATE_:
            pass
    #end state machine
    
    #publication
        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])
            
        target_pos_msg = Odometry()
        target_pos_msg.header = self.odom_.header
        if self.target_frame_ == self.GLOBAL_FRAME_:
            target_pos_msg.child_frame_id = "global"
        elif self.target_frame_ == self.LOCAL_FRAME_:
            target_pos_msg.child_frame_id = "local"
        target_pos_msg.pose.pose.position.x = self.target_xy_pos_[0]
        target_pos_msg.pose.pose.position.y = self.target_xy_pos_[1]
        target_pos_msg.pose.pose.position.z = self.target_z_pos_
        target_pos_msg.pose.pose.orientation.w = self.target_yaw_
        self.target_pos_pub_.publish(target_pos_msg)

        self.vel_pub_.publish(vel_msg)
        if self.use_dji_ == True:
            self.drone_.velocity_control(0, vel_msg.linear.x, -vel_msg.linear.y, vel_msg.linear.z, -vel_msg.angular.z * 180 / math.pi) #machine frame yaw_rate[deg]

if __name__ == '__main__':
    try:
        circle_motion = CircleMotion()
        circle_motion.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
