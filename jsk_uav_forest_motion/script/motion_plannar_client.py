#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
import numpy as np
from dji_sdk.dji_drone import DJIDrone
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

class MotionPlannar:
    
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
            rospy.init_node('motion_plannar', anonymous=True)

        self.odom_ = Odometry()
        self.GLOBAL_FRAME_ = 0
        self.LOCAL_FRAME_ = 1
        self.target_xy_global_pos_ = np.zeros(2)
        self.target_xy_local_pos_ = np.zeros(2)
        self.global_landmark_xy_pos_ = np.zeros(2)
        self.target_z_pos_ = 0.0
        self.midway_landmark_xy_pos_ = np.zeros(2)
        self.target_yaw_ = 0.0
        self.target_frame_ = self.GLOBAL_FRAME_
        self.tree_cluster_ = LaserScan()

        #state machine
        self.INITIAL_STATE_ = 0
        self.ADJUST_YAW_STATE = 1
        self.SAFE_FLYING_STATE = 2
        self.AVOID_OBSTACLE_STATE = 3
        self.FINISH_GOPOS_STATE = 4
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "adjust_yaw", "safe_flying", "avoid_obstacle", "finish_gopos"]

        self.uav_xy_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_yaw_ = 0.0
    
        self.vel_pub_topic_name_ = rospy.get_param("~vel_pub_topic_name", "cmd_vel")
        self.state_machine_pub_topic_name_ = rospy.get_param("~plannar_state_machine_pub_topic_name", "plannar_state_machine")
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "ground_truth/state")
        self.target_pos_sub_topic_name_ = rospy.get_param("~target_pos_pub_topic_name", "uav_target_pos")
        self.tree_cluster_sub_topic_name_ = rospy.get_param("~tree_cluster_sub_topic_name", "tree_cluster")
        self.control_rate_ = rospy.get_param("~control_rate", 20)
        self.nav_xy_pos_pgain_ = rospy.get_param("~nav_xy_pos_pgain", 1.0)
        self.nav_z_pos_pgain_ = rospy.get_param("~nav_z_pos_pgain", 1.0)
        self.nav_yaw_pgain_ = rospy.get_param("~nav_yaw_pgain", 1.0)
        self.nav_xy_vel_thresh_ = rospy.get_param("~nav_xy_vel_thresh", 2.0)
        self.nav_z_vel_thresh_ = rospy.get_param("~nav_z_vel_thresh", 2.0)
        self.nav_yaw_vel_thresh_ = rospy.get_param("~nav_yaw_vel_thresh", 1.0)
        self.global_landmark_change_thresh_= rospy.get_param("~global_landmark_change_thresh", 1.0)
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.1)
        self.nav_yaw_convergence_thresh_ = rospy.get_param("~nav_yaw_convergence_thresh", 0.05)
        self.nav_vel_convergence_thresh_ = rospy.get_param("~nav_vel_convergence_thresh", 0.1)
        self.task_kind_ = rospy.get_param("~task_kind", 1) #1 yosen 2 honsen 3 kesshou
        self.drone_obstacle_ignore_maximum_radius_ = rospy.get_param("~drone_obstacle_ignore_maximum_radius", 3.0)
        self.drone_safety_minimum_radius_ = rospy.get_param("~drone_safety_minimum_radius", 0.75)

        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)
        self.target_pos_sub_ = rospy.Subscriber(self.target_pos_sub_topic_name_, Odometry, targetCallback)
        self.odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.odomCallback)
        self.tree_cluster_sub_ = rospy.Subscriber(self.tree_cluster_sub_topic_name_, LaserScan, self.treeClusterCallback)

    def odomCallback(self, msg):
        self.odom_ = msg
        self.uav_xy_pos_ = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        self.uav_yaw_ = tf.transformations.euler_from_quaternion(quaternion)[2]

    def treeClusterCallback(self, msg):
        self.tree_cluster_ = *msg

    def cvtLocaltoGlobal(self, local_pos):
        rot_mat = np.array([[math.cos(self.uav_yaw_), -math.sin(self.uav_yaw_)], [math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
        global_pos = np.dot(rot_mat, local_pos) + self.uav_xy_pos_
        return global_pos

    def cvtGlobaltoLocal(self, global_pos):
        rot_mat = np.array([[math.cos(self.uav_yaw_), -math.sin(self.uav_yaw_)], [math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
        local_pos = np.dot(inv(rot_mat), global_pos - self.uav_xy_pos_)
        return local_pos
    
    def targetCallback(self, msg):
        target_x_pos = msg.pose.pose.position.x
        target_y_pos = msg.pose.pose.position.y
        self.target_z_pos_ = msg.pose.pose.position.z
        self.target_yaw_ = msg.pose.pose.orientation.w
        if msg.child_frame_id == "local":
            self.target_xy_local_pos_[0] = target_x_pos
            self.target_xy_local_pos_[1] = target_y_pos
            self.target_xy_global_pos_ = self.cvtLocaltoGlobal(self.target_xy_local_pos_)
        else:
            self.target_xy_global_pos_[0] = target_x_pos
            self.target_xy_global_pos_[1] = target_y_pos
            self.target_xy_local_pos_ = self.cvtGlobaltoLocal(self.target_xy_global_pos_)

        ## When target global position changes more than threshold or first time get, re-do global planning
        if np.sqrt(np.sum((self.target_xy_global_pos_ - self.global_landmark_xy_pos_) ** 2)) > self.global_landmark_change_thresh_:
            self.state_machine_ = self.ADJUST_YAW_STATE
            self.global_landmark_xy_pos_ = self.target_xy_global_pos_

    def nearestObstacleSafetyDetection(self):
        nearest_obstalce_distance = 10000.0
        nearest_obstalce_id = 0
        for i in range(0, tree_cluster.ranges.size()):
            if not math.isnan(tree_cluster.ranges[i]):
                ## Here, x > 0.01 means x != 0.0
                if abs(tree_cluster.ranges[i]) > 0.01 and abs(tree_cluster.ranges[i]) < nearest_obstalce_distance:
                    nearest_obstalce_distance = abs(tree_cluster.ranges[i])
                    nearest_obstalce_id = i
        nearest_obstacle_angle = nearest_obstalce_id * tree_cluster.angle_increment + tree_cluster.angle_min
        nearest_obstacle_distance_to_head_direction = nearest_obstalce_distance * math.sin(math.pi / 2.0 - nearest_obstacle_angle)
        
        if nearest_obstacle_distance  < self.drone_obstacle_ignore_maximum_radius_ and nearest_obstacle_distance_to_head_direction < self.drone_safety_minimum_radius_:
            landmark_distance = np.sqrt(np.sum((self.uav_xy_pos_ - self.global_landmark_xy_pos_) ** 2))
            obstacle_to_landmark_edge = landmark_distance - nearest_obstalce_distance * math.cos(math.pi / 2.0 - nearest_obstacle_angle)
            theta1 = math.asin(nearest_obstacle_distance_to_head_direction / obstacle_to_landmark_edge)
            theta2 = math.asin(self.drone_safety_minimum_radius_ / obstacle_to_landmark_edge) - theta1
            ## Right hand coordinate, head direction is x axis
            if nearest_obstacle_angle <= math.pi / 2.0:
                shift_distance = -landmark_distance * math.tan(theta2)
            else:
                shift_distance = landmark_distance * math.tan(theta2)
            return [True, shift_distance]
        else:
            return [False]
        
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
        return vel_msg

    def controlCallback(self, event):
        if (self.state_machine_ == self.INITIAL_STATE_):
            return
        elif (self.state_machine_ == self.ADJUST_YAW_STATE):
            vel_msg = self.goPos(self.LOCAL_FRAME_, np.zeros(2), self.target_z_pos_, self.target_yaw_)
        elif (self.state_machine_ == self.AVOID_OBSTACLE_STATE):
            vel_msg = self.goPos(self.GLOBAL_FRAME_, self.midway_landmark_xy_pos_, self.target_z_pos_, self.target_yaw_)
        elif (self.state_machine_ == self.SAFE_FLYING_STATE):
            vel_msg = self.goPos(self.GLOBAL_FRAME_, self.global_landmark_xy_pos_, self.target_z_pos_, self.target_yaw_)

            
        if (self.state_machine_ == self.ADJUST_YAW_STATE):
            if self.isConvergent(self.LOCAL_FRAME_, np.zeros(2), self.target_z_pos_, self.target_yaw_):
               self.state_machine_ = self.SAFE_FLYING_STATE
               
        if (self.state_machine_ = self.SAFE_FLYING_STATE):
            nearest_obstacle = self.nearestObstacleSafetyDetection()
            if nearest_obstacle[0]:
                self.state_machine_ = self.AVOID_OBSTACLE_STATE
                self.midway_landmark_xy_pos_ = self.cvtLocaltoGlobal(np.array([0, nearest_obstacle[1]]))
                
        if (self.state_machine_ = self.AVOID_OBSTACLE_STATE):
            if self.isConvergent(self.GLOBAL_FRAME_, self.midway_landmark_xy_pos_, self.target_z_pos_, self.target_yaw_):
                self.state_machine_ = self.ADJUST_YAW_STATE
        
        #publish
        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])
        
        self.vel_pub_.publish(vel_msg)
        if self.use_dji_ == True:
            self.drone_.velocity_control(0, vel_msg.linear.x, -vel_msg.linear.y, vel_msg.linear.z, -vel_msg.angular.z * 180 / math.pi) #machine frame

if __name__ == '__main__':
    try:
        motion_plannar = MotionPlannar()
        motion_plannar.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
