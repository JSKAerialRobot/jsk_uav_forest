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

        rospy.loginfo("Using motion planning!!!")

        self.odom_ = Odometry()
        self.GLOBAL_FRAME_ = 0
        self.LOCAL_FRAME_ = 1
        self.target_xy_global_pos_ = np.zeros(2)
        self.target_xy_local_pos_ = np.zeros(2)
        self.global_landmark_xy_pos_ = np.zeros(2)
        self.global_landmark_ang_ = 0.0
        self.target_z_pos_ = 0.0
        self.midway_landmark_xy_pos_ = np.zeros(2)
        self.target_yaw_ = 0.0
        self.target_frame_ = self.GLOBAL_FRAME_
        self.tree_cluster_ = LaserScan()

        #state machine
        self.INITIAL_STATE_ = 0
        self.SAFE_FLYING_STATE_ = 1
        self.AVOID_OBSTACLE_STATE_ = 2
        self.plannar_state_machine_ = self.INITIAL_STATE_
        self.plannar_state_name_ = ["initial", "safe_flying", "avoid_obstacle"]
        self.global_state_machine_ = self.INITIAL_STATE_
        self.global_state_name_ = String()
        self.INITIAL_STATE_ = 0
        self.TAKEOFF_STATE_ = 1
        self.TREE_DETECTION_START_STATE_ = 2
        self.APPROACHING_TO_TREE_STATE_ = 3
        self.START_CIRCLE_MOTION_STATE_ = 4
        self.FINISH_CIRCLE_MOTION_STATE_ = 5
        self.RETURN_HOME_STATE_ = 6
        self.state_name_ = ["initial", "takeoff", "tree detection start", "approaching to tree", "circle motion", "finish circle motion", "return home"]
        self.uav_xy_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_yaw_ = 0.0

        self.vel_pub_topic_name_ = rospy.get_param("~vel_pub_topic_name", "cmd_vel")
        self.plannar_state_machine_pub_topic_name_ = rospy.get_param("~plannar_state_machine_pub_topic_name", "plannar_state_machine")
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "ground_truth/state")
        self.target_pos_sub_topic_name_ = rospy.get_param("~target_pos_sub_topic_name", "uav_target_pos")
        self.tree_cluster_sub_topic_name_ = rospy.get_param("~tree_cluster_sub_topic_name", "scan_clustered")
        self.global_state_name_sub_topic_name_ = rospy.get_param("~global_state_name_sub_topic_name", "state_machine")
        self.control_rate_ = rospy.get_param("~control_rate", 20)
        self.cluster_num_min_ = rospy.get_param("~cluster_num_min", 10)
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
        self.drone_obstacle_ignore_maximum_radius_ = rospy.get_param("~drone_obstacle_ignore_maximum_radius", 0.90)
        self.drone_safety_minimum_radius_ = rospy.get_param("~drone_safety_minimum_radius", 0.85)

        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        self.plannar_state_machine_pub_ = rospy.Publisher(self.plannar_state_machine_pub_topic_name_, String, queue_size = 10)
        self.target_pos_sub_ = rospy.Subscriber(self.target_pos_sub_topic_name_, Odometry, self.targetCallback)
        self.odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.odomCallback)
        self.tree_cluster_sub_ = rospy.Subscriber(self.tree_cluster_sub_topic_name_, LaserScan, self.treeClusterCallback)
        self.global_state_name_sub_ = rospy.Subscriber(self.global_state_name_sub_topic_name_, String, self.globalStateNameCallback)

    def odomCallback(self, msg):
        self.odom_ = msg
        self.uav_xy_pos_ = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        self.uav_yaw_ = tf.transformations.euler_from_quaternion(quaternion)[2]

    def treeClusterCallback(self, msg):
        self.tree_cluster_ = msg

    def globalStateNameCallback(self, msg):
        self.global_state_name_ = msg

    def cvtLocaltoGlobal(self, local_pos):
        rot_mat = np.array([[math.cos(self.uav_yaw_), -math.sin(self.uav_yaw_)], [math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
        global_pos = np.dot(rot_mat, local_pos) + self.uav_xy_pos_
        return global_pos

    def cvtGlobaltoLocal(self, global_pos):
        rot_mat = np.array([[math.cos(self.uav_yaw_), -math.sin(self.uav_yaw_)], [math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
        local_pos = np.dot(np.linalg.inv(rot_mat), global_pos - self.uav_xy_pos_)
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

        ## when first time to get target information, intialize plannar state to SAFE_FLYING_STATE to get ready to move
        if self.plannar_state_machine_ == self.INITIAL_STATE_:
            self.plannar_state_machine_ = self.SAFE_FLYING_STATE_

    def nearestObstacleSafetyDetection(self):
        nearest_obstalce_distance_to_head_direction = 10000.0
        nearest_obstalce_id = -1
        ## get the nearest_obstalce_distance_to_head_direction from laser lines which is shorter than obstacle_ignore radius
        ## obstacle_ignore radius should not be larger than 1m, since task2 it requires drone to arrive within 1m distance to red tree
        ## TODO: add changable obstacle_ignore radius, since 1m is too near for normal obstacle
        for i in range(0, len(self.tree_cluster_.ranges)):

            if self.tree_cluster_.ranges[i] > 0:
                # check whether the obstacle is big enough
                big_cluster = True
                for j in range (-self.cluster_num_min_ / 2, self.cluster_num_min_ / 2):
                    if math.isnan(self.tree_cluster_.ranges[i + j]):
                        big_cluster = False
                        break
                if not big_cluster:
                    continue

                if abs(self.tree_cluster_.ranges[i]) < self.drone_obstacle_ignore_maximum_radius_:
                    obstacle_angle = i * self.tree_cluster_.angle_increment + self.tree_cluster_.angle_min
                    obstalce_distance_to_head_direction = abs(self.tree_cluster_.ranges[i] * math.sin(obstacle_angle))
                    if obstalce_distance_to_head_direction < nearest_obstalce_distance_to_head_direction:
                        nearest_obstacle_distance_to_head_direction = obstalce_distance_to_head_direction
                        nearest_obstalce_id = i
        nearest_obstacle_angle = nearest_obstalce_id * self.tree_cluster_.angle_increment + self.tree_cluster_.angle_min
        ## nearest_obstalce_id >= 0 condition means in previous step, a potential nearest_obstacle is found (nearest_obstalce_id initial value is -1)
        ## when potential nearest_obstacle makes nearest_obstalce_distance_to_head_direction shorter than safety_minimum_radius, then obstacle-avoidance needs consideration
        if nearest_obstalce_id >= 0 and nearest_obstacle_distance_to_head_direction < self.drone_safety_minimum_radius_:
            rospy.logwarn("Meet obstacle: %f, [range] %f, [ang] %f", nearest_obstacle_distance_to_head_direction, self.tree_cluster_.ranges[nearest_obstalce_id], nearest_obstacle_angle / math.pi * 180.0)

            return [True, nearest_obstacle_angle]
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
            delta_xy = self.cvtGlobaltoLocal(target_xy)
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

    def controlCallback(self, event):
        ## navigation control
        vel_msg = Twist()
        ## TODO:
        ## Currently, only consider obstacle avoidance in self.APPROACHING_TO_TREE_STATE_ and self.FINISH_CIRCLE_MOTION_STATE_, where we use self.setGoal() function to publish goal data to motion_plannar_client for safety control.
        if (self.global_state_name_.data == "circle motion" or self.global_state_name_.data == "takeoff" or self.global_state_name_.data == "tree detection start"):
            return
        if (self.plannar_state_machine_ == self.INITIAL_STATE_):
            return
        if (self.plannar_state_machine_ == self.SAFE_FLYING_STATE_):
            ## Judge whether requiring obstacles_avoidance
            nearest_obstacle = self.nearestObstacleSafetyDetection()
            if nearest_obstacle[0]:
                self.plannar_state_machine_ = self.AVOID_OBSTACLE_STATE_
                ## if nearest obstacle is on drone right side, drone moves left
                if nearest_obstacle[1] < 0.0:
                    vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([0.0, 0.5]), self.target_z_pos_, self.target_yaw_)
                ## if nearest obstacle is on drone left side, drone moves right
                else:
                    vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([0.0, -0.5]), self.target_z_pos_, self.target_yaw_)
            ## if no obstacle influence, just follow the command from circle_motion_server
            else:
                vel_msg = self.goPos(self.LOCAL_FRAME_, self.target_xy_local_pos_, self.target_z_pos_, self.target_yaw_)
        elif (self.plannar_state_machine_ == self.AVOID_OBSTACLE_STATE_):
            nearest_obstacle = self.nearestObstacleSafetyDetection()
            if nearest_obstacle[0]:
                if nearest_obstacle[1] < 0.0:
                    vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([0.0, -0.5]), self.target_z_pos_, self.target_yaw_)
                else:
                    vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([0.0, -0.5]), self.target_z_pos_, self.target_yaw_)
            ## if drone is already out of obstacle influence, change back to SAFE_FLYING_STATE
            else:
                self.plannar_state_machine_ = self.SAFE_FLYING_STATE_
                vel_msg = self.goPos(self.LOCAL_FRAME_, self.target_xy_local_pos_, self.target_z_pos_, self.target_yaw_)

        #publish
        self.plannar_state_machine_pub_.publish(self.plannar_state_name_[self.plannar_state_machine_])

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
