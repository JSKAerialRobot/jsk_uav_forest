#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
import numpy as np
from dji_sdk.dji_drone import DJIDrone
from geometry_msgs.msg import Twist, Quaternion, PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool, ColorRGBA, Empty
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse, SetBool

from jsk_rviz_plugins.msg import OverlayText

class ForestMotion:

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
        self.initial_xy_global_pos_ = np.zeros(2)
        self.initial_z_pos_ = 0.0
        self.initial_yaw_ = 0.0

        #state machine
        self.INITIAL_STATE_ = 0
        self.TAKEOFF_STATE_ = 1
        self.TREE_DETECTION_START_STATE_ = 2
        self.APPROACHING_TO_TREE_STATE_ = 3
        self.START_CIRCLE_MOTION_STATE_ = 4
        self.FINISH_CIRCLE_MOTION_STATE_ = 5
        self.TURN_STATE_ = 6
        self.RETURN_HOME_STATE_ = 7
        self.FINISH_STATE_ = 8
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "takeoff", "tree detection start", "approaching to tree", "circle motion", "finish circle motion", "turn", "return home", "finish"]
        self.circle_motion_count_ = 0
        self.target_count_ = 0

        self.odom_update_flag_ = False
        self.uav_xy_global_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_yaw_ = 0.0
        self.uav_yaw_old_ = 0.0
        self.uav_yaw_overflow_ = 0
        self.uav_accumulated_yaw_ = 0.0
        self.tree_cluster_ = LaserScan()

        self.tree_xy_local_pos_ = np.zeros(2)
        self.tree_pos_update_flag_ = False

        self.task_start_ = False
        self.task_start_time_ = rospy.Time.now()
        self.task_elapsed_time_ = rospy.Time(0)

        self.turn_uav_xy_global_pos_ = np.zeros(2)

        self.vel_pub_topic_name_ = rospy.get_param("~vel_pub_topic_name", "cmd_vel")
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "state_machine")
        self.target_pos_pub_topic_name_ = rospy.get_param("~target_pos_pub_topic_name", "uav_target_pos")
        self.tracking_control_pub_topic_name_ = rospy.get_param("~tracking_control_pub_topic_name", "/tracking_control")
        self.state_visualization_pub_topic_name_ = rospy.get_param("state_visualization_pub_topic_name", "overlay_text")
        self.task_start_sub_topic_name_ = rospy.get_param("~task_start_sub_topic_name", "task_start")
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "ground_truth/state")
        self.tree_location_sub_topic_name_ = rospy.get_param("~tree_location_sub_topic_name", "tree_location")
        self.tree_detection_start_pub_topic_name_ = rospy.get_param("~tree_detection_start_pub_topic_name", "detection_start")
        self.tree_cluster_sub_topic_name_ = rospy.get_param("~tree_cluster_sub_topic_name", "scan_clustered")
        self.update_target_tree_service_name_ = rospy.get_param("~update_target_tree_service_name", "/update_target_tree")
        self.global_state_name_sub_topic_name_ = rospy.get_param("~global_state_name_sub_topic_name", "state_machine")
        self.set_first_tree_service_name_ = rospy.get_param("~set_first_tree_service_name_", "/set_first_tree")

        self.control_rate_ = rospy.get_param("~control_rate", 20)
        
        #navigation
        self.takeoff_height_ = rospy.get_param("~takeoff_height", 1.5)
        self.return_height_ = rospy.get_param("~return_height", 1.5)
        self.nav_xy_pos_pgain_ = rospy.get_param("~nav_xy_pos_pgain", 1.0)
        self.nav_z_pos_pgain_ = rospy.get_param("~nav_z_pos_pgain", 1.0)
        self.nav_yaw_pgain_ = rospy.get_param("~nav_yaw_pgain", 1.0)
        self.nav_xy_vel_thresh_ = rospy.get_param("~nav_xy_vel_thresh", 2.0)
        self.nav_z_vel_thresh_ = rospy.get_param("~nav_z_vel_thresh", 2.0)
        self.nav_yaw_vel_thresh_ = rospy.get_param("~nav_yaw_vel_thresh", 1.0)
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.1)
        self.nav_yaw_convergence_thresh_ = rospy.get_param("~nav_yaw_convergence_thresh", 0.05)
        self.nav_vel_convergence_thresh_ = rospy.get_param("~nav_vel_convergence_thresh", 0.1)
        self.takeoff_forward_offset_ = rospy.get_param("~takeoff_forward_offset", 4.0)
        self.deep_return_dist_ = rospy.get_param("~deep_return_dist", -0.6)
        self.turn_radius_offset_ = rospy.get_param("~turn_radius_offset", -1.0)    
        
        #circle motion
        self.circle_radius_ = rospy.get_param("~circle_radius", 1.0)
        self.circle_y_vel_ = rospy.get_param("~circle_y_vel", 0.5)
        
        #obstacle avoidance
        self.cluster_num_min_ = rospy.get_param("~cluster_num_min", 10)
        self.safe_zone_radius_ = rospy.get_param("~safe_zone", 1.0)
        self.drone_obstacle_ignore_maximum_radius_ = rospy.get_param("~drone_obstacle_ignore_maximum_radius", 0.90)
        self.drone_safety_minimum_radius_ = rospy.get_param("~drone_safety_minimum_radius", 0.85)
        self.avoid_vel_ = rospy.get_param("~avoid_vel", 0.5)
        
        #flag
        self.task_kind_ = rospy.get_param("~task_kind", 1) #1 yosen 2 honsen 3 kesshou
        self.do_avoidance_ = rospy.get_param("~do_avoidance", False)
        self.turn_before_return_ = rospy.get_param("~turn_before_return", True)
        self.visualization_ = rospy.get_param("~visualization", True)

        ## task3 different
        self.circle_motion_times_ = 1
        self.circle_motion_height_step_ = 0
        if self.task_kind_ == 2:
            self.circle_motion_times_ = rospy.get_param("~circle_motion_times", 1)
            self.circle_motion_height_step_ = rospy.get_param("~circle_motion_height_step_", 0.5)

        self.target_num_ = 1
        if self.task_kind_ == 3:
            self.target_num_ = rospy.get_param("~target_num", 2)
            self.turn_before_return_ = True
            self.do_avoidance_ = True

        self.vel_pub_ = rospy.Publisher(self.vel_pub_topic_name_, Twist, queue_size = 10)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)
        self.target_pos_pub_ = rospy.Publisher(self.target_pos_pub_topic_name_, Odometry, queue_size = 10)
        self.tracking_control_pub_ = rospy.Publisher(self.tracking_control_pub_topic_name_, Bool, queue_size = 1)
        self.tree_detection_start_pub_ = rospy.Publisher(self.tree_detection_start_pub_topic_name_, Bool, queue_size = 10)
        self.state_visualization_pub_ = rospy.Publisher(self.state_visualization_pub_topic_name_, OverlayText, queue_size = 10)
        self.task_start_sub_ = rospy.Subscriber(self.task_start_sub_topic_name_, Empty, self.taskStartCallback)
        self.odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.odomCallback)
        self.tree_location_sub_ = rospy.Subscriber(self.tree_location_sub_topic_name_, PointStamped, self.treeLocationCallback)
        self.tree_cluster_sub_ = rospy.Subscriber(self.tree_cluster_sub_topic_name_, LaserScan, self.treeClusterCallback)
        
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

    def taskStartCallback(self, msg):
        rospy.loginfo("Task Start")
        self.task_start_time_ = rospy.Time.now()
        self.task_start_ = True
        self.task_start_sub_.unregister()
    
    def odomCallback(self, msg):
        self.odom_ = msg
        self.uav_xy_global_pos_ = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

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
        msg = Bool()
        msg.data = True
        self.tree_detection_start_pub_.publish(msg)

    def treeClusterCallback(self, msg):
        self.tree_cluster_ = msg

    def treeLocationCallback(self, msg):
        self.tree_pos_update_flag_ = True
        self.tree_xy_local_pos_ = np.array([msg.point.x, msg.point.y])

    def isConvergent(self, frame, target_xy_pos, target_z_pos, target_yaw):
        if frame == self.GLOBAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0] - self.uav_xy_global_pos_[0], target_xy_pos[1] - self.uav_xy_global_pos_[1], target_z_pos - self.uav_z_pos_])
            #delta_pos = np.array([target_xy_pos[0] - self.uav_xy_global_pos_[0], target_xy_pos[1] - self.uav_xy_global_pos_[1], 0])
        elif frame == self.LOCAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0], target_xy_pos[1], target_z_pos - self.uav_z_pos_])
            #delta_pos = np.array([target_xy_pos[0], target_xy_pos[1], 0])
        else:
            return

        delta_yaw = target_yaw - self.uav_yaw_
        if delta_yaw > math.pi:
            delta_yaw -= math.pi * 2
        elif delta_yaw < -math.pi:
            delta_yaw += math.pi * 2

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
            delta_xy = np.dot(rot_mat, target_xy - self.uav_xy_global_pos_)
        elif frame == self.LOCAL_FRAME_:
            delta_xy = target_xy
        else:
            return

        delta_z = target_z - self.uav_z_pos_

        delta_yaw = target_yaw - self.uav_yaw_
        if delta_yaw > math.pi:
            delta_yaw -= math.pi * 2
        elif delta_yaw < -math.pi:
            delta_yaw += math.pi * 2

        nav_xy_vel = delta_xy * self.nav_xy_pos_pgain_
        nav_z_vel = delta_z * self.nav_z_pos_pgain_
        nav_yaw_vel = delta_yaw * self.nav_yaw_pgain_

        nav_xy_vel, nav_z_vel, nav_yaw_vel = self.saturateVelocity(nav_xy_vel, nav_z_vel, nav_yaw_vel)
        vel_msg = Twist()
        vel_msg.linear.x = nav_xy_vel[0]
        vel_msg.linear.y = nav_xy_vel[1]
        vel_msg.linear.z = nav_z_vel
        vel_msg.angular.z = nav_yaw_vel

        self.target_xy_pos_ = target_xy
        self.target_z_pos_ = target_z
        self.target_yaw_ = target_yaw
        self.target_frame_ = frame

        return vel_msg

    def goCircle(self, circle_center_local_xy, target_z, target_y_vel, radius):
        nav_xy_vel = np.zeros(2)
        nav_xy_vel[0] = -(radius - circle_center_local_xy[0]) * self.nav_xy_pos_pgain_
        nav_xy_vel[1] = target_y_vel
        nav_z_vel = target_z - self.uav_z_pos_
        nav_yaw_vel = math.atan2(circle_center_local_xy[1], circle_center_local_xy[0]) * self.nav_yaw_pgain_ - target_y_vel / radius #feedback+feedforward
        nav_xy_vel, nav_z_vel, nav_yaw_vel = self.saturateVelocity(nav_xy_vel, nav_z_vel, nav_yaw_vel)

        vel_msg = Twist()
        vel_msg.linear.x = nav_xy_vel[0]
        vel_msg.linear.y = nav_xy_vel[1]
        vel_msg.linear.z = nav_z_vel
        vel_msg.angular.z = nav_yaw_vel

        return vel_msg


    def obstacleDetection(self):
        if not self.do_avoidance_:
            return [False]

        if self.state_machine_ != self.TAKEOFF_STATE_ and self.state_machine_ != self.APPROACHING_TO_TREE_STATE_ and self.state_machine_ != self.RETURN_HOME_STATE_:
            return [False]

        ## If closed to the target tree within a certain distance, that is safe zone,
        ## we do not need to the obstacle avoidance.
        dist = np.linalg.norm(self.target_xy_pos_)
        if self.target_frame_ == self.GLOBAL_FRAME_:
            dist = np.linalg.norm(self.target_xy_pos_ - self.uav_xy_global_pos_)
        if dist < self.safe_zone_radius_:
           #rospy.loginfo("safe zone, distance :%f", dist)
           return [False]
        nearest_obstacle_distance_to_head_direction = 0.0
        nearest_obstacle_distance_in_head_direction = 10000.0
        nearest_obstacle_id = -1
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
                    obstacle_distance_to_head_direction = abs(self.tree_cluster_.ranges[i] * math.sin(obstacle_angle))
                    if obstacle_distance_to_head_direction < self.drone_safety_minimum_radius_:
                        ## the closest obstacle in head direction influence in earlist time
                        obstacle_distance_in_head_direction = abs(self.tree_cluster_.ranges[i]) * math.cos(obstacle_angle)
                        if obstacle_distance_in_head_direction < nearest_obstacle_distance_in_head_direction:
                            nearest_obstacle_distance_in_head_direction = obstacle_distance_in_head_direction
                            nearest_obstacle_distance_to_head_direction = obstacle_distance_to_head_direction
                            nearest_obstacle_id = i
        if nearest_obstacle_id >= 0:
            nearest_obstacle_angle = nearest_obstacle_id * self.tree_cluster_.angle_increment + self.tree_cluster_.angle_min
            # rospy.logwarn("Meet obstacle: [%.3f, %.3f] -> %.3f[m], %.3f[deg] in phase: %s", nearest_obstacle_distance_in_head_direction, nearest_obstacle_distance_to_head_direction, self.tree_cluster_.ranges[nearest_obstacle_id], nearest_obstacle_angle / math.pi * 180.0, self.state_name_[self.state_machine_])

            return [True, nearest_obstacle_angle]
        else:
            return [False]

    def controlCallback(self, event):
        if (not self.odom_update_flag_) or (not self.task_start_):
            return

        #navigation
        vel_msg = Twist()
        if self.state_machine_ == self.INITIAL_STATE_:
            vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.z = 0.0
        if self.state_machine_ == self.TAKEOFF_STATE_:
            rot_mat = np.array([[math.cos(self.initial_yaw_), -math.sin(self.initial_yaw_)],[math.sin(self.initial_yaw_), math.cos(self.initial_yaw_)]])
            takeoff_xy_global_pos_ = self.initial_xy_global_pos_ + np.dot(rot_mat, np.array([self.takeoff_forward_offset_, 0]))
            vel_msg = self.goPos(self.GLOBAL_FRAME_, takeoff_xy_global_pos_, self.takeoff_height_, self.initial_yaw_) 
        if self.state_machine_ == self.TREE_DETECTION_START_STATE_:
            vel_msg = self.goPos(self.GLOBAL_FRAME_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_) #hover
        if self.state_machine_ == self.APPROACHING_TO_TREE_STATE_:
            tree_direction = math.atan2(self.tree_xy_local_pos_[1], self.tree_xy_local_pos_[0])
            vel_msg = self.goPos(self.LOCAL_FRAME_,
                                 np.array([self.tree_xy_local_pos_[0] - self.circle_radius_ * math.cos(tree_direction),
                                           self.tree_xy_local_pos_[1] - self.circle_radius_ * math.sin(tree_direction)]),
                                 self.takeoff_height_,
                                 self.uav_yaw_ + tree_direction)

        if self.state_machine_ == self.START_CIRCLE_MOTION_STATE_:
            vel_msg = self.goCircle(self.tree_xy_local_pos_, self.takeoff_height_ + self.circle_motion_count_ * self.circle_motion_height_step_, self.circle_y_vel_, self.circle_radius_)
        if self.state_machine_ == self.FINISH_CIRCLE_MOTION_STATE_:
            tree_direction = math.atan2(self.tree_xy_local_pos_[1], self.tree_xy_local_pos_[0])
            vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([self.tree_xy_local_pos_[0] - self.circle_radius_,
                                                              self.tree_xy_local_pos_[1]]),
                                 self.takeoff_height_,
                                 self.circle_initial_yaw_)

        if self.state_machine_ == self.TURN_STATE_:
            vel_msg = self.goPos(self.GLOBAL_FRAME_, self.turn_uav_xy_global_pos_, self.takeoff_height_, self.initial_yaw_ + math.pi)
        if self.state_machine_ == self.RETURN_HOME_STATE_:
            if self.turn_before_return_ == False:
                #use local frame
                vel_msg = self.goPos(self.LOCAL_FRAME_, self.tree_xy_local_pos_ - self.initial_target_tree_xy_local_pos_ - np.array([self.takeoff_forward_offset_, 0]), self.return_height_, self.initial_yaw_)
            else:
                #use global fram
                vel_msg = self.goPos(self.GLOBAL_FRAME_, self.final_xy_global_pos_ + self.final_target_tree_xy_global_pos_ - self.initial_target_tree_xy_global_pos_, self.return_height_, self.initial_yaw_ + math.pi)

        # obstacle avoidance
        obstacle = self.obstacleDetection()
        if obstacle[0]:
            # print "avoidance activate"
            ## if nearest obstacle is on drone right side, drone moves left
            if obstacle[1] < 0.0:
                vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([0.0, self.avoid_vel_]), self.target_z_pos_, self.target_yaw_)
            ## if nearest obstacle is on drone left side, drone moves right
            else:
                vel_msg = self.goPos(self.LOCAL_FRAME_, np.array([0.0, -self.avoid_vel_]), self.target_z_pos_, self.target_yaw_)
        #end navigation

        #state machine
        if self.state_machine_ == self.INITIAL_STATE_:
            self.initial_xy_global_pos_ = np.array(self.uav_xy_global_pos_)
            rot_mat = np.array([[math.cos(self.uav_yaw_), -math.sin(self.uav_yaw_)],[math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
            self.final_xy_global_pos_ = np.dot(rot_mat, np.array([self.deep_return_dist_,0])) + self.initial_xy_global_pos_
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
            if self.tree_pos_update_flag_ == True:
                self.state_machine_ = self.APPROACHING_TO_TREE_STATE_
                rot_mat = np.array([[math.cos(self.uav_yaw_), -math.sin(self.uav_yaw_)],[math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
                self.initial_target_tree_xy_global_pos_ = np.dot(rot_mat, self.tree_xy_local_pos_) + self.uav_xy_global_pos_
                self.initial_target_tree_xy_local_pos_ = self.tree_xy_local_pos_

        elif self.state_machine_ == self.APPROACHING_TO_TREE_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_):
                self.circle_initial_accumulated_yaw_ = self.uav_accumulated_yaw_
                self.circle_initial_yaw_ = self.uav_yaw_
                self.circle_initial_xy_ = np.array(self.uav_xy_global_pos_)
                if self.task_kind_ == 1:
                    self.state_machine_ = self.FINISH_CIRCLE_MOTION_STATE_
                elif self.task_kind_ > 1:
                    if self.task_kind_ == 3 and self.target_count_ == self.target_num_:
                        self.state_machine_ = self.FINISH_CIRCLE_MOTION_STATE_
                    else:
                        self.state_machine_ = self.START_CIRCLE_MOTION_STATE_

        elif self.state_machine_ == self.START_CIRCLE_MOTION_STATE_:
            if abs(self.circle_initial_accumulated_yaw_ - self.uav_accumulated_yaw_) > 2 * math.pi:
                self.circle_initial_accumulated_yaw_ = self.uav_accumulated_yaw_
                if self.task_kind_ == 2:
                    self.circle_motion_count_ += 1
                    if self.circle_motion_count_ == self.circle_motion_times_:
                        self.state_machine_ = self.FINISH_CIRCLE_MOTION_STATE_
                else:
                    self.state_machine_ = self.FINISH_CIRCLE_MOTION_STATE_

        elif self.state_machine_ == self.FINISH_CIRCLE_MOTION_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_):

                # task3 special process
                self.target_count_ += 1
                if self.task_kind_ == 3: 
                    if self.target_count_ < self.target_num_:
                        rospy.wait_for_service(self.update_target_tree_service_name_)
                        try:
                            update_target_tree = rospy.ServiceProxy(self.update_target_tree_service_name_, Trigger)
                            res = update_target_tree()
                            
                            if res.success:
                                self.state_machine_ = self.TREE_DETECTION_START_STATE_
                                self.tree_pos_update_flag_ = False
                                time.sleep(0.5)
                                return
                            else:
                                self.target_count_ = self.target_num_
                                rospy.logerr("can not get next target tree")
                        except rospy.ServiceException, e:
                            print "Service call failed: %s"%e
                    
                    if self.target_count_ == self.target_num_:
                        rospy.wait_for_service(self.set_first_tree_service_name_)
                        try:
                            set_first_tree = rospy.ServiceProxy(self.set_first_tree_service_name_, Trigger)
                            res = set_first_tree()
        
                            if res.success:
                                self.state_machine_ = self.TREE_DETECTION_START_STATE_
                                self.tree_pos_update_flag_ = False
                                time.sleep(0.5)
                                return
                        except rospy.ServiceException, e:
                            print "Service call failed: %s"%e

                if self.turn_before_return_ == True:
                    rot_mat = np.array([[math.cos(self.uav_yaw_), -math.sin(self.uav_yaw_)],[math.sin(self.uav_yaw_), math.cos(self.uav_yaw_)]])
                    self.final_target_tree_xy_global_pos_ = np.dot(rot_mat, self.tree_xy_local_pos_) + self.uav_xy_global_pos_
                    rot_mat = np.array([[math.cos(self.initial_yaw_), -math.sin(self.initial_yaw_)],[math.sin(self.initial_yaw_), math.cos(self.initial_yaw_)]])
                    if self.task_kind_ != 3:
                        self.turn_uav_xy_global_pos_ = np.dot(rot_mat, np.array([self.turn_radius_offset_, 0])) + self.uav_xy_global_pos_
                    else:
                        self.turn_uav_xy_global_pos_ = self.uav_xy_global_pos_
                    #self.turn_uav_yaw_ = self.uav_yaw_
                    self.state_machine_ = self.TURN_STATE_

                    # stop tree tracking if necessary
                    stop_msg = Bool()
                    self.tracking_control_pub_.publish(stop_msg)
                else:
                    self.state_machine_ = self.RETURN_HOME_STATE_

        elif self.state_machine_ == self.TURN_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_):
                self.state_machine_ = self.RETURN_HOME_STATE_
                # TODO: stop mapping and tree database update

        elif self.state_machine_ == self.RETURN_HOME_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_, self.target_yaw_):
                self.state_machine_ = self.FINISH_STATE_

        elif self.state_machine_ == self.FINISH_STATE_:
            pass
        #end state machine

        #publication
        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])

        if self.visualization_ == True:
            if self.state_machine_ != self.FINISH_STATE_:
                self.task_elapsed_time = (rospy.Time.now() - self.task_start_time_).to_sec()

            text_msg = OverlayText()
            text_msg.width = 500
            text_msg.height = 110
            text_msg.left = 10
            text_msg.top = 10
            text_msg.text_size = 20
            text_msg.line_width = 2
            text_msg.font = "DejaVu Sans Mono"
            text_msg.text = """Task Kind:%d
                               State:%d,%s
                               Time:%.1f  """ % (self.task_kind_, self.state_machine_, self.state_name_[self.state_machine_], self.task_elapsed_time)
            text_msg.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
            text_msg.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
            self.state_visualization_pub_.publish(text_msg)

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
        forest_motion = ForestMotion()
        forest_motion.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
