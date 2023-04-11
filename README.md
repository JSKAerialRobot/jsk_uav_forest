[![Build Status](https://travis-ci.org/tongtybj/jsk_uav_forest.svg?branch=master)](https://travis-ci.org/tongtybj/jsk_uav_forest)

## Introduction

This is the repository for the uav challenge in forest:
[森のドローン・ロボット競技会](http://www.lsse.kyutech.ac.jp/~sociorobo/ja/forestdrone17)

![](jsk_uav_forest_common/images/demo.gif)


## how to compile

```
mkdir <catkin_ws>
cd <catkin_ws>
wstool init src
wstool set -t src jsk_uav_forest http://github.com/JSKAerialRobot/jsk_uav_forest --git
wstool merge -t src https://raw.githubusercontent.com/JSKAerialRobot/jsk_uav_forest/master/jsk_uav_forest.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## how to run program in simulation

- run in gazebo by autonomous perception and motion
1. task1 
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch task_kind:=1
```
2. task2
```
roslaunch jsk_uav_forest_simulation forest_simulation.launch task_kind:=2 circle_motion_times:=2
```
3. task3
```
roslaunch jsk_uav_forest_simulation forest_simulation.launch task_kind:=3 target_num:=3
```

start by clicking the `Send Topic` button in the left-bottom corner of RVIZ

- run in gazebo by manual operation
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch manual:=true
```

use terminal to operate the quadrotor


## forest challenge (using DJI M100 + DJI Guidance + Pointgrey Chameleon3 + Hokuyo UST20LX)
1. integrated launch file in UAV
```
$ roslaunch jsk_uav_forest_common challenge.launch (same options with the above cases according to different tasks)
```

2. remote communication in UAV
```
$ roslaunch jsk_uav_forest_common communication2ground_station.launch UAV_IP:=10.42.0.1 GROUND_STATION_IP:=10.42.0.XXX
```
    10.42.0.XXX is the IP address of you rmeote PC. 

3. remote communication in Remote PC
```
 roslaunch jsk_uav_forest_common ground_station.launch UAV_IP:=10.42.0.1 GROUND_STATION_IP:=10.42.0.XXX
```
    10.42.0.XXX is the IP address of you rmeote PC. 

4. start the task
```
$ rostopic pub /task_start std_msgs/Empty "{}"
```

## view the result
```
$ roslaunch jsk_uav_forest_common result.launch
```

default data is the result from the gazebo.
