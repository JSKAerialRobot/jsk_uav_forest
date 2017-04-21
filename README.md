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
wstool set -t src jsk_uav_forest http://github.com/tongtybj/jsk_uav_forest --git
wstool merge -t src https://raw.githubusercontent.com/tongtybj/jsk_uav_forest/master/jsk_uav_forest.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## how to run program in simulation

- run in gazebo:
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch
```

- run in gazebo by manual operation
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch manual:=true
```

- run in gazebo by autonomous perception and motion
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch
```
  start by calling a rosservice
```
$ rosservice call /task_start true
```


## how to activate plannar

- When adding obstacles, remember not to block the view from drone to red tree:
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch plannar:=true
```


## forest challenge (using DJI M100 + DJI Guidance + Pointgrey Chameleon3 + Hokuyo UST20LX)
1. integrated launch file in UAV
```
$ roslaunch jsk_uav_forest_common challenge.launch
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

