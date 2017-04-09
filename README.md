[![Build Status](https://travis-ci.org/tongtybj/jsk_uav_forest.svg?branch=master)](https://travis-ci.org/tongtybj/jsk_uav_forest)

## Introduction

This is the repository for the uav challenge in forest:
[森のドローン・ロボット競技会](http://www.lsse.kyutech.ac.jp/~sociorobo/ja/forestdrone17)

## how to compile

```
mkdir <catkin_ws>
cd <catkin_ws>
wstool init src
wstool set -t src jsk_uav_forest http://github.com/tongtybj/jsk_uav_forest --git
wstool merge -t src https://raw.githubusercontent.com/tongtybj/jsk_uav_forest/jsk_uav_forest.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## how to run program

- run in gazebo:
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch
```

- run in gazebo by manual operation
```
$ roslaunch jsk_uav_forest_simulation forest_simulation.launch manual:=true
```
