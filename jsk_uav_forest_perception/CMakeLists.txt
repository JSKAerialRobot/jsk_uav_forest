cmake_minimum_required(VERSION 2.8.3)
project(jsk_uav_forest_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  dynamic_reconfigure
  nodelet
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_geometry
  image_transport
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
message(WARNING "OPENCV ${OpenCV_VERSION} FOUND")

generate_dynamic_reconfigure_options(
  cfg/HsvFilter.cfg
  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES tree_tracking
)

###########
## Build ##
###########
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(laser_clustering_filter SHARED src/laser_clustering_filter.cpp)
target_link_libraries(laser_clustering_filter ${catkin_LIBRARIES})

add_library(vision_detection_pluginlib
  src/vision_detector/independent_hsv_filter.cpp
  src/vision_detector/conditional_hsv_filter.cpp
  src/vision_detector/no_image_filter.cpp)
target_link_libraries(vision_detection_pluginlib ${catkin_LIBRARIES})
add_dependencies(vision_detection_pluginlib ${PROJECT_NAME}_gencfg)

add_executable(vision_detection_node src/vision_detection.cpp)
target_link_libraries (vision_detection_node ${catkin_LIBRARIES})
add_dependencies(vision_detection_node ${PROJECT_NAME}_gencfg)

add_library(tree_tracking src/tree_tracking.cpp src/tree_database.cpp src/util/circle_detection.cpp)
target_link_libraries(tree_tracking ${catkin_LIBRARIES})

add_executable(tree_tracking_node src/tree_tracking_node.cpp)
target_link_libraries (tree_tracking_node ${catkin_LIBRARIES} tree_tracking)


#############
## Install ##
#############

