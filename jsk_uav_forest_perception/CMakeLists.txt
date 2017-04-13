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

find_package(OpenCV REQUIRED)
message(WARNING "OPENCV ${OpenCV_VERSION} FOUND")

generate_dynamic_reconfigure_options(
  cfg/ColorRegionDetector.cfg
  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_library(color_region_detection src/color_region_detector.cpp)
target_link_libraries(color_region_detection ${catkin_LIBRARIES})
add_dependencies(color_region_detection ${PROJECT_NAME}_gencfg)

add_library(jsk_laser_scan_filters SHARED src/laser_scan_filters.cpp)
target_link_libraries(jsk_laser_scan_filters ${catkin_LIBRARIES})

add_library(tree_detector src/tree_detector.cpp)
target_link_libraries(tree_detector ${catkin_LIBRARIES})
add_executable(tree_detector_node src/tree_detector_node.cpp)
target_link_libraries (tree_detector_node ${catkin_LIBRARIES} tree_detector)


#############
## Install ##
#############

