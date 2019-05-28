cmake_minimum_required(VERSION 3.2)
project(container_finder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  image_transport
  roscpp
  tf
  sensor_msgs
  std_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(hsv_calib_node src/hsv_calibration.cpp)
target_link_libraries(hsv_calib_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(laser_node src/laser_node.cpp)
target_link_libraries(laser_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(algorithm_node src/algorithm_node.cpp)
target_link_libraries(algorithm_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
