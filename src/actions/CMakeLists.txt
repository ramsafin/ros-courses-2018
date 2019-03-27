cmake_minimum_required(VERSION 2.8.3)
project(actions)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  actionlib
  roscpp
  rospy
)

find_package(Boost REQUIRED)

add_action_files(DIRECTORY action FILES Factorial.action Averaging.action)
# add_action_files(DIRECTORY action FILES Averaging.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(factorial_client src/factorial/factorial_client.cpp)
add_executable(factorial_server src/factorial/factorial_server.cpp)

add_executable(averaging_client src/averaging/averaging_client.cpp)
add_executable(averaging_server src/averaging/averaging_server.cpp)

target_link_libraries(factorial_client ${catkin_LIBRARIES})
target_link_libraries(factorial_server ${catkin_LIBRARIES})

target_link_libraries(averaging_client ${catkin_LIBRARIES})
target_link_libraries(averaging_server ${catkin_LIBRARIES})

# build action headers before using them
add_dependencies(factorial_client ${actions_EXPORTED_TARGETS})
add_dependencies(factorial_server ${actions_EXPORTED_TARGETS})

add_dependencies(averaging_client ${actions_EXPORTED_TARGETS})
add_dependencies(averaging_server ${actions_EXPORTED_TARGETS})

