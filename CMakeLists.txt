cmake_minimum_required(VERSION 2.8.3)
project(freefloating_test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg)

## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(battery_state src/battery_state.cpp)
target_link_libraries(battery_state ${catkin_LIBRARIES})
