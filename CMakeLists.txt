cmake_minimum_required(VERSION 3.0.2)
project(hri_optitrack)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  shape_msgs
  moveit_msgs
  message_generation  # Add message_generation here
)

## Declare ROS messages and services
add_message_files(
  FILES
  CreateObject.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs shape_msgs moveit_msgs std_msgs message_runtime  # Add message_runtime here
)

## Build the nodes
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/optitrack_collector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
