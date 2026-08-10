cmake_minimum_required(VERSION 3.0.2)
project(my_project_1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# This repository contains plain Python scripts. To run them as ROS nodes you can:
# 1) Move them into a ROS package "scripts/" directory and mark executable, or
# 2) Use rosrun with the path to the script if the package is sourced and installed.

