cmake_minimum_required(VERSION 2.8.3)

project(duckie_lab)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  duckietown_msgs
  std_msgs
  sensor_msgs
)
 
catkin_package()
