cmake_minimum_required(VERSION 3.10.0)
project(python_subscriber)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_python_setup()

