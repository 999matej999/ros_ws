cmake_minimum_required(VERSION 3.10.0)
project(cpp_publisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cpp_publisher src/main.cpp)
target_link_libraries(cpp_publisher ${catkin_LIBRARIES})

