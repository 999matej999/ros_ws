cmake_minimum_required(VERSION 3.10.0)
project(head_tracking)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(head_tracking src/main.cpp)
target_link_libraries(head_tracking ${catkin_LIBRARIES})


