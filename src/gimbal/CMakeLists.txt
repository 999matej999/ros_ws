cmake_minimum_required(VERSION 3.10.0)
project(gimbal)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

find_package(RoboUtils REQUIRED)

find_library(wiringPi_LIB wiringPi)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gimbal src/main.cpp)

target_link_libraries(gimbal ${catkin_LIBRARIES} libroboutils ${wiringPi_LIB})
