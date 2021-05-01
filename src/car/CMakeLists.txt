cmake_minimum_required(VERSION 3.10.0)
project(car)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

find_package(RoboUtils REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(car src/main.cpp)
target_link_libraries(car ${catkin_LIBRARIES} libroboutils)
