cmake_minimum_required(VERSION 3.10.0)
project(head_tracking)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
## include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/include)
##link_directories(${CMAKE_BINARY_DIR})

add_executable(head_tracking src/main.cpp)
target_link_libraries(head_tracking ${catkin_LIBRARIES} openhmd)
##target_link_libraries(head_tracking ${catkin_LIBRARIES} PRIVATE openhmd)

##if (UNIX)
##    target_link_libraries(simple PRIVATE m)
##endif()
