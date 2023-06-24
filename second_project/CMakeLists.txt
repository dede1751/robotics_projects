cmake_minimum_required(VERSION 2.8.3)
project(second_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs tf)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tf_publisher src/tf_publisher.cpp)
add_executable(navigation src/navigation.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})
target_link_libraries(navigation ${catkin_LIBRARIES})
