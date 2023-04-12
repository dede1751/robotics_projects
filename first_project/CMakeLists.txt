cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation tf)

add_service_files(FILES ResetOdom.srv)
add_message_files(FILES Odom.msg)
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_node src/pub.cpp)
add_dependencies(odom_node custom_messages_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_node ${catkin_LIBRARIES})


#add_executable(sub src/sub.cpp)
#target_link_libraries(sub ${catkin_LIBRARIES})

