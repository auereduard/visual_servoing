cmake_minimum_required(VERSION 2.8.3)
project(vs_msgs)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  geometry_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  receive_cam_data.msg  
)


add_action_files(DIRECTORY action
  FILES
  start_vs.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)

