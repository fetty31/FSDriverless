cmake_minimum_required(VERSION 2.8.3)
project(camera_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES
  Camera_Detection.msg
  Camera_DetectionArray.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
