cmake_minimum_required(VERSION 3.0.2)
project(tracker)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

message(STATUS "TRACKER says hi :)")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ipg_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  message_generation
)

add_service_files(
  FILES
    Init.srv
)

generate_messages(
   DEPENDENCIES 
    std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs ipg_msgs visualization_msgs tf2 tf2_ros message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/accumulator.cpp 
                               src/main.cpp
                               )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)