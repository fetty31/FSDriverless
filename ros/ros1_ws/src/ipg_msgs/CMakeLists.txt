cmake_minimum_required(VERSION 3.0.2)
project(ipg_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BatteryStatus.msg 
  CarCommands.msg 
  CarOdometry.msg 
  CarState.msg 
  CarVelocity.msg 
  Cone.msg 
  ConeArray.msg
  DynEstimate.msg 
  KinEstimate.msg 
  GeneralSensor.msg 
  LapInfo.msg 
  LapInfoArray.msg 
  MotorInput.msg 
  MotorOutput.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})