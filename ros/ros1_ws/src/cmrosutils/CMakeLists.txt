cmake_minimum_required(VERSION 2.8.3)
project(cmrosutils)

## Add support for C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()


## CarMaker specific
# - Definitions see <CMInstDir>/include/Makedefs.*

# Set GET_CM_VERSION_FROM_CMD_LINE_OR_CACHE to OFF,
# to prevent overriding the CarMaker Version from outside
# (i.e., from command line, CM Makefile or cached value)
set(GET_CM_VERSION_FROM_CMD_LINE_OR_CACHE OFF) # changed by fetty

if(NOT ${GET_CM_VERSION_FROM_CMD_LINE_OR_CACHE})
  set(CARMAKER_VER 11.0.1)
  set(CARMAKER_DIR $ENV{IPGHOME}/carmaker/linux64-${CARMAKER_VER})
else()
  set(CARMAKER_VER
      CACHE STRING "CarMaker Version, e.g. 9.0.1")

  set(CARMAKER_DIR $ENV{IPGHOME}/carmaker/linux64-${CARMAKER_VER}
      CACHE STRING "CarMaker installation directory")
endif()

set(CARMAKER_INC_DIR ${CARMAKER_DIR}/include)
set(CMAKE_POSITION_INDEPENDENT_CODE_ON)

if(NOT EXISTS ${CARMAKER_INC_DIR})
  if(NOT DEFINED ${CARMAKER_VER})
    message(FATAL_ERROR "Please set desired CarMaker Version in $CARMAKER_VER")
  else()
    message(FATAL_ERROR "'${CARMAKER_INC_DIR}' does not exist!")
  endif()
endif()

add_definitions(-DCARMAKER_VER=${CARMAKER_VER})
add_definitions(-DROS_DISTRO=$ENV{ROS_DISTRO})


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CMRemoteControl.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${CARMAKER_INC_DIR}
  LIBRARIES cmrosif
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CARMAKER_INC_DIR}
)

## Declare a C++ library

find_library(CMJob_LIB NAMES "CMJob" PATHS "lib")

add_library(cmrosif 
  src/cmrosif.cpp
)

target_link_libraries(cmrosif 
  ${catkin_LIBRARIES}
  ${CMJob_LIB}
)

#############
## Install ##
#############

message(${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
