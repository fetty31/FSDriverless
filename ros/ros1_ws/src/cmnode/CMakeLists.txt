cmake_minimum_required(VERSION 2.8.3)
project(cmnode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

message(STATUS "CMNode says hi :)")

# Debug?
if(CMAKE_BUILD_TYPE MATCHES Debug)
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
else()
  add_definitions(-DNDEBUG)
endif()

find_package(catkin REQUIRED COMPONENTS
   tf
   tf2
   tf2_ros
   tf2_geometry_msgs
   roscpp
   std_msgs
   ipg_msgs
   cmrosutils
   nav_msgs
   sensor_msgs
   tracker
)

find_library(CM_ROSIF libcmrosif.so PATHS ${CATKIN_PACKAGE_LIB_DESTINATION})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime cmrosutils ipg_msgs tracker tf tf2 tf2_ros tf2_geometry_msgs nav_msgs sensor_msgs
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  ${cmrosutils_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/cmnode.cpp
                            src/utils/params.cpp
)

add_dependencies(${PROJECT_NAME} ${ipg_msgs_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${CM_ROSIF} 
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
