cmake_minimum_required(VERSION 3.5)
project(carmaker_rsds_client)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## CarMaker specific
# - Definitions see <CMInstDir>/include/Makedefs.*

# Set GET_CM_VERSION_FROM_CMD_LINE_OR_CACHE to OFF,
# to prevent overriding the CarMaker Version from outside
# (i.e., from command line, CM Makefile or cached value)
set(GET_CM_VERSION_FROM_CMD_LINE_OR_CACHE ON)

if(NOT ${GET_CM_VERSION_FROM_CMD_LINE_OR_CACHE})
  set(CARMAKER_VER 11.0.1)
  set(CARMAKER_DIR $ENV{IPGHOME}/carmaker/linux64-${CARMAKER_VER})
else()
  set(CARMAKER_VER
      CACHE STRING "CarMaker Version, e.g. 11.0.1")

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cmrosutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_msgs REQUIRED)
find_package(image_transport REQUIRED)
# find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${CARMAKER_INC_DIR})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/carmaker_rsds_client_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  cmrosutils
  sensor_msgs
  camera_msgs
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

install(TARGETS
   ${PROJECT_NAME}_node
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
