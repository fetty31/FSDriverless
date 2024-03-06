cmake_minimum_required(VERSION 3.5)
project(cmrosutils)

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
  set(CARMAKER_VER 9.0.1)
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


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CMRemoteControl.srv"
  DEPENDENCIES
    builtin_interfaces
 )

## Declare a C++ library

find_library(CMJob_LIB NAMES "CMJob" PATHS "lib")

add_library(cmrosif SHARED
  src/cmrosif.cpp
)

ament_target_dependencies(cmrosif
  rclcpp
 )

rosidl_target_interfaces(cmrosif
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
 )

target_link_libraries(cmrosif
  ${CMJob_LIB}
)

target_include_directories(cmrosif
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CARMAKER_INC_DIR}>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:${CARMAKER_INC_DIR}>
 )

if(COMMAND ament_export_targets)
  ament_export_targets(cmrosifTargets HAS_LIBRARY_TARGET)
else()
  ament_export_libraries(cmrosif)
  ament_export_include_directories(include ${CARMAKER_INC_DIR})
  ament_export_dependencies(rclcpp)
  ament_export_dependencies(rosidl_default_runtime)
endif()

install(
  DIRECTORY include/
  DESTINATION include
 )

install(
  TARGETS cmrosif
  EXPORT cmrosifTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
