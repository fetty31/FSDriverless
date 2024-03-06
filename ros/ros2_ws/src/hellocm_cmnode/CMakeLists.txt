cmake_minimum_required(VERSION 3.5)
project(hellocm_cmnode)

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

###   CarMaker ROS Node specific   ###
set(CMNODE_BUILD_NAME CMNode_ROS2_HelloCM)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmrosutils REQUIRED)
find_package(hellocm_msgs REQUIRED)
find_package(vehiclecontrol_msgs REQUIRED)
find_package(camera_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

## Declare a C++ library
add_library(${CMNODE_BUILD_NAME} SHARED 
  src/${CMNODE_BUILD_NAME}.cpp
)

ament_target_dependencies(${CMNODE_BUILD_NAME}
    rclcpp
    std_msgs
    cmrosutils
    hellocm_msgs
    vehiclecontrol_msgs
    camera_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    sensor_msgs
 )

install(TARGETS
  ${CMNODE_BUILD_NAME}
  DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(${CMNODE_BUILD_NAME})

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
