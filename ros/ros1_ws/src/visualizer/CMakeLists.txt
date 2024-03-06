cmake_minimum_required(VERSION 3.0.2)
project(visualizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ipg_msgs
  roscpp
  tf
)

catkin_package(
  # LIBRARIES $(PROJECT_NAME)
  CATKIN_DEPENDS roscpp tf ipg_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})