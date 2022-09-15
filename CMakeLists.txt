cmake_minimum_required(VERSION 2.8.3)
project(decanstructor)
aux_source_directory(. SRC_LIST)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  can_msgs
  std_msgs
  message_generation
)

set(wxWidgets_CONFIGURATION mswu)

find_package(wxWidgets COMPONENTS
  core
  base
  adv
  REQUIRED
)

add_message_files(
  FILES
  CanEvent.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  can_msgs
  std_msgs
  message_runtime
  DEPENDS wxWidgets
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

include("${wxWidgets_USE_FILE}")

add_executable(${PROJECT_NAME}_node
  src/message_analyzer.cpp
  src/decanstructor_node.cpp
  ${SRC_LIST}
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${wxWidgets_LIBRARIES}
  ${catkin_LIBRARIES}
)
