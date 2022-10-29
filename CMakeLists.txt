cmake_minimum_required(VERSION 3.5)
project(decanstructor)

## Compile as C++11, supported in ROS Kinetic and newer
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(wxWidgets COMPONENTS
  core
  base
  adv
  REQUIRED
)

ament_auto_find_build_dependencies()

set(wxWidgets_CONFIGURATION mswu)
include(${wxWidgets_USE_FILE})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanEvent.msg"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_auto_add_library(${PROJECT_NAME}_nodes
  src/message_analyzer.cpp
  src/decanstructor_node.cpp
)

get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(${PROJECT_NAME}_nodes
      ${PROJECT_NAME} ${typesupport_impl}
    )
  endforeach()
endforeach()

target_link_libraries(${PROJECT_NAME}_nodes
  ${wxWidgets_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "DecanstructorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
