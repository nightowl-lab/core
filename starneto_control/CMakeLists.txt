cmake_minimum_required(VERSION 3.8)
project(starneto_control)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/starneto_control_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "starneto_control::StarnetoControlNode"
  EXECUTABLE starneto_control
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
