cmake_minimum_required(VERSION 3.14)
project(yhs_can_control)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(yhs_can_control_node SHARED
  src/diagnostic_publisher.cpp
  src/yhs_can_control_node.cpp
)

rclcpp_components_register_node(yhs_can_control_node
  PLUGIN "yhs_can_control::YHSCANControlNode"
  EXECUTABLE yhs_can_control
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
