cmake_minimum_required(VERSION 3.14)
project(yhs_autoware_interface)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(yhs_autoware_interface_node SHARED
  src/yhs_autoware_interface_node.cpp
)

rclcpp_components_register_node(yhs_autoware_interface_node
  PLUGIN "yhs_autoware_interface::YHSAutowareInterfaceNode"
  EXECUTABLE yhs_autoware_interface
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)