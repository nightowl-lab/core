cmake_minimum_required(VERSION 3.14)
project(baidu_joy_controller)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(baidu_joy_controller_node SHARED
  src/baidu_joy_controller_node.cpp
)

rclcpp_components_register_node(baidu_joy_controller_node
  PLUGIN "baidu_joy_controller::BaiduJoyControllerNode"
  EXECUTABLE baidu_joy_controller
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)