cmake_minimum_required(VERSION 3.8)
project(sanchi_amov)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/sanchi_amov.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "imu_driver::ImuDriverNode"
  EXECUTABLE imu_driver
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

