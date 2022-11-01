cmake_minimum_required(VERSION 3.8)
project(lidar_driver)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/lidar_driver_node.cpp
  src/lidar_input.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "lidar_driver::LidarDriver"
  EXECUTABLE lidar_driver
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
