cmake_minimum_required(VERSION 3.8)
project(lidar_decoder)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/lidar_decoder_node.cpp
  src/lidar_rawdata.cpp
  src/pointcloudXYZIRADT.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "lidar_decoder::LidarDecoder"
  EXECUTABLE lidar_decoder
)


install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)