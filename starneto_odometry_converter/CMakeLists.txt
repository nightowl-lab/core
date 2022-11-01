cmake_minimum_required(VERSION 3.8)
project(starneto_odometry_converter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
  ${GeographicLib_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/starneto_odometry_converter_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${GeographicLib_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "starneto_odometry_converter::StarnetoOdometryConverterNode"
  EXECUTABLE starneto_odometry_converter
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
