cmake_minimum_required(VERSION 3.8)
project(starneto_configure)

link_directories(/lib/x86_64-linux-gnu/libreadline.so)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/starneto_configure_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "starneto_configure::StarnetoConfigureNode"
  EXECUTABLE starneto_configure_node
)

target_link_libraries(${PROJECT_NAME} -lreadline ${LD_LIBRARY_PATH})

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
