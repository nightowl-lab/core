cmake_minimum_required(VERSION 3.14)
project(joy2)

find_package(ament_cmake_auto REQUIRED)

include(ExternalProject)
ExternalProject_Add(SDL2-2.0.22
  PREFIX SDL2-2.0.22
  URL https://www.libsdl.org/release/SDL2-2.0.22.zip
  URL_MD5 334ba533f297ba29de89e4528e534cb1
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/SDL2/install
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/SDL2/install"
  SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/SDL2/src"
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/SDL2/install/include/SDL2)

ament_auto_find_build_dependencies()

ament_auto_add_library(joy_node SHARED
  src/joy.cpp
)

add_dependencies(joy_node SDL2-2.0.22)
target_link_libraries(joy_node ${CMAKE_CURRENT_BINARY_DIR}/SDL2/install/lib/libSDL2.so)

rclcpp_components_register_node(joy_node
  PLUGIN "joy::Joy"
  EXECUTABLE joy
)

ament_auto_add_executable(joy_enumerate_devices
	src/joy_enumerate_devices.cpp
)

add_dependencies(joy_enumerate_devices SDL2-2.0.22)
target_link_libraries(joy_enumerate_devices ${CMAKE_CURRENT_BINARY_DIR}/SDL2/install/lib/libSDL2.so)


ament_auto_package(
  INSTALL_TO_SHARE
    launch
)