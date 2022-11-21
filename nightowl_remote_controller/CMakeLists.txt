cmake_minimum_required(VERSION 3.14)
project(nightowl_remote_controller)

find_package(ament_cmake_auto REQUIRED)

include(ExternalProject)
ExternalProject_Add(libdatachannel-0.17.11
  PREFIX libdatachannel-0.17.11
  GIT_REPOSITORY https://github.com/paullouisageneau/libdatachannel.git
  GIT_TAG v0.17.11
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/libdatachannel/install
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/libdatachannel/install"
  SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/libdatachannel/src"
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/libdatachannel/install/include)

ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(Libavcodec REQUIRED libavcodec)
pkg_check_modules(Libavutil REQUIRED libavutil)
pkg_check_modules(Libswscale REQUIRED libswscale)

ament_auto_add_library(nightowl_remote_controller_node SHARED
  src/webrtc.cpp
  src/h264_encoder.cpp
  src/tool.cpp
  src/nightowl_remote_controller_node.cpp
)

add_dependencies(nightowl_remote_controller_node libdatachannel-0.17.11)
target_link_libraries(
  nightowl_remote_controller_node
  ${Libavcodec_LIBRARIES}
  ${Libavutil_LIBRARIES}
  ${Libswscale_LIBRARIES}
  ${CMAKE_CURRENT_BINARY_DIR}/libdatachannel/install/lib/libdatachannel.so
)

rclcpp_components_register_node(nightowl_remote_controller_node
  PLUGIN "nightowl_remote_controller::NightOwlRemoteControllerNode"
  EXECUTABLE nightowl_remote_controller
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libdatachannel/install/include DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)