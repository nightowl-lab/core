cmake_minimum_required(VERSION 3.8)
project(smarter_eye_camera)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(smarter_eye_camera_node SHARED
  src/smarter_eye_camera_node.cpp
)

# 链接SDK
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/smarter_eye_sdk/include)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  target_link_libraries(smarter_eye_camera_node ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/smarter_eye_sdk/bin/aarch64/libStereoCamera.so)
  target_link_libraries(smarter_eye_camera_node ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/smarter_eye_sdk/bin/aarch64/libImageUtils.so)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/smarter_eye_sdk/bin/aarch64/
    DESTINATION lib
  )
else()
  target_link_libraries(smarter_eye_camera_node ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/smarter_eye_sdk/bin/amd64/libStereoCamera.so)
  target_link_libraries(smarter_eye_camera_node ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/smarter_eye_sdk/bin/amd64/libImageUtils.so)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/smarter_eye_sdk/bin/amd64/
    DESTINATION lib
  )
endif()

rclcpp_components_register_node(smarter_eye_camera_node
  PLUGIN "smarter_eye_camera::SmarterEyeCameraNode"
  EXECUTABLE smarter_eye_camera
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
