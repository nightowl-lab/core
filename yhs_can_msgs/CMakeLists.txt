cmake_minimum_required(VERSION 3.5)
project(yhs_can_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryFlagFeedback.msg"
  "msg/BatteryInformationFeedback.msg"
  "msg/CtrlCommand.msg"
  "msg/CtrlFeeback.msg"
  "msg/DiagnosticFeedback.msg"
  "msg/IoCommand.msg"
  "msg/IoFeedback.msg"
  "msg/OdometryFeedback.msg"
  "msg/WheelEncoderFeedback.msg"
  "msg/WheelFeedback.msg"
  "msg/OnlineStatus.msg"
  DEPENDENCIES
    "std_msgs"
    ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
