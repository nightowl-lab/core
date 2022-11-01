cmake_minimum_required(VERSION 3.8)
project(starneto_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpfpd.msg"
  "msg/Gpgga.msg"
  "msg/Gtimu.msg"
  "msg/Pos320Nav.msg"
  DEPENDENCIES 
    builtin_interfaces 
    std_msgs
)

ament_package()
