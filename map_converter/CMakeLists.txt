cmake_minimum_required(VERSION 3.8)
project(map_converter)

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

ament_auto_add_executable(${PROJECT_NAME}
  src/map_converter.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
)

ament_auto_package()
