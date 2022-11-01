cmake_minimum_required(VERSION 3.14)
project(nightowl_description)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Build xacro files
file(GLOB xacro_files
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/**/*.xacro
)
foreach(xacro_file ${xacro_files})
    string(REGEX MATCH "(.*)[.]xacro$" unused ${xacro_file})
    set(output_filename ${CMAKE_MATCH_1})
    xacro_add_xacro_file(${CMAKE_CURRENT_SOURCE_DIR}/${xacro_file} ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${output_filename})
    list(APPEND xacro_build_files ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${output_filename})
endforeach(xacro_file)

add_custom_target(urdf_build_files ALL DEPENDS ${xacro_build_files})

ament_auto_package(
  INSTALL_TO_SHARE
    meshes
    urdf
)