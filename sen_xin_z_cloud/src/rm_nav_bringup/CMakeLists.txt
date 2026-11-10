cmake_minimum_required(VERSION 3.8)
project(rm_nav_bringup)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS scripts/fake_vel_transform.py scripts/zlio_nav_adapter.py
  scripts/export_mapping_map.py scripts/read_passage_ranges.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  map
  PCD
  urdf
)
