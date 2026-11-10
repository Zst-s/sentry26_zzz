cmake_minimum_required(VERSION 3.8)
project(sentry_passage_fusion_test)
find_package(ament_cmake REQUIRED)

# 只安装结构记忆及配对静态图导出所需模块。
set(MEMORY_MODULES
  continuous_passage_memory global_passage_memory grid_fingerprint
  live_memory_shadow localization_health mapping_terrain paired_grid
  passage_memory_v2 passage_record_io passage_region passage_ranges rigid_transform
  roof_association structure_observation wall_bounded_region wall_observation)
foreach(module IN LISTS MEMORY_MODULES)
  install(FILES scripts/${module}.py DESTINATION lib/${PROJECT_NAME})
endforeach()
install(PROGRAMS scripts/live_memory_runtime.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_test(NAME memory_continuous
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_continuous_memory.py)
  add_test(NAME memory_reassociation
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_memory_reassociation.py)
  add_test(NAME wall_tf_retry
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_wall_tf_retry.py)
  add_test(NAME passage_ranges_export
    COMMAND /usr/bin/python3 -B ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_passage_ranges.py)
endif()
ament_package()
