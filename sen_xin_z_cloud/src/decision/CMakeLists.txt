cmake_minimum_required(VERSION 3.8)
project(decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(rm_decision_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(serial REQUIRED)
find_package(rm_serial_protocol REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(trans_node src/trans_node.cpp)
add_executable(trans_node2 src/trans_node2.cpp)
target_compile_features(trans_node2 PUBLIC cxx_std_17)
ament_target_dependencies(trans_node2 rclcpp rclcpp_action diagnostic_msgs std_msgs
  base_interfaces_demo rm_decision_interfaces geometry_msgs nav_msgs tf2_ros serial rm_serial_protocol)
add_executable(trans_node4 src/trans_node4.cpp src/crc8.cpp)
add_executable(referee_map_tf src/path_vision.cpp)
add_executable(rosout_node src/rosout.cpp)
add_executable(rosout_trans src/rosout_trans.cpp)


target_include_directories(trans_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trans_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(trans_node4 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trans_node4 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  trans_node
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
  "rm_decision_interfaces"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "serial"
)

ament_target_dependencies(
  referee_map_tf
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
)

ament_target_dependencies(
  trans_node4
  "rclcpp"
  "rcl_interfaces"
  "std_msgs"
  "base_interfaces_demo"
  "rm_decision_interfaces"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "serial"
)

ament_target_dependencies(
  rosout_node
  "rclcpp"
  "std_msgs"
  "rcl_interfaces"
)

ament_target_dependencies(
  rosout_trans
  "rclcpp"
  "rcl_interfaces"
)

install(TARGETS 
  trans_node2
  trans_node
  trans_node4
  referee_map_tf
  rosout_node
  rosout_trans
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(path_sampling_test test/path_sampling_test.cpp)
  target_include_directories(path_sampling_test PRIVATE include)
  target_compile_features(path_sampling_test PRIVATE cxx_std_17)
  add_test(NAME path_sampling_test COMMAND path_sampling_test)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
