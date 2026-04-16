cmake_minimum_required(VERSION 3.8)
project(patrick_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(patrick_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ── Manipulation lifecycle node (high-level arm control) ─────────────────
add_executable(manipulation_node src/manipulation_node.cpp)
target_include_directories(manipulation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(manipulation_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  patrick_msgs
)
target_link_libraries(manipulation_node Eigen3::Eigen)

# ── Arduino hardware interface plugin (ros2_control) ─────────────────────
add_library(arduino_hardware_interface SHARED
  src/arduino_hardware_interface.cpp
)
target_include_directories(arduino_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arduino_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface patrick_manipulation_plugin.xml)

# ── Functionality check node (sequential joint test) ─────────────────────
add_executable(functionality_check src/functionality_check.cpp)
target_include_directories(functionality_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(functionality_check
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  sensor_msgs
)

install(TARGETS manipulation_node functionality_check
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS arduino_hardware_interface
  EXPORT export_arduino_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(arduino_hardware_interface)
ament_export_targets(export_arduino_hardware_interface)

ament_package()
