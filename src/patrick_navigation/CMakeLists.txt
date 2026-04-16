cmake_minimum_required(VERSION 3.8)
project(patrick_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(patrick_msgs REQUIRED)

# Navigation lifecycle node
add_executable(navigation_node src/navigation_node.cpp)
target_include_directories(navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(navigation_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Navigation functionality check
add_executable(nav_functionality_check src/nav_functionality_check.cpp)
ament_target_dependencies(nav_functionality_check
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS navigation_node nav_functionality_check
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
