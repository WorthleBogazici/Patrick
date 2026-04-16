cmake_minimum_required(VERSION 3.8)
project(patrick_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(patrick_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

# ── Object frame publisher (C++) ────────────────────────────────────────────
add_executable(object_frame_publisher src/object_frame_publisher.cpp)
target_include_directories(object_frame_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(object_frame_publisher ${PCL_LIBRARIES})
ament_target_dependencies(object_frame_publisher
  rclcpp std_msgs sensor_msgs geometry_msgs
  cv_bridge tf2_ros tf2_geometry_msgs patrick_msgs
)

# ── Object marker publisher (C++) ───────────────────────────────────────────
add_executable(object_marker_publisher src/object_marker_publisher.cpp)
ament_target_dependencies(object_marker_publisher
  rclcpp visualization_msgs
)

install(TARGETS
  object_frame_publisher
  object_marker_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python nodes ────────────────────────────────────────────────────────────
install(PROGRAMS
  scripts/fastsam_color_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  segmentation
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
