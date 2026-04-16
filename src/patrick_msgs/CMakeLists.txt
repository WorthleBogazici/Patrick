cmake_minimum_required(VERSION 3.8)
project(patrick_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BumperEvent.msg"
  "msg/CliffEvent.msg"
  "msg/WheelDropEvent.msg"
  "msg/MotorPower.msg"
  "msg/Led.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "srv/GetLinkPose.srv"
  "srv/SetLinkPose.srv"
  "action/ManipulationGoal.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

ament_package()
