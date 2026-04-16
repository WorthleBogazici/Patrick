cmake_minimum_required(VERSION 3.8)
project(patrick_main)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  config
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
