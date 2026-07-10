cmake_minimum_required(VERSION 3.8)
project(zsl_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
