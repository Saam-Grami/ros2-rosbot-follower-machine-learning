cmake_minimum_required(VERSION 3.8)
project(mtre4820_rosbot)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
