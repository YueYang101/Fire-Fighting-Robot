cmake_minimum_required(VERSION 3.5)
project(fire_fighting_robot_bringup)  # <-- This must match the folder name

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()