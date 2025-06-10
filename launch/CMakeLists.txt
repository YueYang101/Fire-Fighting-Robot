cmake_minimum_required(VERSION 3.5)
project(fire_fighting_robot)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()