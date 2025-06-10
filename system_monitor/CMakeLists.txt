cmake_minimum_required(VERSION 3.5)
project(system_monitor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemLoad.msg"
  "msg/ProcessInfo.msg"
  "msg/NetworkStats.msg"
  "msg/DiskUsage.msg"
  "msg/SystemStatus.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  ${PROJECT_NAME}/system_monitor_node.py
  ${PROJECT_NAME}/system_info_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
