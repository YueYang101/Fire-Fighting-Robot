cmake_minimum_required(VERSION 3.8)
project(pca9685_interfaces)

# 1) find the ROS 2 code-generator framework
find_package(rosidl_default_generators REQUIRED)

# 2) find every interface pkg you listed under DEPENDENCIES
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# 3) now generate your service code
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMotor.srv"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

# 4) boilerplate
ament_package()
