cmake_minimum_required(VERSION 3.8)
project(pca9685_interfaces)

find_package(rosidl_default_generators REQUIRED)

# Generate Python/C++ code for the service -------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMotor.srv"
  DEPENDENCIES builtin_interfaces std_msgs   # list any msgs you use
)

# Nothing to install manually; rosidl + ament do all the work
ament_package()
