#!/usr/bin/env python3
"""
Complete script to create the ROS2 servo controller package
"""

import os
import sys

def create_directory_structure():
    """Create all necessary directories"""
    directories = [
        "servo_controller_py/servo_controller_py",
        "servo_controller_py/launch",
        "servo_controller_py/config",
        "servo_controller_py/test",
        "servo_controller_py/resource",
        "servo_interfaces/msg",
        "servo_interfaces/srv"
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"Created directory: {directory}")
    
    # Create empty files
    open("servo_controller_py/servo_controller_py/__init__.py", 'a').close()
    open("servo_controller_py/resource/servo_controller_py", 'a').close()

def create_servo_controller_node():
    """Create the main servo controller node"""
    content = '''#!/usr/bin/env python3
"""
ROS2 node for controlling pan-tilt servos on Raspberry Pi
"""

import rclpy
from rclpy.node import Node
from servo_interfaces.msg import ServoPosition, ServoState
from servo_interfaces.srv import SetServoPosition, SetServoSpeed
import RPi.GPIO as GPIO
import time
import threading


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # Declare parameters
        self.declare_parameter('pan_pin', 13)
        self.declare_parameter('tilt_pin', 12)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('min_angle', 0)
        self.declare_parameter('max_angle', 270)
        self.declare_parameter('default_speed', 50)  # degrees per second
        
        # Get parameters
        self.pan_pin = self.get_parameter('pan_pin').value
        self.tilt_pin = self.get_parameter('tilt_pin').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.default_speed = self.get_parameter('default_speed').value
        
        # Current servo states
        self.current_pan = 135.0  # Center position
        self.current_tilt = 135.0  # Center position
        self.pan_speed = self.default_speed
        self.tilt_speed = self.default_speed
        
        # Movement flags
        self.pan_moving = False
        self.tilt_moving = False
        
        # Initialize GPIO
        self.init_gpio()
        
        # Create publishers
        self.state_publisher = self.create_publisher(ServoState, 'servo_state', 10)
        
        # Create subscribers
        self.position_subscriber = self.create_subscription(
            ServoPosition,
            'servo_position_cmd',
            self.position_callback,
            10
        )
        
        # Create services
        self.set_position_service = self.create_service(
            SetServoPosition,
            'set_servo_position',
            self.set_position_callback
        )
        
        self.set_speed_service = self.create_service(
            SetServoSpeed,
            'set_servo_speed',
            self.set_speed_callback
        )
        
        # Create timer for publishing state
        self.state_timer = self.create_timer(0.1, self.publish_state)
        
        # Move to initial position
        self.move_servo_immediate(self.pan_pwm, self.current_pan)
        self.move_servo_immediate(self.tilt_pwm, self.current_tilt)
        
        self.get_logger().info(f'Servo controller initialized - Pan: GPIO{self.pan_pin}, Tilt: GPIO{self.tilt_pin}')
    
    def init_gpio(self):
        """Initialize GPIO and PWM"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        
        self.pan_pwm = GPIO.PWM(self.pan_pin, self.pwm_frequency)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, self.pwm_frequency)
        
        self.pan_pwm.start(0)
        self.tilt_pwm.start(0)
        
        time.sleep(0.5)
    
    def angle_to_duty_cycle(self, angle):
        """Convert angle to duty cycle percentage"""
        # Map angle (0-270) to duty cycle (2.5-12.5)
        duty_cycle = 2.5 + (angle / 270.0) * 10.0
        return duty_cycle
    
    def move_servo_immediate(self, servo_pwm, angle):
        """Move servo immediately to specified angle"""
        if self.min_angle <= angle <= self.max_angle:
            duty_cycle = self.angle_to_duty_cycle(angle)
            servo_pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.02)  # Small delay for signal
            servo_pwm.ChangeDutyCycle(0)  # Stop signal to reduce jitter
            return True
        return False
    
    def move_servo_smooth(self, servo_pwm, start_angle, target_angle, speed, is_pan=True):
        """Move servo smoothly from start to target angle at specified speed"""
        if is_pan:
            self.pan_moving = True
        else:
            self.tilt_moving = True
        
        current = start_angle
        step = 1.0 if target_angle > start_angle else -1.0
        delay = 1.0 / speed  # Time between steps
        
        while abs(current - target_angle) > 0.5:
            current += step
            if step > 0:
                current = min(current, target_angle)
            else:
                current = max(current, target_angle)
            
            self.move_servo_immediate(servo_pwm, current)
            
            if is_pan:
                self.current_pan = current
            else:
                self.current_tilt = current
            
            time.sleep(delay)
        
        # Final position
        self.move_servo_immediate(servo_pwm, target_angle)
        
        if is_pan:
            self.current_pan = target_angle
            self.pan_moving = False
        else:
            self.current_tilt = target_angle
            self.tilt_moving = False
    
    def position_callback(self, msg):
        """Handle position command messages"""
        # Start pan movement in thread
        if msg.pan_angle != self.current_pan and not self.pan_moving:
            pan_thread = threading.Thread(
                target=self.move_servo_smooth,
                args=(self.pan_pwm, self.current_pan, msg.pan_angle, self.pan_speed, True)
            )
            pan_thread.daemon = True
            pan_thread.start()
        
        # Start tilt movement in thread
        if msg.tilt_angle != self.current_tilt and not self.tilt_moving:
            tilt_thread = threading.Thread(
                target=self.move_servo_smooth,
                args=(self.tilt_pwm, self.current_tilt, msg.tilt_angle, self.tilt_speed, False)
            )
            tilt_thread.daemon = True
            tilt_thread.start()
    
    def set_position_callback(self, request, response):
        """Handle set position service requests"""
        success = True
        
        # Validate angles
        if not (self.min_angle <= request.pan_angle <= self.max_angle):
            response.success = False
            response.message = f"Pan angle must be between {self.min_angle} and {self.max_angle}"
            return response
        
        if not (self.min_angle <= request.tilt_angle <= self.max_angle):
            response.success = False
            response.message = f"Tilt angle must be between {self.min_angle} and {self.max_angle}"
            return response
        
        # Create and publish position message
        pos_msg = ServoPosition()
        pos_msg.pan_angle = request.pan_angle
        pos_msg.tilt_angle = request.tilt_angle
        self.position_callback(pos_msg)
        
        response.success = True
        response.message = f"Moving to Pan: {request.pan_angle}°, Tilt: {request.tilt_angle}°"
        return response
    
    def set_speed_callback(self, request, response):
        """Handle set speed service requests"""
        if request.pan_speed > 0:
            self.pan_speed = request.pan_speed
        if request.tilt_speed > 0:
            self.tilt_speed = request.tilt_speed
        
        response.success = True
        response.message = f"Speed set - Pan: {self.pan_speed}°/s, Tilt: {self.tilt_speed}°/s"
        return response
    
    def publish_state(self):
        """Publish current servo state"""
        state_msg = ServoState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.pan_angle = self.current_pan
        state_msg.tilt_angle = self.current_tilt
        state_msg.pan_moving = self.pan_moving
        state_msg.tilt_moving = self.tilt_moving
        self.state_publisher.publish(state_msg)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        servo_controller = ServoControllerNode()
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.cleanup()
        servo_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    
    with open("servo_controller_py/servo_controller_py/servo_controller_node.py", "w") as f:
        f.write(content)
    print("Created servo_controller_node.py")

def create_launch_file():
    """Create the launch file"""
    content = '''from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('servo_controller_py')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'servo_config.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to servo configuration file'
        ),
        
        # Servo controller node
        Node(
            package='servo_controller_py',
            executable='servo_controller_node',
            name='servo_controller',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            respawn=True,
            respawn_delay=2.0
        )
    ])
'''
    
    with open("servo_controller_py/launch/servo_controller_launch.py", "w") as f:
        f.write(content)
    print("Created servo_controller_launch.py")

def create_package_files():
    """Create all package configuration files"""
    files = {
        "servo_controller_py/package.xml": '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>servo_controller_py</name>
  <version>0.0.1</version>
  <description>ROS2 package for controlling pan-tilt servos on Raspberry Pi</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>servo_interfaces</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
''',

        "servo_controller_py/setup.py": '''from setuptools import setup
import os
from glob import glob

package_name = 'servo_controller_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for controlling pan-tilt servos on Raspberry Pi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller_node = servo_controller_py.servo_controller_node:main',
        ],
    },
)
''',

        "servo_controller_py/setup.cfg": '''[develop]
script_dir=$base/lib/servo_controller_py
[install]
install_scripts=$base/lib/servo_controller_py
''',

        "servo_controller_py/config/servo_config.yaml": '''servo_controller:
  ros__parameters:
    # GPIO pin numbers (BCM numbering)
    pan_pin: 13    # GPIO 13 for pan servo
    tilt_pin: 12   # GPIO 12 for tilt servo
    
    # PWM settings
    pwm_frequency: 50  # Hz
    
    # Angle limits
    min_angle: 0     # degrees
    max_angle: 270   # degrees
    
    # Movement speed
    default_speed: 50  # degrees per second
''',

        "servo_interfaces/package.xml": '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>servo_interfaces</name>
  <version>0.0.1</version>
  <description>Custom messages and services for servo control</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
''',

        "servo_interfaces/CMakeLists.txt": '''cmake_minimum_required(VERSION 3.8)
project(servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoPosition.msg"
  "msg/ServoState.msg"
  "srv/SetServoPosition.srv"
  "srv/SetServoSpeed.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
''',

        "servo_interfaces/msg/ServoPosition.msg": '''# Message for commanding servo positions
float32 pan_angle   # Pan angle in degrees (0-270)
float32 tilt_angle  # Tilt angle in degrees (0-270)
''',

        "servo_interfaces/msg/ServoState.msg": '''# Message for current servo state
std_msgs/Header header
float32 pan_angle    # Current pan angle in degrees
float32 tilt_angle   # Current tilt angle in degrees
bool pan_moving      # True if pan servo is currently moving
bool tilt_moving     # True if tilt servo is currently moving
''',

        "servo_interfaces/srv/SetServoPosition.srv": '''# Service for setting servo positions
float32 pan_angle   # Target pan angle in degrees (0-270)
float32 tilt_angle  # Target tilt angle in degrees (0-270)
---
bool success        # True if command was accepted
string message      # Response message
''',

        "servo_interfaces/srv/SetServoSpeed.srv": '''# Service for setting servo movement speeds
float32 pan_speed   # Pan speed in degrees per second (0 = keep current)
float32 tilt_speed  # Tilt speed in degrees per second (0 = keep current)
---
bool success        # True if command was accepted
string message      # Response message
'''
    }
    
    for filepath, content in files.items():
        with open(filepath, "w") as f:
            f.write(content)
        print(f"Created {filepath}")

def main():
    print("Creating ROS2 Servo Controller Package...")
    print("=" * 50)
    
    # Create directory structure
    create_directory_structure()
    
    # Create all files
    create_package_files()
    create_servo_controller_node()
    create_launch_file()
    
    print("\n" + "=" * 50)
    print("✅ Package creation complete!")
    print("\nNext steps:")
    print("1. Update email and name in package.xml and setup.py files")
    print("2. Commit to git:")
    print("   git add servo_controller_py/ servo_interfaces/")
    print("   git commit -m 'Add servo controller ROS2 packages'")
    print("   git push")
    print("\n3. On Raspberry Pi:")
    print("   cd ~/ros2_ws")
    print("   colcon build --packages-select servo_interfaces servo_controller_py")
    print("   source install/setup.bash")
    print("   ros2 launch servo_controller_py servo_controller_launch.py")

if __name__ == "__main__":
    main()
