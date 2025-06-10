#!/usr/bin/env python3
"""
Complete script to create the ROS2 linear actuator controller package
Run this script to generate all necessary files and directories
"""

import os
import sys

def create_directory_structure():
    """Create all necessary directories"""
    directories = [
        "actuator_controller/actuator_controller",
        "actuator_controller/launch",
        "actuator_controller/config",
        "actuator_controller/resource",
        "actuator_interfaces/srv"
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"Created directory: {directory}")
    
    # Create empty files
    open("actuator_controller/actuator_controller/__init__.py", 'a').close()
    open("actuator_controller/resource/actuator_controller", 'a').close()

def create_actuator_node():
    """Create the main actuator controller node"""
    content = '''#!/usr/bin/env python3
"""
ROS2 Node for Linear Actuator Control using PCA9685
Controls a DC linear actuator through channels 4-5 on PCA9685
"""

import rclpy
from rclpy.node import Node
from actuator_interfaces.srv import SetActuator
import time
import threading

try:
    import board
    import busio
    import adafruit_pca9685
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("Warning: PCA9685 hardware not available - running in simulation mode")
    
    # Dummy classes for testing
    class DummyChannel:
        def __init__(self):
            self.duty_cycle = 0
    
    class DummyPCA:
        def __init__(self):
            self.channels = [DummyChannel() for _ in range(16)]
            self.frequency = 1000


class LinearActuatorNode(Node):
    def __init__(self):
        super().__init__('linear_actuator_node')
        
        # Declare parameters
        self.declare_parameter('in1_channel', 4)
        self.declare_parameter('in2_channel', 5)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('max_extend_time', 8.0)
        
        # Get parameters
        self.in1_channel = self.get_parameter('in1_channel').value
        self.in2_channel = self.get_parameter('in2_channel').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.max_extend_time = self.get_parameter('max_extend_time').value
        
        # Initialize PCA9685
        if HARDWARE_AVAILABLE:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = adafruit_pca9685.PCA9685(self.i2c)
            self.pca.frequency = self.pwm_frequency
        else:
            self.get_logger().warn('Hardware not available - running in simulation mode')
            self.pca = DummyPCA()
        
        # State tracking
        self.current_action = "stop"
        self.current_speed = 0
        self.extend_start_time = None
        self.is_extending = False
        self.movement_thread = None
        
        # Create service
        self.srv = self.create_service(
            SetActuator,
            'set_actuator',
            self.handle_actuator_request
        )
        
        # Create timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check_callback)
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(
            f'Linear actuator node initialized - IN1: ch{self.in1_channel}, '
            f'IN2: ch{self.in2_channel}, Max extend: {self.max_extend_time}s'
        )
        
        # Initialize to stopped state
        self.stop()
    
    def handle_actuator_request(self, request, response):
        """Handle actuator control service requests"""
        try:
            action = request.action.lower()
            speed = max(0, min(100, request.speed))
            duration = max(0, request.duration)
            
            self.get_logger().info(
                f'Received command: action={action}, speed={speed}, duration={duration}'
            )
            
            # Stop any existing movement
            if self.movement_thread and self.movement_thread.is_alive():
                self.stop()
                time.sleep(0.1)
            
            # Execute action
            if action == 'extend':
                self.extend(speed)
                if duration > 0:
                    # Start timed operation in thread
                    self.movement_thread = threading.Thread(
                        target=self._timed_operation,
                        args=(duration,)
                    )
                    self.movement_thread.daemon = True
                    self.movement_thread.start()
                    
            elif action == 'retract':
                self.retract(speed)
                if duration > 0:
                    self.movement_thread = threading.Thread(
                        target=self._timed_operation,
                        args=(duration,)
                    )
                    self.movement_thread.daemon = True
                    self.movement_thread.start()
                    
            elif action == 'stop':
                self.stop()
            else:
                response.success = False
                response.message = f"Unknown action: {action}. Use 'extend', 'retract', or 'stop'"
                return response
            
            response.success = True
            response.message = f"Actuator {action} at {speed}% speed"
            if duration > 0:
                response.message += f" for {duration} seconds"
            
        except Exception as e:
            self.get_logger().error(f'Error handling request: {str(e)}')
            response.success = False
            response.message = str(e)
        
        return response
    
    def _timed_operation(self, duration):
        """Run operation for specified duration then stop"""
        time.sleep(duration)
        self.stop()
        self.get_logger().info(f'Timed operation completed after {duration} seconds')
    
    def extend(self, speed=100):
        """Extend the linear actuator"""
        duty_cycle = int((speed / 100.0) * 65535)
        
        # Set PWM values
        self.pca.channels[self.in1_channel].duty_cycle = duty_cycle
        self.pca.channels[self.in2_channel].duty_cycle = 0
        
        # Update state
        self.current_action = "extend"
        self.current_speed = speed
        self.is_extending = True
        self.extend_start_time = time.time()
        
        self.get_logger().info(f'Extending at {speed}% (duty cycle: {duty_cycle})')
    
    def retract(self, speed=100):
        """Retract the linear actuator"""
        duty_cycle = int((speed / 100.0) * 65535)
        
        # Set PWM values
        self.pca.channels[self.in1_channel].duty_cycle = 0
        self.pca.channels[self.in2_channel].duty_cycle = duty_cycle
        
        # Update state
        self.current_action = "retract"
        self.current_speed = speed
        self.is_extending = False
        self.extend_start_time = None
        
        self.get_logger().info(f'Retracting at {speed}% (duty cycle: {duty_cycle})')
    
    def stop(self):
        """Stop the linear actuator"""
        # Set both channels to 0
        self.pca.channels[self.in1_channel].duty_cycle = 0
        self.pca.channels[self.in2_channel].duty_cycle = 0
        
        # Update state
        self.current_action = "stop"
        self.current_speed = 0
        self.is_extending = False
        self.extend_start_time = None
        
        self.get_logger().info('Actuator stopped')
    
    def safety_check_callback(self):
        """Check for safety timeout on extension"""
        if self.is_extending and self.extend_start_time:
            elapsed = time.time() - self.extend_start_time
            
            if elapsed >= self.max_extend_time:
                self.get_logger().warn(
                    f'SAFETY STOP: Extension time limit ({self.max_extend_time}s) reached!'
                )
                self.stop()
    
    def publish_status(self):
        """Publish current status (for debugging)"""
        status = {
            'action': self.current_action,
            'speed': self.current_speed,
            'is_extending': self.is_extending,
            'hardware_available': HARDWARE_AVAILABLE
        }
        
        if self.is_extending and self.extend_start_time:
            status['extend_elapsed'] = time.time() - self.extend_start_time
            status['extend_remaining'] = max(0, self.max_extend_time - status['extend_elapsed'])
        
        self.get_logger().debug(f'Status: {status}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down - stopping actuator')
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = LinearActuatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    
    with open("actuator_controller/actuator_controller/actuator_node.py", "w") as f:
        f.write(content)
    print("Created actuator_node.py")

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
    pkg_dir = get_package_share_directory('actuator_controller')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'actuator_config.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to actuator configuration file'
        ),
        
        # Linear actuator controller node
        Node(
            package='actuator_controller',
            executable='actuator_node',
            name='linear_actuator_controller',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            respawn=True,
            respawn_delay=2.0
        )
    ])
'''
    
    with open("actuator_controller/launch/actuator_launch.py", "w") as f:
        f.write(content)
    print("Created actuator_launch.py")

def create_package_files():
    """Create all package configuration files"""
    files = {
        "actuator_controller/package.xml": '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>actuator_controller</name>
  <version>0.0.1</version>
  <description>ROS2 package for controlling linear actuator via PCA9685</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>actuator_interfaces</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
''',

        "actuator_controller/setup.py": '''from setuptools import setup
import os
from glob import glob

package_name = 'actuator_controller'

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
    description='ROS2 package for controlling linear actuator via PCA9685',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator_node = actuator_controller.actuator_node:main',
        ],
    },
)
''',

        "actuator_controller/setup.cfg": '''[develop]
script_dir=$base/lib/actuator_controller
[install]
install_scripts=$base/lib/actuator_controller
''',

        "actuator_controller/config/actuator_config.yaml": '''linear_actuator_controller:
  ros__parameters:
    # PCA9685 channel configuration
    in1_channel: 4      # Channel for IN1 (extend)
    in2_channel: 5      # Channel for IN2 (retract)
    
    # PWM settings
    pwm_frequency: 1000  # Hz
    
    # Safety settings
    max_extend_time: 8.0  # Maximum extension time in seconds
''',

        "actuator_interfaces/package.xml": '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>actuator_interfaces</name>
  <version>0.0.1</version>
  <description>Custom services for linear actuator control</description>
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

        "actuator_interfaces/CMakeLists.txt": '''cmake_minimum_required(VERSION 3.8)
project(actuator_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetActuator.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
''',

        "actuator_interfaces/srv/SetActuator.srv": '''# Service for controlling linear actuator
string action      # "extend", "retract", or "stop"
int32 speed        # Speed percentage (0-100)
float32 duration   # Optional duration in seconds (0 = continuous)
---
bool success       # True if command was accepted
string message     # Response message
'''
    }
    
    for filepath, content in files.items():
        with open(filepath, "w") as f:
            f.write(content)
        print(f"Created {filepath}")

def create_test_script():
    """Create a test script for the actuator"""
    content = '''#!/usr/bin/env python3
"""
Test script for linear actuator ROS2 service
"""

import rclpy
from rclpy.node import Node
from actuator_interfaces.srv import SetActuator
import sys


class ActuatorTestClient(Node):
    def __init__(self):
        super().__init__('actuator_test_client')
        self.cli = self.create_client(SetActuator, 'set_actuator')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self, action, speed=100, duration=0):
        request = SetActuator.Request()
        request.action = action
        request.speed = speed
        request.duration = duration
        
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        response = self.future.result()
        self.get_logger().info(f'Response: success={response.success}, message="{response.message}"')
        return response


def main():
    rclpy.init()
    client = ActuatorTestClient()
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run actuator_controller test_actuator <action> [speed] [duration]")
        print("  action: extend, retract, stop")
        print("  speed: 0-100 (default: 100)")
        print("  duration: seconds (default: 0 = continuous)")
        return
    
    action = sys.argv[1]
    speed = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 0
    
    client.send_request(action, speed, duration)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    
    with open("actuator_controller/actuator_controller/test_actuator.py", "w") as f:
        f.write(content)
    print("Created test_actuator.py")

def update_setup_py():
    """Update setup.py to include test script"""
    with open("actuator_controller/setup.py", "r") as f:
        content = f.read()
    
    # Add test_actuator to console_scripts
    content = content.replace(
        "'actuator_node = actuator_controller.actuator_node:main',",
        "'actuator_node = actuator_controller.actuator_node:main',\n            'test_actuator = actuator_controller.test_actuator:main',"
    )
    
    with open("actuator_controller/setup.py", "w") as f:
        f.write(content)
    print("Updated setup.py with test script")

def main():
    print("Creating ROS2 Linear Actuator Controller Package...")
    print("=" * 50)
    
    # Create directory structure
    create_directory_structure()
    
    # Create all files
    create_package_files()
    create_actuator_node()
    create_launch_file()
    create_test_script()
    update_setup_py()
    
    print("\n" + "=" * 50)
    print("✅ Package creation complete!")
    print("\nNext steps:")
    print("1. Update email and name in package.xml and setup.py files")
    print("2. Copy packages to your ROS2 workspace:")
    print("   cp -r actuator_controller/ actuator_interfaces/ ~/ros2_ws/src/")
    print("\n3. Build the packages:")
    print("   cd ~/ros2_ws")
    print("   colcon build --packages-select actuator_interfaces actuator_controller")
    print("   source install/setup.bash")
    print("\n4. Launch the actuator node:")
    print("   ros2 launch actuator_controller actuator_launch.py")
    print("\n5. Test commands:")
    print("   ros2 run actuator_controller test_actuator extend 100")
    print("   ros2 run actuator_controller test_actuator retract 50")
    print("   ros2 run actuator_controller test_actuator stop")
    print("   ros2 run actuator_controller test_actuator extend 75 3.0  # extend at 75% for 3 seconds")
    print("\n6. Service call examples:")
    print('   ros2 service call /set_actuator actuator_interfaces/srv/SetActuator "{action: \'extend\', speed: 100, duration: 0.0}"')
    print('   ros2 service call /set_actuator actuator_interfaces/srv/SetActuator "{action: \'stop\', speed: 0, duration: 0.0}"')

if __name__ == "__main__":
    main()