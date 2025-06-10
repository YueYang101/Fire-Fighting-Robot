#!/bin/bash

# ROS2 System Monitor Package Generator
# This script creates a complete ROS2 Humble package for system monitoring

PACKAGE_NAME="system_monitor"
WORKSPACE_NAME="system_monitor_ws"

echo "🚀 Creating ROS2 System Monitor Package..."

# Create workspace structure
mkdir -p $WORKSPACE_NAME/src
cd $WORKSPACE_NAME/src

# Create package directory structure
mkdir -p $PACKAGE_NAME/$PACKAGE_NAME
mkdir -p $PACKAGE_NAME/launch
mkdir -p $PACKAGE_NAME/msg

# Create package.xml
cat > $PACKAGE_NAME/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>system_monitor</name>
  <version>0.1.0</version>
  <description>ROS2 package for monitoring system metrics on Raspberry Pi</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt
cat > $PACKAGE_NAME/CMakeLists.txt << 'EOF'
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
EOF

# Create setup.py
cat > $PACKAGE_NAME/setup.py << 'EOF'
from setuptools import setup

package_name = 'system_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 System Monitor for Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
)
EOF

# Create custom message files
cat > $PACKAGE_NAME/msg/SystemLoad.msg << 'EOF'
# System Load Message
std_msgs/Header header
float32 cpu_percent
float32[] cpu_percent_per_core
float32 memory_percent
float32 memory_used_gb
float32 memory_total_gb
float32 swap_percent
float32[3] load_average  # 1min, 5min, 15min
EOF

cat > $PACKAGE_NAME/msg/ProcessInfo.msg << 'EOF'
# Process Information Message
string name
int32 pid
float32 cpu_percent
float32 memory_percent
EOF

cat > $PACKAGE_NAME/msg/NetworkStats.msg << 'EOF'
# Network Statistics Message
float32 bytes_sent_mb
float32 bytes_recv_mb
int64 packets_sent
int64 packets_recv
int32 errors_in
int32 errors_out
EOF

cat > $PACKAGE_NAME/msg/DiskUsage.msg << 'EOF'
# Disk Usage Message
string mount_point
string device
float32 percent
float32 used_gb
float32 total_gb
float32 free_gb
EOF

cat > $PACKAGE_NAME/msg/SystemStatus.msg << 'EOF'
# Complete System Status Message
std_msgs/Header header
string hostname
float32 cpu_temperature
float32 gpu_temperature
SystemLoad system_load
NetworkStats network_stats
DiskUsage[] disk_usage
ProcessInfo[] top_cpu_processes
ProcessInfo[] top_memory_processes
int32 process_count
int32 thread_count
float32 uptime_hours
EOF

# Create __init__.py
cat > $PACKAGE_NAME/$PACKAGE_NAME/__init__.py << 'EOF'
# System Monitor Package
EOF

# Create the main system monitor node
cat > $PACKAGE_NAME/$PACKAGE_NAME/system_monitor_node.py << 'EOF'
#!/usr/bin/env python3
"""
ROS2 System Monitor Node
Publishes system metrics for Raspberry Pi
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32, String
from sensor_msgs.msg import Temperature
import psutil
import os
import subprocess
import json
from datetime import datetime

# Import custom messages (will be generated)
from system_monitor.msg import (
    SystemLoad, ProcessInfo, NetworkStats, 
    DiskUsage, SystemStatus
)


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('enable_detailed_logs', False)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_detailed_logs = self.get_parameter('enable_detailed_logs').value
        
        # Create publishers
        self.temp_pub = self.create_publisher(Temperature, 'system/temperature', 10)
        self.cpu_pub = self.create_publisher(Float32, 'system/cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, 'system/memory_usage', 10)
        self.system_load_pub = self.create_publisher(SystemLoad, 'system/load', 10)
        self.network_pub = self.create_publisher(NetworkStats, 'system/network', 10)
        self.status_pub = self.create_publisher(SystemStatus, 'system/status', 10)
        self.json_pub = self.create_publisher(String, 'system/status_json', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f'System Monitor Node started (rate: {self.publish_rate} Hz)')
    
    def get_cpu_temperature(self):
        """Get CPU temperature"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return float(f.read()) / 1000.0
        except:
            try:
                result = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
                return float(result.replace('temp=', '').replace("'C\\n", ''))
            except:
                return -1.0
    
    def get_gpu_temperature(self):
        """Get GPU temperature for Raspberry Pi"""
        try:
            result = subprocess.check_output(['vcgencmd', 'measure_temp', 'gpu']).decode()
            return float(result.replace('temp=', '').replace("'C\\n", ''))
        except:
            return -1.0
    
    def get_system_load(self):
        """Get system load information"""
        msg = SystemLoad()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # CPU info
        msg.cpu_percent = psutil.cpu_percent(interval=0.1)
        msg.cpu_percent_per_core = list(psutil.cpu_percent(interval=0.1, percpu=True))
        
        # Memory info
        mem = psutil.virtual_memory()
        msg.memory_percent = mem.percent
        msg.memory_used_gb = mem.used / (1024**3)
        msg.memory_total_gb = mem.total / (1024**3)
        
        # Swap info
        swap = psutil.swap_memory()
        msg.swap_percent = swap.percent
        
        # Load average
        msg.load_average = list(psutil.getloadavg())
        
        return msg
    
    def get_network_stats(self):
        """Get network statistics"""
        msg = NetworkStats()
        net_io = psutil.net_io_counters()
        
        msg.bytes_sent_mb = net_io.bytes_sent / (1024**2)
        msg.bytes_recv_mb = net_io.bytes_recv / (1024**2)
        msg.packets_sent = net_io.packets_sent
        msg.packets_recv = net_io.packets_recv
        msg.errors_in = net_io.errin
        msg.errors_out = net_io.errout
        
        return msg
    
    def get_disk_usage(self):
        """Get disk usage for all partitions"""
        disk_msgs = []
        partitions = psutil.disk_partitions()
        
        for partition in partitions:
            try:
                usage = psutil.disk_usage(partition.mountpoint)
                msg = DiskUsage()
                msg.mount_point = partition.mountpoint
                msg.device = partition.device
                msg.percent = usage.percent
                msg.used_gb = usage.used / (1024**3)
                msg.total_gb = usage.total / (1024**3)
                msg.free_gb = usage.free / (1024**3)
                disk_msgs.append(msg)
            except PermissionError:
                continue
        
        return disk_msgs
    
    def get_top_processes(self, by='cpu', top_n=5):
        """Get top N processes by CPU or memory"""
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
            try:
                processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        
        key = 'cpu_percent' if by == 'cpu' else 'memory_percent'
        sorted_procs = sorted(processes, key=lambda x: x[key], reverse=True)[:top_n]
        
        process_msgs = []
        for proc in sorted_procs:
            msg = ProcessInfo()
            msg.name = proc['name']
            msg.pid = proc['pid']
            msg.cpu_percent = proc['cpu_percent']
            msg.memory_percent = proc['memory_percent']
            process_msgs.append(msg)
        
        return process_msgs
    
    def timer_callback(self):
        """Main timer callback to publish all metrics"""
        # Temperature
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = "cpu"
        temp_msg.temperature = self.get_cpu_temperature()
        temp_msg.variance = 0.0
        self.temp_pub.publish(temp_msg)
        
        # Simple CPU and Memory
        cpu_msg = Float32()
        cpu_msg.data = psutil.cpu_percent(interval=0.1)
        self.cpu_pub.publish(cpu_msg)
        
        memory_msg = Float32()
        memory_msg.data = psutil.virtual_memory().percent
        self.memory_pub.publish(memory_msg)
        
        # Detailed system load
        system_load = self.get_system_load()
        self.system_load_pub.publish(system_load)
        
        # Network stats
        network_stats = self.get_network_stats()
        self.network_pub.publish(network_stats)
        
        # Complete system status
        status_msg = SystemStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.hostname = os.uname().nodename
        status_msg.cpu_temperature = self.get_cpu_temperature()
        status_msg.gpu_temperature = self.get_gpu_temperature()
        status_msg.system_load = system_load
        status_msg.network_stats = network_stats
        status_msg.disk_usage = self.get_disk_usage()
        status_msg.top_cpu_processes = self.get_top_processes('cpu')
        status_msg.top_memory_processes = self.get_top_processes('memory')
        status_msg.process_count = len(psutil.pids())
        status_msg.thread_count = sum([p.num_threads() for p in psutil.process_iter(['num_threads'])])
        status_msg.uptime_hours = (psutil.time.time() - psutil.boot_time()) / 3600.0
        
        self.status_pub.publish(status_msg)
        
        # Also publish as JSON for easy consumption
        json_data = {
            'timestamp': datetime.now().isoformat(),
            'hostname': status_msg.hostname,
            'cpu_temperature': status_msg.cpu_temperature,
            'gpu_temperature': status_msg.gpu_temperature,
            'cpu_percent': cpu_msg.data,
            'memory_percent': memory_msg.data,
            'uptime_hours': status_msg.uptime_hours,
            'process_count': status_msg.process_count
        }
        json_msg = String()
        json_msg.data = json.dumps(json_data)
        self.json_pub.publish(json_msg)
        
        if self.enable_detailed_logs:
            self.get_logger().info(
                f'CPU: {cpu_msg.data:.1f}% | '
                f'Memory: {memory_msg.data:.1f}% | '
                f'Temp: {status_msg.cpu_temperature:.1f}°C'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# Create a simple publisher example
cat > $PACKAGE_NAME/$PACKAGE_NAME/system_info_publisher.py << 'EOF'
#!/usr/bin/env python3
"""
Simple System Info Publisher
Publishes basic system information at regular intervals
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json
from datetime import datetime


class SystemInfoPublisher(Node):
    def __init__(self):
        super().__init__('system_info_publisher')
        self.publisher_ = self.create_publisher(String, 'system_info', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('System Info Publisher started')

    def timer_callback(self):
        msg = String()
        
        # Collect system info
        info = {
            'timestamp': datetime.now().isoformat(),
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent,
            'boot_time': datetime.fromtimestamp(psutil.boot_time()).isoformat(),
            'network_bytes_sent': psutil.net_io_counters().bytes_sent,
            'network_bytes_recv': psutil.net_io_counters().bytes_recv
        }
        
        msg.data = json.dumps(info, indent=2)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Publishing: CPU={info["cpu_percent"]:.1f}%, MEM={info["memory_percent"]:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    publisher = SystemInfoPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# Create launch file
cat > $PACKAGE_NAME/launch/system_monitor_launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )
    
    enable_logs_arg = DeclareLaunchArgument(
        'enable_detailed_logs',
        default_value='false',
        description='Enable detailed logging'
    )
    
    # Create nodes
    system_monitor_node = Node(
        package='system_monitor',
        executable='system_monitor_node.py',
        name='system_monitor',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'enable_detailed_logs': LaunchConfiguration('enable_detailed_logs')
        }],
        output='screen'
    )
    
    system_info_publisher = Node(
        package='system_monitor',
        executable='system_info_publisher.py',
        name='system_info_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        publish_rate_arg,
        enable_logs_arg,
        system_monitor_node,
        system_info_publisher
    ])
EOF

# Create README
cat > $PACKAGE_NAME/README.md << 'EOF'
# ROS2 System Monitor Package

A ROS2 Humble package for monitoring system metrics on Raspberry Pi and other Linux systems.

## Features

- CPU usage and temperature monitoring
- Memory and swap usage
- Network statistics
- Disk usage for all partitions
- Top processes by CPU and memory
- Custom ROS2 messages for detailed system information
- JSON output for easy integration

## Topics Published

- `/system/temperature` (sensor_msgs/Temperature) - CPU temperature
- `/system/cpu_usage` (std_msgs/Float32) - CPU usage percentage
- `/system/memory_usage` (std_msgs/Float32) - Memory usage percentage
- `/system/load` (system_monitor/SystemLoad) - Detailed system load
- `/system/network` (system_monitor/NetworkStats) - Network statistics
- `/system/status` (system_monitor/SystemStatus) - Complete system status
- `/system/status_json` (std_msgs/String) - JSON formatted status
- `/system_info` (std_msgs/String) - Simple system info in JSON

## Installation

1. Clone into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <this_repository>
```

2. Install dependencies:
```bash
sudo apt update
sudo apt install python3-psutil
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select system_monitor
source install/setup.bash
```

## Usage

### Run the main system monitor node:
```bash
ros2 run system_monitor system_monitor_node.py
```

### Run with custom parameters:
```bash
ros2 run system_monitor system_monitor_node.py --ros-args -p publish_rate:=2.0 -p enable_detailed_logs:=true
```

### Launch both nodes:
```bash
ros2 launch system_monitor system_monitor_launch.py
```

### Launch with arguments:
```bash
ros2 launch system_monitor system_monitor_launch.py publish_rate:=5.0 enable_detailed_logs:=true
```

## View Topics

```bash
# List all topics
ros2 topic list

# Echo CPU usage
ros2 topic echo /system/cpu_usage

# Echo complete system status as JSON
ros2 topic echo /system/status_json

# Monitor temperature
ros2 topic echo /system/temperature
```

## Parameters

- `publish_rate` (float, default: 1.0) - Publishing rate in Hz
- `enable_detailed_logs` (bool, default: false) - Enable detailed console logging

## Custom Messages

The package defines several custom messages:
- `SystemLoad.msg` - CPU, memory, and load information
- `ProcessInfo.msg` - Process details
- `NetworkStats.msg` - Network statistics
- `DiskUsage.msg` - Disk partition usage
- `SystemStatus.msg` - Complete system status

## Requirements

- ROS2 Humble
- Python 3.8+
- psutil Python package
EOF

# Make scripts executable
chmod +x $PACKAGE_NAME/$PACKAGE_NAME/*.py

# Create a build script
cd ../..
cat > build_and_run.sh << 'EOF'
#!/bin/bash

# Build and run the ROS2 System Monitor package

echo "🔨 Building ROS2 System Monitor Package..."

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash 2>/dev/null || echo "⚠️  ROS2 Humble not found in /opt/ros/humble"
fi

# Build the package
colcon build --packages-select system_monitor

# Source the workspace
source install/setup.bash

echo "✅ Build complete!"
echo ""
echo "📋 Available commands:"
echo "  - Run system monitor node:"
echo "    ros2 run system_monitor system_monitor_node.py"
echo ""
echo "  - Launch all nodes:"
echo "    ros2 launch system_monitor system_monitor_launch.py"
echo ""
echo "  - View topics:"
echo "    ros2 topic list"
echo "    ros2 topic echo /system/cpu_usage"
echo "    ros2 topic echo /system/status_json"
EOF

chmod +x build_and_run.sh

echo "✅ ROS2 System Monitor package created successfully!"
echo ""
echo "📁 Structure created:"
echo "  $WORKSPACE_NAME/"
echo "  ├── src/"
echo "  │   └── $PACKAGE_NAME/"
echo "  │       ├── $PACKAGE_NAME/"
echo "  │       │   ├── __init__.py"
echo "  │       │   ├── system_monitor_node.py"
echo "  │       │   └── system_info_publisher.py"
echo "  │       ├── launch/"
echo "  │       │   └── system_monitor_launch.py"
echo "  │       ├── msg/"
echo "  │       │   ├── SystemLoad.msg"
echo "  │       │   ├── ProcessInfo.msg"
echo "  │       │   ├── NetworkStats.msg"
echo "  │       │   ├── DiskUsage.msg"
echo "  │       │   └── SystemStatus.msg"
echo "  │       ├── CMakeLists.txt"
echo "  │       ├── package.xml"
echo "  │       ├── setup.py"
echo "  │       └── README.md"
echo "  └── build_and_run.sh"
echo ""
echo "🚀 Next steps:"
echo "  1. cd $WORKSPACE_NAME"
echo "  2. ./build_and_run.sh"
EOF