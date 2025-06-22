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
