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
