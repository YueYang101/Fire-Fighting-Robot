#!/usr/bin/env python3
import subprocess
import time
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Kill any existing ROS processes first to ensure clean start
    print("Cleaning up any existing ROS processes...")
    subprocess.run(['pkill', '-9', '-f', 'rosbridge_websocket'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'rosapi_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'motor_driver_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'ydlidar'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'thermal_camera_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'servo_controller_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'actuator_node'], capture_output=True)  # Add this
    subprocess.run(['pkill', '-9', '-f', 'system_monitor_node'], capture_output=True)  # Add this
    time.sleep(1)  # Give processes time to fully terminate
    print("Cleanup complete. Starting fresh...")
    
    # Define the servo controller node
    servo_controller = Node(
        package='servo_controller_py',
        executable='servo_controller_node',
        name='servo_controller',
        parameters=['/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/servo_controller_py/config/servo_config.yaml'],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Define the actuator controller node
    actuator_controller = Node(
        package='actuator_controller',
        executable='actuator_node',
        name='linear_actuator_controller',
        output='screen',
        parameters=[{
            'in1_channel': 4,
            'in2_channel': 5,
            'max_extend_time': 8.0
        }]
    )
    
    # Define the system monitor node
    system_monitor = Node(
        package='system_monitor',
        executable='system_monitor_node',
        name='system_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        # ROSbridge WebSocket - start nodes directly
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0'
            }]
        ),
        
        # ROSapi node (needed by rosbridge)
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='screen'
        ),
        
        # Motor Driver Node
        Node(
            package='pca9685_motor_driver_py',
            executable='motor_driver_node',
            name='motor_driver',
            parameters=['/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/pca9685_motor_driver_py/config/motor_map.yaml'],
            output='screen'
        ),
        
        # YDLIDAR
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch.py',
                 'params_file:=/home/ubuntu-robot-pi4/ros2_ws/src/ydlidar_ros2_driver/params/G2.yaml'],
            output='screen'
        ),
        
        # Thermal Camera - using direct path
        ExecuteProcess(
            cmd=['/home/ubuntu-robot-pi4/ros2_ws/install/mlx90640_driver/bin/thermal_camera_node'],
            output='screen'
        ),
        
        # System Monitor Node
        system_monitor,
        
        # Actuator Controller Node
        actuator_controller,
        
        # Servo Controller Node - Start with 3 second delay to avoid conflicts
        TimerAction(
            period=3.0,
            actions=[servo_controller]
        )
    ])