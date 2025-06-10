#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROSbridge WebSocket
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
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
        
        # Thermal Camera
        Node(
            package='mlx90640_driver',
            executable='thermal_camera_node',
            name='thermal_camera_node',
            output='screen'
        )
    ])