#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pca9685_dir = get_package_share_directory('pca9685_motor_driver_py')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    mlx90640_dir = get_package_share_directory('mlx90640_driver')
    
    # Config files
    motor_config = os.path.join(
        '/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/pca9685_motor_driver_py/config',
        'motor_map.yaml'
    )
    
    lidar_config = os.path.join(
        '/home/ubuntu-robot-pi4/ros2_ws/src/ydlidar_ros2_driver/params',
        'G2.yaml'
    )
    
    return LaunchDescription([
        # ROSbridge WebSocket
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('rosbridge_server'),
                '/launch/rosbridge_websocket_launch.xml'
            ])
        ),
        
        # Motor Driver Node
        Node(
            package='pca9685_motor_driver_py',
            executable='motor_driver_node',
            name='motor_driver',
            parameters=[motor_config],
            output='screen'
        ),
        
        # YDLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ydlidar_dir,
                '/launch/ydlidar_launch.py'
            ]),
            launch_arguments={'params_file': lidar_config}.items()
        ),
        
        # Thermal Camera
        Node(
            package='mlx90640_driver',
            executable='thermal_camera_node',
            name='thermal_camera_node',
            output='screen'
        )
    ])