from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('pca9685_motor_driver_py'),
        'config', 'motor_map.yaml')
    return LaunchDescription([
        Node(
            package='pca9685_motor_driver_py',
            executable='motor_driver_node',
            parameters=[cfg])
    ])
