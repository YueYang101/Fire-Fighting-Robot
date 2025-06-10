from launch import LaunchDescription
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