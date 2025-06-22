from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('mlx90640_driver')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'thermal_camera.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),
        
        DeclareLaunchArgument(
            'output',
            default_value='screen',
            description='Output type for node'
        ),
        
        Node(
            package='mlx90640_driver',
            executable='thermal_camera_node',
            name='thermal_camera_node',
            output=LaunchConfiguration('output'),
            parameters=[LaunchConfiguration('config_file')],
            respawn=True,  # Restart if it crashes
            respawn_delay=2.0,  # Wait 2 seconds before restarting
        )
    ])