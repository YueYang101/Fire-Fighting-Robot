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
