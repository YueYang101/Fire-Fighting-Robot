"""
Backend package for ROS 2 Motor Control
"""

from .ros_bridge import ROSBridgeConnection, MotorController, get_ros_bridge, get_motor_controller

__all__ = [
    'ROSBridgeConnection',
    'MotorController', 
    'get_ros_bridge',
    'get_motor_controller'
]

__version__ = '1.0.0'