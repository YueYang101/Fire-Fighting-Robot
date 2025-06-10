"""
Servo Control Interface for Aiming System
Controls pan and tilt servos through ROS2 topics
"""

import json
import logging
from typing import Dict, Any, Optional, Callable
from datetime import datetime
import asyncio

logger = logging.getLogger(__name__)

class ServoController:
    def __init__(self):
        self.pan_angle = 135.0  # Default center position
        self.tilt_angle = 135.0  # Default center position
        self.pan_moving = False
        self.tilt_moving = False
        self.connected = False
        self.ros_bridge = None
        self.websocket_handler = None
        self.last_command_time = None
        
        # Servo limits
        self.pan_min = 75.0
        self.pan_max = 195.0
        self.tilt_min = 75.0
        self.tilt_max = 195.0
        
        # Movement tracking
        self.target_pan = 135.0
        self.target_tilt = 135.0
        self.movement_speed = 60.0  # degrees per second
        
        logger.info("Servo controller initialized")
    
    def set_ros_bridge(self, ros_bridge):
        """Set the ROS bridge instance for publishing commands"""
        self.ros_bridge = ros_bridge
        self.connected = True
        logger.info("ROS bridge connected to servo controller")
    
    def set_websocket_handler(self, handler: Callable):
        """Set the WebSocket handler for sending updates"""
        self.websocket_handler = handler
    
    def move_to_position(self, pan: float, tilt: float) -> Dict[str, Any]:
        """
        Move servos to specified position (synchronous version)
        
        Args:
            pan: Pan angle (75-195 degrees)
            tilt: Tilt angle (75-195 degrees)
            
        Returns:
            Status dictionary
        """
        # Run async version in sync context
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        result = loop.run_until_complete(self._async_move_to_position(pan, tilt))
        loop.close()
        return result
    
    async def _async_move_to_position(self, pan: float, tilt: float) -> Dict[str, Any]:
        """
        Move servos to specified position (async version)
        
        Args:
            pan: Pan angle (75-195 degrees)
            tilt: Tilt angle (75-195 degrees)
            
        Returns:
            Status dictionary
        """
        # Validate angles
        pan = max(self.pan_min, min(self.pan_max, pan))
        tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        
        self.target_pan = pan
        self.target_tilt = tilt
        self.last_command_time = datetime.now()
        
        # Check if movement needed
        if abs(self.pan_angle - pan) > 0.5:
            self.pan_moving = True
        if abs(self.tilt_angle - tilt) > 0.5:
            self.tilt_moving = True
        
        # Send ROS command if bridge available
        if self.ros_bridge:
            try:
                from backend.ros_bridge import get_servo_publisher
                servo_pub = get_servo_publisher()
                success = servo_pub.publish_position(pan, tilt)
                if success:
                    logger.info(f"Servo command sent: pan={pan}, tilt={tilt}")
                else:
                    logger.error("Failed to publish servo command")
                    return {
                        'success': False,
                        'error': 'Failed to publish servo command'
                    }
            except Exception as e:
                logger.error(f"Failed to send servo command: {e}")
                return {
                    'success': False,
                    'error': str(e)
                }
        
        # Update positions immediately for UI feedback
        self.pan_angle = pan
        self.tilt_angle = tilt
        self.pan_moving = False
        self.tilt_moving = False
        
        return {
            'success': True,
            'pan_angle': pan,
            'tilt_angle': tilt,
            'message': f"Moving to pan={pan}°, tilt={tilt}°"
        }
    
    def get_state(self) -> Dict[str, Any]:
        """Get current servo state"""
        return {
            'pan_angle': round(self.pan_angle, 1),
            'tilt_angle': round(self.tilt_angle, 1),
            'pan_moving': self.pan_moving,
            'tilt_moving': self.tilt_moving,
            'connected': self.connected,
            'target_pan': self.target_pan,
            'target_tilt': self.target_tilt,
            'timestamp': datetime.now().isoformat()
        }
    
    def center_position(self) -> Dict[str, Any]:
        """Move servos to center position (135, 135)"""
        return self.move_to_position(135.0, 135.0)
    
    def move_left(self) -> Dict[str, Any]:
        """Move to left position"""
        return self.move_to_position(75.0, 135.0)
    
    def move_right(self) -> Dict[str, Any]:
        """Move to right position"""
        return self.move_to_position(195.0, 135.0)
    
    def move_up(self) -> Dict[str, Any]:
        """Move to up position"""
        return self.move_to_position(135.0, 195.0)
    
    def move_down(self) -> Dict[str, Any]:
        """Move to down position"""
        return self.move_to_position(135.0, 75.0)
    
    def emergency_stop(self):
        """Stop all servo movement"""
        self.pan_moving = False
        self.tilt_moving = False
        self.target_pan = self.pan_angle
        self.target_tilt = self.tilt_angle
        logger.warning("Emergency stop activated for servos")
    
    def process_ros_feedback(self, data: Dict[str, Any]):
        """Process feedback from ROS about actual servo positions"""
        if 'pan_angle' in data:
            self.pan_angle = data['pan_angle']
        if 'tilt_angle' in data:
            self.tilt_angle = data['tilt_angle']
        
        # Check if reached target
        if abs(self.pan_angle - self.target_pan) < 1.0:
            self.pan_moving = False
        if abs(self.tilt_angle - self.target_tilt) < 1.0:
            self.tilt_moving = False
    
    def handle_websocket_command(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle commands from WebSocket (synchronous)"""
        action = data.get('action')
        
        if action == 'move':
            pan = data.get('pan_angle', self.pan_angle)
            tilt = data.get('tilt_angle', self.tilt_angle)
            return self.move_to_position(pan, tilt)
        
        elif action == 'center':
            return self.center_position()
        
        elif action == 'get_state':
            return self.get_state()
        
        elif action == 'emergency_stop':
            self.emergency_stop()
            return {'success': True, 'message': 'Emergency stop activated'}
        
        else:
            return {'success': False, 'error': f'Unknown action: {action}'}