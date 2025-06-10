"""
Servo Control Interface for Aiming System
Controls pan and tilt servos through ROS2 topics
"""

import asyncio
import json
import logging
from typing import Dict, Any, Optional, Callable
from datetime import datetime

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
    
    async def move_to_position(self, pan: float, tilt: float) -> Dict[str, Any]:
        """
        Move servos to specified position
        
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
        if self.ros_bridge and self.ros_bridge.connected:
            try:
                await self.ros_bridge.publish_servo_position(pan, tilt)
                logger.info(f"Servo command sent: pan={pan}, tilt={tilt}")
            except Exception as e:
                logger.error(f"Failed to send servo command: {e}")
                return {
                    'success': False,
                    'error': str(e)
                }
        
        # Simulate movement completion after a delay
        asyncio.create_task(self._simulate_movement(pan, tilt))
        
        return {
            'success': True,
            'pan_angle': pan,
            'tilt_angle': tilt,
            'message': f"Moving to pan={pan}°, tilt={tilt}°"
        }
    
    async def _simulate_movement(self, target_pan: float, target_tilt: float):
        """Simulate servo movement with gradual position updates"""
        steps = 10
        pan_step = (target_pan - self.pan_angle) / steps
        tilt_step = (target_tilt - self.tilt_angle) / steps
        
        for i in range(steps):
            await asyncio.sleep(0.05)  # 50ms per step
            
            # Update positions
            if self.pan_moving:
                self.pan_angle += pan_step
            if self.tilt_moving:
                self.tilt_angle += tilt_step
            
            # Send update via WebSocket
            await self._send_state_update()
        
        # Final position
        self.pan_angle = target_pan
        self.tilt_angle = target_tilt
        self.pan_moving = False
        self.tilt_moving = False
        
        # Send final update
        await self._send_state_update()
        logger.info(f"Servo movement completed: pan={self.pan_angle}, tilt={self.tilt_angle}")
    
    async def _send_state_update(self):
        """Send current state via WebSocket"""
        if self.websocket_handler:
            state = self.get_state()
            await self.websocket_handler({
                'type': 'servo_state',
                **state
            })
    
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
    
    async def center_position(self) -> Dict[str, Any]:
        """Move servos to center position (135, 135)"""
        return await self.move_to_position(135.0, 135.0)
    
    async def move_left(self) -> Dict[str, Any]:
        """Move to left position"""
        return await self.move_to_position(75.0, 135.0)
    
    async def move_right(self) -> Dict[str, Any]:
        """Move to right position"""
        return await self.move_to_position(195.0, 135.0)
    
    async def move_up(self) -> Dict[str, Any]:
        """Move to up position"""
        return await self.move_to_position(135.0, 195.0)
    
    async def move_down(self) -> Dict[str, Any]:
        """Move to down position"""
        return await self.move_to_position(135.0, 75.0)
    
    async def emergency_stop(self):
        """Stop all servo movement"""
        self.pan_moving = False
        self.tilt_moving = False
        self.target_pan = self.pan_angle
        self.target_tilt = self.tilt_angle
        
        logger.warning("Emergency stop activated for servos")
        await self._send_state_update()
    
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
    
    async def handle_websocket_command(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle commands from WebSocket"""
        action = data.get('action')
        
        if action == 'move':
            pan = data.get('pan_angle', self.pan_angle)
            tilt = data.get('tilt_angle', self.tilt_angle)
            return await self.move_to_position(pan, tilt)
        
        elif action == 'center':
            return await self.center_position()
        
        elif action == 'get_state':
            return self.get_state()
        
        elif action == 'emergency_stop':
            await self.emergency_stop()
            return {'success': True, 'message': 'Emergency stop activated'}
        
        else:
            return {'success': False, 'error': f'Unknown action: {action}'}