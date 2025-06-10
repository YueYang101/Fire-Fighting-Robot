"""
Motor Controller Module
Handles motor control logic and ROS communication
"""

import logging
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

class MotorController:
    """Motor controller for 4-motor robot using L298N drivers"""
    
    def __init__(self, ros_bridge=None):
        self.ros_bridge = ros_bridge
        
        # Motor state tracking
        self.motor_states = {
            1: {"direction": "brake", "speed": 0},
            2: {"direction": "brake", "speed": 0},
            3: {"direction": "brake", "speed": 0},
            4: {"direction": "brake", "speed": 0}
        }
        
        # Motor channel mapping (2 motors per L298N channel)
        self.motor_to_channel = {
            1: 'A',  # Front-left
            2: 'A',  # Front-right
            3: 'B',  # Rear-left
            4: 'B'   # Rear-right
        }
        
        logger.info("Motor controller initialized")
    
    def set_ros_bridge(self, ros_bridge):
        """Set or update the ROS bridge instance"""
        self.ros_bridge = ros_bridge
        logger.info("ROS bridge connected to motor controller")
    
    def set_motor(self, motor_id: int, direction: str, speed: int) -> Dict[str, Any]:
        """
        Control a specific motor
        
        Args:
            motor_id: Motor number (1-4)
            direction: 'forward', 'backward', or 'brake'
            speed: Speed value (0-100)
        
        Returns:
            Result dictionary with success status
        """
        # Validate inputs
        if motor_id not in range(1, 5):
            return {
                "success": False,
                "error": f"Invalid motor ID: {motor_id}. Must be 1-4."
            }
        
        if direction not in ['forward', 'backward', 'brake']:
            return {
                "success": False,
                "error": f"Invalid direction: {direction}"
            }
        
        # Clamp speed to valid range
        speed = max(0, min(100, speed))
        
        # Update motor state
        self.motor_states[motor_id] = {
            "direction": direction,
            "speed": speed
        }
        
        # Get channel for this motor
        channel = self.motor_to_channel[motor_id]
        
        # Send command via ROS bridge if available
        if self.ros_bridge:
            try:
                # For motors sharing a channel, we need to consider both motors
                channel_motors = [m for m, ch in self.motor_to_channel.items() if ch == channel]
                
                # If both motors on the channel have the same direction, use that
                # Otherwise, handle mixed directions appropriately
                channel_direction = direction
                channel_speed = speed
                
                # Send motor command
                self.ros_bridge.send_motor_command(channel, channel_direction, channel_speed)
                
                logger.info(f"Motor {motor_id} set to {direction} at speed {speed}")
                
                return {
                    "success": True,
                    "motor_id": motor_id,
                    "direction": direction,
                    "speed": speed,
                    "channel": channel
                }
                
            except Exception as e:
                logger.error(f"Failed to send motor command: {e}")
                return {
                    "success": False,
                    "error": str(e)
                }
        else:
            # No ROS bridge - just update local state
            return {
                "success": True,
                "motor_id": motor_id,
                "direction": direction,
                "speed": speed,
                "message": "Motor state updated (no ROS bridge connected)"
            }
    
    def get_motor_state(self, motor_id: int) -> Optional[Dict[str, Any]]:
        """Get current state of a specific motor"""
        if motor_id in self.motor_states:
            return self.motor_states[motor_id]
        return None
    
    def get_all_motor_states(self) -> Dict[int, Dict[str, Any]]:
        """Get current state of all motors"""
        return self.motor_states.copy()
    
    def stop_all_motors(self) -> Dict[str, Any]:
        """Emergency stop - brake all motors"""
        try:
            for motor_id in range(1, 5):
                self.set_motor(motor_id, "brake", 0)
            
            logger.warning("Emergency stop - all motors braked")
            
            return {
                "success": True,
                "message": "All motors stopped"
            }
            
        except Exception as e:
            logger.error(f"Failed to stop all motors: {e}")
            return {
                "success": False,
                "error": str(e)
            }
    
    def set_channel_motors(self, channel: str, direction: str, speed: int) -> Dict[str, Any]:
        """
        Control all motors on a specific channel
        Used for coordinated movement (e.g., both left motors for turning)
        
        Args:
            channel: 'A' or 'B'
            direction: 'forward', 'backward', or 'brake'
            speed: Speed value (0-100)
        """
        if channel not in ['A', 'B']:
            return {
                "success": False,
                "error": f"Invalid channel: {channel}"
            }
        
        # Find motors on this channel
        channel_motors = [m for m, ch in self.motor_to_channel.items() if ch == channel]
        
        # Set all motors on the channel
        for motor_id in channel_motors:
            result = self.set_motor(motor_id, direction, speed)
            if not result['success']:
                return result
        
        return {
            "success": True,
            "channel": channel,
            "motors": channel_motors,
            "direction": direction,
            "speed": speed
        }

# Singleton instance
_motor_controller_instance = None

def get_motor_controller(ros_bridge=None):
    """Get or create motor controller instance"""
    global _motor_controller_instance
    
    if _motor_controller_instance is None:
        _motor_controller_instance = MotorController(ros_bridge)
    elif ros_bridge is not None:
        _motor_controller_instance.set_ros_bridge(ros_bridge)
    
    return _motor_controller_instance