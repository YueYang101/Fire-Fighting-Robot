#!/usr/bin/env python3
"""
Actuator Control Module for Fire Extinguisher Trigger
Handles linear actuator control through ROS2 service calls
"""

import logging
from typing import Dict, Any, Optional
from datetime import datetime

logger = logging.getLogger(__name__)

class ActuatorController:
    """Controls the linear actuator for fire extinguisher trigger mechanism"""
    
    def __init__(self, ros_bridge=None):
        """
        Initialize actuator controller
        
        Args:
            ros_bridge: ROSBridgeConnection instance
        """
        self.ros_bridge = ros_bridge
        self.connected = False
        self.current_state = "stopped"
        self.current_speed = 0
        self.last_command_time = None
        
        logger.info("Actuator controller initialized")
    
    def set_ros_bridge(self, ros_bridge):
        """Set or update the ROS bridge instance"""
        self.ros_bridge = ros_bridge
        self.connected = True
        logger.info("ROS bridge connected to actuator controller")
    
    def control_actuator(self, action: str, speed: int = 0, duration: float = 0.0) -> Dict[str, Any]:
        """
        Control the actuator
        
        Args:
            action: 'extend', 'retract', or 'stop'
            speed: Speed percentage (0-100)
            duration: Duration in seconds (0 for continuous)
            
        Returns:
            Dict with operation result
        """
        # Validate inputs
        if action not in ['extend', 'retract', 'stop']:
            return {
                "success": False,
                "error": f"Invalid action: {action}. Must be 'extend', 'retract', or 'stop'"
            }
        
        if action != 'stop':
            speed = max(0, min(100, speed))
        else:
            speed = 0
        
        duration = max(0.0, duration)
        
        # Update state
        self.current_state = action
        self.current_speed = speed
        self.last_command_time = datetime.now()
        
        # Send command via ROS bridge if available
        if self.ros_bridge:
            try:
                # Call the set_actuator service with proper type
                service_args = {
                    "action": action,
                    "speed": speed,
                    "duration": duration
                }
                
                logger.info(f"Calling /set_actuator with args: {service_args}")
                
                # Add service type for rosbridge
                success, response = self.ros_bridge.call_service(
                    "/set_actuator",
                    service_args,
                    "set_actuator",
                    service_type="actuator_interfaces/srv/SetActuator"
                )
                
                logger.info(f"Service call response: success={success}, response={response}")
                
                # Check different response formats
                if success:
                    # Extract the actual service response from rosbridge format
                    service_response = response.get("values", {})
                    
                    # Check if the service itself returned success
                    service_success = service_response.get("success", True)
                    service_message = service_response.get("message", "Command sent")
                    
                    if service_success:
                        logger.info(f"Actuator command successful: {service_message}")
                        return {
                            "success": True,
                            "action": action,
                            "speed": speed,
                            "duration": duration,
                            "message": service_message or f"Actuator {action} at {speed}% speed" + (f" for {duration}s" if duration > 0 else " (continuous)"),
                            "response": service_response
                        }
                    else:
                        logger.error(f"Service returned failure: {service_message}")
                        return {
                            "success": False,
                            "error": service_message or "Actuator command failed",
                            "details": service_response
                        }
                else:
                    error_msg = response.get("error", response.get("msg", "Unknown error"))
                    logger.error(f"Failed to call service: {error_msg}")
                    logger.error(f"Full response: {response}")
                    return {
                        "success": False,
                        "error": error_msg,
                        "details": response
                    }
                    
            except Exception as e:
                logger.error(f"Failed to send actuator command: {e}")
                return {
                    "success": False,
                    "error": str(e)
                }
        else:
            # No ROS bridge - just update local state
            return {
                "success": True,
                "action": action,
                "speed": speed,
                "duration": duration,
                "message": "Actuator command simulated (no ROS bridge connected)"
            }
    
    def extend(self, speed: int = 100, duration: float = 0.0) -> Dict[str, Any]:
        """Extend the actuator"""
        return self.control_actuator("extend", speed, duration)
    
    def retract(self, speed: int = 100, duration: float = 0.0) -> Dict[str, Any]:
        """Retract the actuator"""
        return self.control_actuator("retract", speed, duration)
    
    def stop(self) -> Dict[str, Any]:
        """Stop the actuator immediately"""
        return self.control_actuator("stop", 0, 0.0)
    
    def get_state(self) -> Dict[str, Any]:
        """Get current actuator state"""
        return {
            "state": self.current_state,
            "speed": self.current_speed,
            "connected": self.connected,
            "last_command": self.last_command_time.isoformat() if self.last_command_time else None,
            "timestamp": datetime.now().isoformat()
        }
    
    def test_connection(self) -> bool:
        """Test if actuator service is available"""
        if not self.ros_bridge:
            self.connected = False
            return False
            
        try:
            # Try to call service with a test command
            success, response = self.ros_bridge.call_service(
                "/set_actuator",
                {"action": "stop", "speed": 0, "duration": 0.0},
                "test_actuator_connection",
                service_type="actuator_interfaces/srv/SetActuator"
            )
            
            self.connected = success
            return success
            
        except Exception as e:
            logger.error(f"Actuator connection test failed: {e}")
            self.connected = False
            return False
    
    def emergency_stop(self) -> Dict[str, Any]:
        """Emergency stop - immediately stop actuator"""
        logger.warning("Emergency stop activated for actuator")
        return self.stop()


# Singleton instance
_actuator_controller = None

def get_actuator_controller(ros_bridge=None):
    """Get or create actuator controller instance"""
    global _actuator_controller
    
    if _actuator_controller is None:
        _actuator_controller = ActuatorController(ros_bridge)
    elif ros_bridge is not None:
        _actuator_controller.set_ros_bridge(ros_bridge)
    
    return _actuator_controller