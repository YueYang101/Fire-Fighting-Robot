#!/usr/bin/env python3
"""
ROS Bridge WebSocket communication module
Handles all ROS 2 communication via rosbridge
"""

import json
import logging
from websocket import create_connection
from typing import Dict, Any, Optional, Tuple

# Setup logging
logger = logging.getLogger(__name__)

class ROSBridgeConnection:
    """Manages WebSocket connection to ROS 2 via rosbridge"""
    
    def __init__(self, host: str = "192.168.2.4", port: int = 9090):
        """
        Initialize ROS Bridge connection parameters
        
        Args:
            host: IP address of the ROS 2 robot
            port: rosbridge websocket port (default: 9090)
        """
        self.host = host
        self.port = port
        self.url = f"ws://{host}:{port}"
        self._connection = None
    
    def connect(self) -> bool:
        """
        Establish connection to rosbridge
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self._connection = create_connection(self.url, timeout=5)
            logger.info(f"Connected to rosbridge at {self.url}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to rosbridge: {e}")
            return False
    
    def disconnect(self):
        """Close the websocket connection"""
        if self._connection:
            self._connection.close()
            self._connection = None
            logger.info("Disconnected from rosbridge")
    
    def is_connected(self) -> bool:
        """Check if connection is active"""
        return self._connection is not None
    
    def call_service(self, service_name: str, args: Dict[str, Any], 
                    service_id: Optional[str] = None) -> Tuple[bool, Dict[str, Any]]:
        """
        Call a ROS service through rosbridge
        
        Args:
            service_name: Name of the ROS service (e.g., "/set_motor")
            args: Service arguments as dictionary
            service_id: Optional ID for the service call
        
        Returns:
            Tuple[bool, Dict]: (success, response_data)
        """
        if not self._connection:
            return False, {"error": "Not connected to rosbridge"}
        
        try:
            # Prepare service call message
            request = {
                "op": "call_service",
                "service": service_name,
                "args": args
            }
            
            if service_id:
                request["id"] = service_id
            
            # Send request
            self._connection.send(json.dumps(request))
            
            # Get response
            response = self._connection.recv()
            response_data = json.loads(response)
            
            logger.debug(f"Service call response: {response_data}")
            return True, response_data
            
        except Exception as e:
            logger.error(f"Service call failed: {e}")
            return False, {"error": str(e)}
    
    def test_connection(self) -> bool:
        """
        Test if rosbridge connection is working
        
        Returns:
            bool: True if connection is working
        """
        try:
            temp_conn = create_connection(self.url, timeout=2)
            temp_conn.close()
            return True
        except Exception:
            return False

class MotorController:
    """High-level motor control interface using ROSBridge"""
    
    def __init__(self, ros_bridge: ROSBridgeConnection):
        """
        Initialize motor controller
        
        Args:
            ros_bridge: ROSBridgeConnection instance
        """
        self.ros_bridge = ros_bridge
        self.motor_states = {
            0: {"direction": "brake", "speed_percent": 0, "speed_pwm": 0},
            1: {"direction": "brake", "speed_percent": 0, "speed_pwm": 0},
            2: {"direction": "brake", "speed_percent": 0, "speed_pwm": 0},
            3: {"direction": "brake", "speed_percent": 0, "speed_pwm": 0}
        }
    
    @staticmethod
    def percent_to_pwm(percent: float) -> int:
        """Convert percentage (0-100) to PWM value (0-65535)"""
        return int((percent / 100.0) * 65535)
    
    @staticmethod
    def pwm_to_percent(pwm: int) -> float:
        """Convert PWM value (0-65535) to percentage (0-100)"""
        return round((pwm / 65535.0) * 100, 1)
    
    def set_motor(self, motor_id: int, direction: str, speed_percent: float) -> Dict[str, Any]:
        """
        Set motor speed and direction
        
        Args:
            motor_id: Motor ID (0-3)
            direction: Direction ("forward", "backward", "brake")
            speed_percent: Speed in percentage (0-100)
        
        Returns:
            Dict containing operation result
        """
        # Validate inputs
        if motor_id < 0 or motor_id > 3:
            return {
                "success": False,
                "error": "Invalid motor ID. Must be 0-3"
            }
        
        if direction not in ["forward", "backward", "brake"]:
            return {
                "success": False,
                "error": "Invalid direction. Must be 'forward', 'backward', or 'brake'"
            }
        
        if not isinstance(speed_percent, (int, float)) or speed_percent < 0 or speed_percent > 100:
            return {
                "success": False,
                "error": "Invalid speed. Must be 0-100"
            }
        
        # Convert to PWM
        speed_pwm = self.percent_to_pwm(speed_percent) if direction != "brake" else 0
        
        # Ensure connection
        if not self.ros_bridge.is_connected():
            if not self.ros_bridge.connect():
                return {
                    "success": False,
                    "error": "Failed to connect to rosbridge"
                }
        
        # Call ROS service
        service_args = {
            "motor_id": motor_id,
            "direction": direction,
            "speed": speed_pwm
        }
        
        success, response = self.ros_bridge.call_service(
            "/set_motor", 
            service_args,
            f"motor_cmd_{motor_id}"
        )
        
        if success:
            # Update internal state
            self.motor_states[motor_id] = {
                "direction": direction,
                "speed_percent": speed_percent,
                "speed_pwm": speed_pwm
            }
            
            logger.info(f"Motor {motor_id} set to {direction} at {speed_percent}% (PWM: {speed_pwm})")
            
            return {
                "success": True,
                "motor_id": motor_id,
                "direction": direction,
                "speed_percent": speed_percent,
                "speed_pwm": speed_pwm,
                "response": response
            }
        else:
            return {
                "success": False,
                "error": response.get("error", "Unknown error"),
                "motor_id": motor_id
            }
    
    def stop_all_motors(self) -> Dict[str, Any]:
        """
        Emergency stop - brake all motors
        
        Returns:
            Dict containing results for all motors
        """
        results = []
        
        for motor_id in range(4):
            result = self.set_motor(motor_id, "brake", 0)
            results.append(result)
        
        all_success = all(r["success"] for r in results)
        
        return {
            "success": all_success,
            "results": results
        }
    
    def get_motor_state(self, motor_id: int) -> Optional[Dict[str, Any]]:
        """
        Get current state of a motor
        
        Args:
            motor_id: Motor ID (0-3)
        
        Returns:
            Dict with motor state or None if invalid ID
        """
        if motor_id in self.motor_states:
            return self.motor_states[motor_id].copy()
        return None
    
    def get_all_motor_states(self) -> Dict[int, Dict[str, Any]]:
        """Get current state of all motors"""
        return self.motor_states.copy()

# Create singleton instances for easy import
_ros_bridge = None
_motor_controller = None

def get_ros_bridge(host: str = "192.168.2.4", port: int = 9090) -> ROSBridgeConnection:
    """Get or create ROSBridge connection instance"""
    global _ros_bridge
    if _ros_bridge is None:
        _ros_bridge = ROSBridgeConnection(host, port)
    return _ros_bridge

def get_motor_controller() -> MotorController:
    """Get or create MotorController instance"""
    global _motor_controller
    if _motor_controller is None:
        _motor_controller = MotorController(get_ros_bridge())
    return _motor_controller