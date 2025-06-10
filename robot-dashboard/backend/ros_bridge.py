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
        try:
            # Try to create a new connection for each call to avoid broken pipe
            ws = create_connection(self.url, timeout=5)
            
            # Prepare service call message
            request = {
                "op": "call_service",
                "service": service_name,
                "args": args
            }
            
            if service_id:
                request["id"] = service_id
            
            # Send request
            ws.send(json.dumps(request))
            
            # Get response
            response = ws.recv()
            response_data = json.loads(response)
            
            # Close connection
            ws.close()
            
            logger.debug(f"Service call response: {response_data}")
            return True, response_data
            
        except Exception as e:
            logger.error(f"Service call failed: {e}")
            return False, {"error": str(e)}
    
    def publish_topic(self, topic_name: str, msg_type: str, msg_data: Dict[str, Any]) -> bool:
        """
        Publish a message to a ROS topic
        
        Args:
            topic_name: Name of the ROS topic (e.g., "/servo_position_cmd")
            msg_type: ROS message type (e.g., "servo_interfaces/msg/ServoPosition")
            msg_data: Message data as dictionary
        
        Returns:
            bool: True if publish successful, False otherwise
        """
        try:
            # Create a new connection for publishing
            ws = create_connection(self.url, timeout=5)
            
            # First advertise the topic
            advertise_msg = {
                "op": "advertise",
                "topic": topic_name,
                "type": msg_type
            }
            ws.send(json.dumps(advertise_msg))
            
            # Then publish the message
            publish_msg = {
                "op": "publish",
                "topic": topic_name,
                "msg": msg_data
            }
            ws.send(json.dumps(publish_msg))
            
            # Close connection
            ws.close()
            
            logger.debug(f"Published to {topic_name}: {msg_data}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish to topic {topic_name}: {e}")
            return False
    
    def subscribe_topic(self, topic_name: str, callback=None) -> bool:
        """
        Subscribe to a ROS topic
        
        Args:
            topic_name: Name of the ROS topic
            callback: Function to call when message received
        
        Returns:
            bool: True if subscription successful
        """
        try:
            if not self._connection:
                self.connect()
            
            # Subscribe message
            subscribe_msg = {
                "op": "subscribe",
                "topic": topic_name
            }
            self._connection.send(json.dumps(subscribe_msg))
            
            logger.info(f"Subscribed to topic: {topic_name}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to subscribe to topic {topic_name}: {e}")
            return False
    
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
        """
        Convert percentage (0-100) to PWM value (0-65535)
        
        For dual-PWM motor drivers, use NORMAL mapping:
        - 0% speed → PWM 0 (motor stopped)
        - 100% speed → PWM 65535 (motor full speed)
        
        Note: The previous INVERTED mapping was for different motor drivers
        """
        # Ensure percent is within bounds
        percent = max(0, min(100, percent))
        
        # NORMAL PWM calculation (not inverted)
        # 0% → 0, 100% → 65535
        pwm = int((percent / 100.0) * 65535)
        
        logger.debug(f"Converting {percent}% to PWM: {pwm}")
        return pwm
    
    @staticmethod
    def pwm_to_percent(pwm: int) -> float:
        """
        Convert PWM value (0-65535) to percentage (0-100)
        
        Using NORMAL mapping:
        - PWM 0 = 0% speed
        - PWM 65535 = 100% speed
        """
        # NORMAL calculation (not inverted)
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
        
        # Convert to PWM with NORMAL mapping
        if direction == "brake":
            # Brake always uses PWM 0
            speed_pwm = 0
        else:
            # Use normal PWM mapping: 0% = PWM 0, 100% = PWM 65535
            speed_pwm = self.percent_to_pwm(speed_percent)
        
        # Call ROS service (connection is handled in call_service)
        service_args = {
            "motor_id": motor_id,
            "direction": direction,
            "speed": speed_pwm
        }
        
        logger.info(f"Sending to ROS - Motor: {motor_id}, Direction: {direction}, Speed: {speed_percent}% (PWM: {speed_pwm})")
        
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
            # Brake with speed 0
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

class ServoPublisher:
    """High-level servo control interface using ROSBridge"""
    
    def __init__(self, ros_bridge: ROSBridgeConnection):
        """
        Initialize servo publisher
        
        Args:
            ros_bridge: ROSBridgeConnection instance
        """
        self.ros_bridge = ros_bridge
        self.topic_name = "/servo_position_cmd"
        self.msg_type = "servo_interfaces/msg/ServoPosition"
    
    def publish_position(self, pan_angle: float, tilt_angle: float) -> bool:
        """
        Publish servo position command
        
        Args:
            pan_angle: Pan servo angle (75-195 degrees)
            tilt_angle: Tilt servo angle (75-195 degrees)
        
        Returns:
            bool: True if successful
        """
        # Prepare message
        msg_data = {
            "pan_angle": pan_angle,
            "tilt_angle": tilt_angle
        }
        
        # Publish to topic
        success = self.ros_bridge.publish_topic(
            self.topic_name,
            self.msg_type,
            msg_data
        )
        
        if success:
            logger.info(f"Published servo position: pan={pan_angle}, tilt={tilt_angle}")
        else:
            logger.error(f"Failed to publish servo position")
        
        return success

# Create singleton instances for easy import
_ros_bridge = None
_motor_controller = None
_servo_publisher = None

def get_ros_bridge(host: str = None, port: int = None) -> ROSBridgeConnection:
    """Get or create ROSBridge connection instance"""
    global _ros_bridge
    
    # If host/port provided, update or create new connection
    if host is not None or port is not None:
        if _ros_bridge:
            # Update existing connection
            if host:
                _ros_bridge.host = host
                _ros_bridge.url = f"ws://{host}:{_ros_bridge.port}"
            if port:
                _ros_bridge.port = port
                _ros_bridge.url = f"ws://{_ros_bridge.host}:{port}"
        else:
            # Create new connection with provided settings
            _ros_bridge = ROSBridgeConnection(
                host or "192.168.2.4", 
                port or 9090
            )
    elif _ros_bridge is None:
        # Create with defaults if not exists
        _ros_bridge = ROSBridgeConnection()
    
    return _ros_bridge

def get_motor_controller() -> MotorController:
    """Get or create MotorController instance"""
    global _motor_controller
    if _motor_controller is None:
        _motor_controller = MotorController(get_ros_bridge())
    return _motor_controller

def get_servo_publisher() -> ServoPublisher:
    """Get or create ServoPublisher instance"""
    global _servo_publisher
    if _servo_publisher is None:
        _servo_publisher = ServoPublisher(get_ros_bridge())
    return _servo_publisher

# Convenience function for servo control to match the expected interface
async def publish_servo_position(pan_angle: float, tilt_angle: float) -> bool:
    """
    Publish servo position (async wrapper for compatibility)
    
    Args:
        pan_angle: Pan servo angle
        tilt_angle: Tilt servo angle
    
    Returns:
        bool: Success status
    """
    servo_pub = get_servo_publisher()
    return servo_pub.publish_position(pan_angle, tilt_angle)