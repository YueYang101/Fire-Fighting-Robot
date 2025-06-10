#!/usr/bin/env python3
"""
Servo control module for the robot dashboard
Handles communication with ROS2 servo controller node
"""

import asyncio
import json
import time
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

class ServoController:
    def __init__(self, ros_bridge=None):
        self.ros_bridge = ros_bridge
        self.websocket_clients = set()
        
        # Current servo state
        self.current_state = {
            'pan_angle': 135.0,
            'tilt_angle': 135.0,
            'pan_moving': False,
            'tilt_moving': False,
            'last_update': None
        }
        
        # Safety constraints
        self.MIN_ANGLE = 75.0   # 135 - 60
        self.MAX_ANGLE = 195.0  # 135 + 60
        self.CENTER_ANGLE = 135.0
        
        # Rate limiting
        self.last_command_time = 0
        self.command_interval = 0.1  # 10Hz max command rate
        
        # Subscribe to servo state if ROS bridge is available
        if self.ros_bridge:
            self.setup_ros_subscriptions()
            logger.info("Servo controller initialized with ROS bridge")
        else:
            logger.warning("Servo controller initialized without ROS bridge")
    
    def setup_ros_subscriptions(self):
        """Setup ROS topic subscriptions"""
        # Subscribe to servo state
        self.ros_bridge.create_subscription(
            'servo_interfaces/msg/ServoState',
            '/servo_state',
            self.servo_state_callback,
            10
        )
        
        # Create publisher for servo commands
        self.servo_cmd_pub = self.ros_bridge.create_publisher(
            'servo_interfaces/msg/ServoPosition',
            '/servo_position_cmd',
            10
        )
        
        logger.info("ROS servo subscriptions setup complete")
    
    def servo_state_callback(self, msg):
        """Handle incoming servo state messages"""
        try:
            # Update current state
            self.current_state['pan_angle'] = msg.pan_angle
            self.current_state['tilt_angle'] = msg.tilt_angle
            self.current_state['pan_moving'] = msg.pan_moving
            self.current_state['tilt_moving'] = msg.tilt_moving
            self.current_state['last_update'] = datetime.now().isoformat()
            
            # Broadcast to all WebSocket clients
            asyncio.create_task(self.broadcast_servo_state())
            
        except Exception as e:
            logger.error(f"Error in servo state callback: {e}")
    
    async def broadcast_servo_state(self):
        """Broadcast current servo state to all connected clients"""
        if not self.websocket_clients:
            return
        
        message = json.dumps({
            'type': 'servo_state',
            'pan_angle': self.current_state['pan_angle'],
            'tilt_angle': self.current_state['tilt_angle'],
            'pan_moving': self.current_state['pan_moving'],
            'tilt_moving': self.current_state['tilt_moving'],
            'timestamp': self.current_state['last_update']
        })
        
        # Send to all connected clients
        disconnected = set()
        for client in self.websocket_clients:
            try:
                await client.send(message)
            except Exception as e:
                logger.error(f"Error sending to client: {e}")
                disconnected.add(client)
        
        # Remove disconnected clients
        self.websocket_clients -= disconnected
    
    def validate_angles(self, pan_angle, tilt_angle):
        """Validate servo angles are within safe limits"""
        # Clamp angles to safe range
        pan_safe = max(self.MIN_ANGLE, min(self.MAX_ANGLE, pan_angle))
        tilt_safe = max(self.MIN_ANGLE, min(self.MAX_ANGLE, tilt_angle))
        
        # Warn if clamping occurred
        if pan_safe != pan_angle:
            logger.warning(f"Pan angle {pan_angle} clamped to {pan_safe}")
        if tilt_safe != tilt_angle:
            logger.warning(f"Tilt angle {tilt_angle} clamped to {tilt_safe}")
        
        return pan_safe, tilt_safe
    
    def send_servo_command(self, pan_angle, tilt_angle):
        """Send servo position command via ROS"""
        # Rate limiting
        current_time = time.time()
        if current_time - self.last_command_time < self.command_interval:
            return False
        
        # Validate angles
        pan_safe, tilt_safe = self.validate_angles(pan_angle, tilt_angle)
        
        if self.ros_bridge and hasattr(self, 'servo_cmd_pub'):
            try:
                # Create and publish message
                msg = self.ros_bridge.create_message('servo_interfaces/msg/ServoPosition')
                msg.pan_angle = float(pan_safe)
                msg.tilt_angle = float(tilt_safe)
                
                self.servo_cmd_pub.publish(msg)
                self.last_command_time = current_time
                
                logger.debug(f"Sent servo command: pan={pan_safe}, tilt={tilt_safe}")
                return True
                
            except Exception as e:
                logger.error(f"Error sending servo command: {e}")
                return False
        else:
            logger.warning("Cannot send servo command - ROS bridge not available")
            return False
    
    def center_servos(self):
        """Move servos to center position"""
        return self.send_servo_command(self.CENTER_ANGLE, self.CENTER_ANGLE)
    
    async def handle_websocket_message(self, websocket, message):
        """Handle incoming WebSocket messages"""
        try:
            data = json.loads(message)
            
            if data.get('type') == 'servo_command':
                action = data.get('action')
                
                if action == 'move':
                    # Handle move command
                    pan = data.get('pan_angle', self.current_state['pan_angle'])
                    tilt = data.get('tilt_angle', self.current_state['tilt_angle'])
                    self.send_servo_command(pan, tilt)
                    
                elif action == 'center':
                    # Center servos
                    self.center_servos()
                    
                elif action == 'get_state':
                    # Send current state
                    await websocket.send(json.dumps({
                        'type': 'servo_state',
                        'pan_angle': self.current_state['pan_angle'],
                        'tilt_angle': self.current_state['tilt_angle'],
                        'pan_moving': self.current_state['pan_moving'],
                        'tilt_moving': self.current_state['tilt_moving'],
                        'timestamp': self.current_state['last_update']
                    }))
                    
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON received: {message}")
        except Exception as e:
            logger.error(f"Error handling WebSocket message: {e}")
    
    def add_websocket_client(self, websocket):
        """Add a new WebSocket client"""
        self.websocket_clients.add(websocket)
        logger.info(f"WebSocket client added. Total clients: {len(self.websocket_clients)}")
    
    def remove_websocket_client(self, websocket):
        """Remove a WebSocket client"""
        self.websocket_clients.discard(websocket)
        logger.info(f"WebSocket client removed. Total clients: {len(self.websocket_clients)}")
    
    def get_status(self):
        """Get current servo controller status"""
        return {
            'connected': self.ros_bridge is not None,
            'current_state': self.current_state,
            'limits': {
                'min_angle': self.MIN_ANGLE,
                'max_angle': self.MAX_ANGLE,
                'center_angle': self.CENTER_ANGLE
            },
            'websocket_clients': len(self.websocket_clients)
        }