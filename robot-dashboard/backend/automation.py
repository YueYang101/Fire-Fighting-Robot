#!/usr/bin/env python3
"""
Automation module for robot mapping and pathfinding
Implements wall-following algorithm with differential drive calculations
"""

import logging
import threading
import time
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
from collections import deque
import math

logger = logging.getLogger(__name__)

class RobotState(Enum):
    """Robot states for mapping"""
    IDLE = "idle"
    MAPPING = "mapping"
    TURNING = "turning"
    MOVING_FORWARD = "moving_forward"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    EMERGENCY_STOP = "emergency_stop"

class TurnDirection(Enum):
    """Turn direction preference"""
    LEFT = "left"
    RIGHT = "right"

class AutoMapper:
    """Autonomous mapping using wall-following algorithm"""
    
    def __init__(self, motor_controller, lidar_sensor):
        """
        Initialize the auto mapper
        
        Args:
            motor_controller: Motor controller instance
            lidar_sensor: Lidar sensor instance
        """
        self.motor_controller = motor_controller
        self.lidar_sensor = lidar_sensor
        
        # Robot dimensions (in mm)
        self.robot_length = 500  # mm
        self.robot_width = 450   # mm
        self.wheelbase = 400     # Approximate wheelbase (mm)
        
        # Safety parameters
        self.safety_distance = 800  # mm (stop if obstacle closer than this)
        self.wall_follow_distance = 1000  # mm (ideal distance from wall)
        self.wall_follow_tolerance = 200  # mm (acceptable deviation)
        
        # Speed parameters (0-100%)
        self.base_speed = 85  # Normal forward speed
        self.turn_speed = 75  # Speed during turns
        self.slow_speed = 75  # Slow speed for careful movement
        
        # Control parameters
        self.turn_direction = TurnDirection.RIGHT  # Prefer right turns
        self.command_delay = 0.2  # Delay between commands (seconds)
        self.sensor_delay = 0.1   # Account for sensor delay
        
        # State management
        self.state = RobotState.IDLE
        self.mapping_active = False
        self.emergency_stop = False
        self._mapping_thread = None
        self._state_lock = threading.Lock()
        
        # Odometry estimation (simple dead reckoning)
        self.robot_x = 0.0  # mm
        self.robot_y = 0.0  # mm
        self.robot_theta = 0.0  # radians
        self.last_odometry_update = time.time()
        
        # Mapping data
        self.visited_cells = set()  # Grid cells visited
        self.wall_map = {}  # Detected walls
        self.mapping_start_time = None
        
        # PID controller for wall following
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.2  # Derivative gain
        self.integral_error = 0.0
        self.last_error = 0.0
        
        logger.info("AutoMapper initialized")
    
    def start_mapping(self) -> bool:
        """Start autonomous mapping"""
        with self._state_lock:
            if self.mapping_active:
                logger.warning("Mapping already active")
                return False
            
            self.mapping_active = True
            self.emergency_stop = False
            self.state = RobotState.IDLE
            self.mapping_start_time = time.time()
            
            # Reset odometry
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_theta = 0.0
            
            # Clear previous mapping data
            self.visited_cells.clear()
            self.wall_map.clear()
            
            # Start mapping thread
            self._mapping_thread = threading.Thread(target=self._mapping_loop)
            self._mapping_thread.daemon = True
            self._mapping_thread.start()
            
            logger.info("Started autonomous mapping")
            return True
    
    def stop_mapping(self) -> bool:
        """Stop autonomous mapping"""
        with self._state_lock:
            if not self.mapping_active:
                return False
            
            self.mapping_active = False
            self.emergency_stop = True
            
        # Stop all motors
        self.motor_controller.stop_all_motors()
        
        # Wait for thread to finish
        if self._mapping_thread:
            self._mapping_thread.join(timeout=5.0)
        
        logger.info("Stopped autonomous mapping")
        return True
    
    def get_status(self) -> Dict[str, Any]:
        """Get current mapping status"""
        with self._state_lock:
            duration = 0
            if self.mapping_start_time:
                duration = time.time() - self.mapping_start_time
            
            return {
                "active": self.mapping_active,
                "state": self.state.value,
                "emergency_stop": self.emergency_stop,
                "robot_position": {
                    "x": round(self.robot_x),
                    "y": round(self.robot_y),
                    "theta": round(math.degrees(self.robot_theta), 1)
                },
                "visited_cells": len(self.visited_cells),
                "walls_detected": len(self.wall_map),
                "duration": round(duration, 1)
            }
    
    def _mapping_loop(self):
        """Main mapping loop"""
        logger.info("Mapping loop started")
        
        while self.mapping_active and not self.emergency_stop:
            try:
                # Get latest lidar scan
                scan_data = self.lidar_sensor.get_latest_scan()
                
                if not scan_data:
                    logger.warning("No lidar data available")
                    time.sleep(0.1)
                    continue
                
                # Process scan and decide action
                self._process_scan(scan_data)
                
                # Execute state-based behavior
                if self.state == RobotState.MOVING_FORWARD:
                    self._move_forward(scan_data)
                elif self.state == RobotState.TURNING:
                    self._execute_turn(scan_data)
                elif self.state == RobotState.OBSTACLE_AVOIDANCE:
                    self._avoid_obstacle(scan_data)
                elif self.state == RobotState.EMERGENCY_STOP:
                    self._emergency_stop()
                    break
                
                # Update odometry
                self._update_odometry()
                
                # Small delay to prevent CPU overload
                time.sleep(0.05)
                
            except Exception as e:
                logger.error(f"Error in mapping loop: {e}")
                self._emergency_stop()
                break
        
        # Ensure motors are stopped
        self.motor_controller.stop_all_motors()
        logger.info("Mapping loop ended")
    
    def _process_scan(self, scan_data: Dict[str, Any]):
        """Process lidar scan and determine robot state"""
        # Check for immediate obstacles
        min_distance = scan_data.get("min_distance", float('inf')) * 1000  # Convert to mm
        
        if min_distance < self.safety_distance:
            # Too close to obstacle
            if min_distance < self.safety_distance * 0.6:  # Very close
                self.state = RobotState.EMERGENCY_STOP
                return
            else:
                self.state = RobotState.OBSTACLE_AVOIDANCE
                return
        
        # Wall following logic
        right_wall_distance = self._get_wall_distance(scan_data, -90)  # Right side
        front_distance = self._get_wall_distance(scan_data, 0)  # Front
        
        # Check if we need to turn
        if front_distance < self.wall_follow_distance * 1.5:
            # Wall or obstacle ahead, need to turn
            self.state = RobotState.TURNING
        elif right_wall_distance is None or right_wall_distance > self.wall_follow_distance * 2:
            # No wall on right, turn right to find wall
            self.state = RobotState.TURNING
        else:
            # Follow wall
            self.state = RobotState.MOVING_FORWARD
    
    def _get_wall_distance(self, scan_data: Dict[str, Any], angle_deg: float) -> Optional[float]:
        """
        Get distance to wall at specific angle
        
        Args:
            scan_data: Lidar scan data
            angle_deg: Angle in degrees (0 = front, -90 = right, 90 = left)
            
        Returns:
            Distance in mm or None if no wall detected
        """
        points = scan_data.get("points", [])
        if not points:
            return None
        
        # Convert angle to radians
        target_angle = math.radians(angle_deg)
        angle_tolerance = math.radians(15)  # 15 degree tolerance
        
        # Find points within angle range
        valid_distances = []
        for point in points:
            angle_diff = abs(point["theta"] - target_angle)
            # Handle angle wrap-around
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff <= angle_tolerance:
                valid_distances.append(point["r"] * 1000)  # Convert to mm
        
        if valid_distances:
            return np.median(valid_distances)  # Use median for robustness
        return None
    
    def _move_forward(self, scan_data: Dict[str, Any]):
        """Move forward while following wall"""
        # Get distance to right wall
        right_wall_distance = self._get_wall_distance(scan_data, -90)
        
        if right_wall_distance is None:
            # No wall detected, move forward slowly
            self._set_motors("forward", self.slow_speed, "forward", self.slow_speed)
            return
        
        # Calculate error from desired wall distance
        error = right_wall_distance - self.wall_follow_distance
        
        # PID control
        self.integral_error += error * self.command_delay
        derivative = (error - self.last_error) / self.command_delay
        self.last_error = error
        
        # Calculate correction
        correction = (self.kp * error + 
                     self.ki * self.integral_error + 
                     self.kd * derivative)
        
        # Convert correction to differential speed
        # Positive correction = turn left (away from wall)
        # Negative correction = turn right (toward wall)
        speed_diff = max(-20, min(20, correction / 50))  # Limit differential
        
        # Note: Motor sides are reversed as mentioned
        # Left motors in code = Right side of robot
        # Right motors in code = Left side of robot
        left_speed = self.base_speed + speed_diff   # Right side of robot
        right_speed = self.base_speed - speed_diff  # Left side of robot
        
        # Ensure speeds are in valid range
        left_speed = max(self.slow_speed, min(100, left_speed))
        right_speed = max(self.slow_speed, min(100, right_speed))
        
        # Set motor speeds (reversed due to wiring)
        self._set_motors("forward", left_speed, "forward", right_speed)
    
    def _execute_turn(self, scan_data: Dict[str, Any]):
        """Execute a turn (preferably right)"""
        front_distance = self._get_wall_distance(scan_data, 0)
        right_distance = self._get_wall_distance(scan_data, -90)
        left_distance = self._get_wall_distance(scan_data, 90)
        
        # Determine turn direction
        if self.turn_direction == TurnDirection.RIGHT:
            # Check if right turn is possible
            if right_distance and right_distance > self.wall_follow_distance * 1.5:
                # Turn right - note motor reversal
                # Left motors forward, right motors backward for right turn
                self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
            elif left_distance and left_distance > self.wall_follow_distance * 1.5:
                # Can't turn right, turn left
                self._set_motors("backward", self.turn_speed, "forward", self.turn_speed)
            else:
                # Can't turn either way, back up
                self._set_motors("backward", self.slow_speed, "backward", self.slow_speed)
        
        # Wait a bit for turn to execute
        time.sleep(0.5)
        
        # Check if turn is complete
        new_front = self._get_wall_distance(scan_data, 0)
        if new_front and new_front > self.wall_follow_distance * 1.5:
            self.state = RobotState.MOVING_FORWARD
    
    def _avoid_obstacle(self, scan_data: Dict[str, Any]):
        """Avoid immediate obstacle"""
        # Stop first
        self.motor_controller.stop_all_motors()
        time.sleep(0.2)
        
        # Determine escape direction
        left_clear = self._check_direction_clear(scan_data, 90)
        right_clear = self._check_direction_clear(scan_data, -90)
        
        if right_clear and (not left_clear or self.turn_direction == TurnDirection.RIGHT):
            # Turn right
            self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
        elif left_clear:
            # Turn left
            self._set_motors("backward", self.turn_speed, "forward", self.turn_speed)
        else:
            # Back up
            self._set_motors("backward", self.slow_speed, "backward", self.slow_speed)
        
        time.sleep(0.5)
        self.state = RobotState.MOVING_FORWARD
    
    def _check_direction_clear(self, scan_data: Dict[str, Any], angle_deg: float) -> bool:
        """Check if direction is clear for movement"""
        distance = self._get_wall_distance(scan_data, angle_deg)
        return distance is None or distance > self.safety_distance
    
    def _set_motors(self, left_dir: str, left_speed: float, 
                    right_dir: str, right_speed: float):
        """
        Set motor speeds accounting for reversal
        Note: Physical left/right are reversed in motor controller
        """
        # Motor 0 in code = Right side of robot
        # Motor 1 in code = Left side of robot
        
        # Set right side of robot (motor 0)
        self.motor_controller.set_motor(0, left_dir, left_speed)
        time.sleep(self.command_delay)
        
        # Set left side of robot (motor 1)
        self.motor_controller.set_motor(1, right_dir, right_speed)
    
    def _update_odometry(self):
        """Update robot position estimate based on motor commands"""
        # Simple dead reckoning - this is approximate
        current_time = time.time()
        dt = current_time - self.last_odometry_update
        self.last_odometry_update = current_time
        
        # Get current motor states
        motor_states = self.motor_controller.get_all_motor_states()
        
        # Calculate wheel velocities (mm/s) - approximate
        left_velocity = 0
        right_velocity = 0
        
        if motor_states[1]["direction"] == "forward":
            left_velocity = motor_states[1]["speed"] * 10  # mm/s at 100% speed
        elif motor_states[1]["direction"] == "backward":
            left_velocity = -motor_states[1]["speed"] * 10
        
        if motor_states[0]["direction"] == "forward":
            right_velocity = motor_states[0]["speed"] * 10
        elif motor_states[0]["direction"] == "backward":
            right_velocity = -motor_states[0]["speed"] * 10
        
        # Calculate robot velocity and angular velocity
        v = (left_velocity + right_velocity) / 2  # Linear velocity
        w = (right_velocity - left_velocity) / self.wheelbase  # Angular velocity
        
        # Update position
        if abs(w) < 0.001:  # Moving straight
            self.robot_x += v * dt * math.cos(self.robot_theta)
            self.robot_y += v * dt * math.sin(self.robot_theta)
        else:  # Turning
            # Use exact motion equations for differential drive
            self.robot_x += (v / w) * (math.sin(self.robot_theta + w * dt) - 
                                       math.sin(self.robot_theta))
            self.robot_y += (v / w) * (-math.cos(self.robot_theta + w * dt) + 
                                       math.cos(self.robot_theta))
            self.robot_theta += w * dt
        
        # Normalize angle
        self.robot_theta = self.robot_theta % (2 * math.pi)
        
        # Update visited cells (10cm grid)
        grid_x = int(self.robot_x / 100)
        grid_y = int(self.robot_y / 100)
        self.visited_cells.add((grid_x, grid_y))
    
    def _emergency_stop(self):
        """Execute emergency stop"""
        logger.warning("Emergency stop activated!")
        self.motor_controller.stop_all_motors()
        self.state = RobotState.EMERGENCY_STOP
        self.emergency_stop = True

# Singleton instance
_auto_mapper = None

def get_auto_mapper(motor_controller=None, lidar_sensor=None):
    """Get or create AutoMapper instance"""
    global _auto_mapper
    if _auto_mapper is None and motor_controller and lidar_sensor:
        _auto_mapper = AutoMapper(motor_controller, lidar_sensor)
    return _auto_mapper