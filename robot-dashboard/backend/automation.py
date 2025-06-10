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
        
        # Sensor offset - lidar is at front center
        self.lidar_offset_front = self.robot_length / 2  # 250mm from center to front
        self.lidar_offset_rear = self.robot_length / 2   # 250mm from center to rear
        self.lidar_offset_side = self.robot_width / 2    # 225mm from center to side
        
        # Safety parameters (distances from robot body, not sensor)
        self.safety_distance = 300  # mm (stop if obstacle closer than this from body)
        self.wall_follow_distance = 500  # mm (ideal distance from wall to body)
        self.wall_follow_tolerance = 150  # mm (acceptable deviation)
        self.critical_distance = 200  # mm (emergency stop distance from body)
        
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
        self.idle_timeout = 5.0  # Seconds before forcing movement in IDLE state
        self.last_state_change = time.time()
        
        # Odometry estimation (simple dead reckoning)
        self.robot_x = 0.0  # mm
        self.robot_y = 0.0  # mm
        self.robot_theta = 0.0  # radians
        self.last_odometry_update = time.time()
        
        # Mapping data
        self.visited_cells = set()  # Grid cells visited
        self.wall_map = {}  # Detected walls
        self.mapping_start_time = None
        
        # Lidar background removal
        self.static_scan_buffer = []
        self.static_scan_samples = 10
        self.static_obstacles = {}  # Store static obstacles like robot components
        self.background_removal_enabled = True
        
        # PID controller for wall following
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.2  # Derivative gain
        self.integral_error = 0.0
        self.last_error = 0.0
        
        # Bug2 algorithm parameters
        self.m_line = None  # Line from start to goal
        self.hit_points = []  # Points where robot hit obstacles
        self.leave_points = []  # Points where robot left obstacles
        
        logger.info("AutoMapper initialized")
    
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
    
    def start_mapping(self) -> bool:
        """Start autonomous mapping"""
        with self._state_lock:
            if self.mapping_active:
                logger.warning("Mapping already active")
                return False
            
            # Calibrate background first
            if not self.calibrate_background():
                logger.warning("Background calibration failed, continuing without it")
            
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
            self.hit_points.clear()
            self.leave_points.clear()
            
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
        
        # Initial delay to ensure sensors are ready
        logger.info("Waiting for sensors to stabilize...")
        time.sleep(2.0)
        
        # Start in IDLE state and wait for good data
        startup_attempts = 0
        while self.mapping_active and not self.emergency_stop and startup_attempts < 10:
            scan_data = self.lidar_sensor.get_latest_scan()
            if scan_data and scan_data.get("point_count", 0) > 10:
                logger.info(f"Good lidar data received, starting mapping. Points: {scan_data.get('point_count')}")
                # Start by moving forward to find walls
                self.state = RobotState.MOVING_FORWARD
                break
            startup_attempts += 1
            time.sleep(0.5)
        
        # If still no data, start moving forward anyway to explore
        if self.state == RobotState.IDLE:
            logger.info("No initial walls detected, starting exploration by moving forward")
            self.state = RobotState.MOVING_FORWARD
        
        while self.mapping_active and not self.emergency_stop:
            try:
                # Get latest lidar scan
                scan_data = self.lidar_sensor.get_latest_scan()
                
                if not scan_data:
                    logger.warning("No lidar data available")
                    time.sleep(0.1)
                    continue
                
                # Filter out static obstacles (robot components)
                filtered_scan = self.filter_static_obstacles(scan_data)
                
                # Process scan and decide action
                self._process_scan(filtered_scan)
                
                # Execute state-based behavior
                current_time = time.time()
                
                # Check for idle timeout
                if self.state == RobotState.IDLE:
                    if current_time - self.last_state_change > self.idle_timeout:
                        logger.warning("IDLE timeout - forcing movement to explore")
                        self.state = RobotState.MOVING_FORWARD
                        self.last_state_change = current_time
                
                # Track state changes
                old_state = self.state
                
                if self.state == RobotState.MOVING_FORWARD:
                    self._move_forward(scan_data)
                elif self.state == RobotState.TURNING:
                    self._execute_turn(scan_data)
                elif self.state == RobotState.OBSTACLE_AVOIDANCE:
                    self._avoid_obstacle(scan_data)
                elif self.state == RobotState.EMERGENCY_STOP:
                    self._emergency_stop()
                    if self.emergency_stop:  # If still in emergency
                        break
                
                # Update state change time
                if old_state != self.state:
                    self.last_state_change = current_time
                
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
    
    def update_safety_parameters(self, critical: float, safety: float, wall_follow: float) -> bool:
        """Update safety parameters dynamically
        
        Args:
            critical: Critical distance in mm
            safety: Safety distance in mm
            wall_follow: Wall following distance in mm
            
        Returns:
            bool: True if successful
        """
        try:
            self.critical_distance = max(100, min(500, critical))
            self.safety_distance = max(200, min(800, safety))
            self.wall_follow_distance = max(300, min(1000, wall_follow))
            logger.info(f"Updated safety parameters - Critical: {self.critical_distance}mm, Safety: {self.safety_distance}mm, Wall follow: {self.wall_follow_distance}mm")
            return True
        except Exception as e:
            logger.error(f"Failed to update safety parameters: {e}")
            return False
    
    def _process_scan(self, scan_data: Dict[str, Any]):
        """Process lidar scan and determine robot state"""
        # Check for immediate obstacles
        min_distance = scan_data.get("min_distance", float('inf')) * 1000  # Convert to mm
        
        # Since lidar is at front center, adjust for robot body
        # For front obstacles, the actual clearance is min_distance
        # For side obstacles, we need to check specific angles
        front_clearance = min_distance  # Direct measurement from front sensor
        
        # Log for debugging
        logger.info(f"Front clearance: {front_clearance}mm, Critical+offset: {self.critical_distance}mm")
        
        if front_clearance < self.critical_distance:
            # Too close to obstacle - emergency stop
            logger.warning(f"CRITICAL: Obstacle at {front_clearance}mm from body - Emergency stop!")
            self.state = RobotState.EMERGENCY_STOP
            return
        elif front_clearance < self.safety_distance:
            # Close to obstacle but not critical
            logger.info(f"Obstacle detected at {front_clearance}mm from body - Avoiding")
            self.state = RobotState.OBSTACLE_AVOIDANCE
            return
        
        # Wall following logic - measure to robot body, not sensor
        right_wall_distance = self._get_wall_distance(scan_data, -90)  # Right side
        front_distance = self._get_wall_distance(scan_data, 0)  # Front
        
        # Adjust side distance for robot width (sensor is at center)
        if right_wall_distance is not None:
            right_wall_distance = max(0, right_wall_distance - self.lidar_offset_side)
        
        logger.debug(f"Wall distances from body - Right: {right_wall_distance}mm, Front: {front_distance}mm")
        
        # Check if we need to turn
        if front_distance and front_distance < self.wall_follow_distance:
            # Wall or obstacle ahead, need to turn
            logger.info(f"Wall ahead at {front_distance}mm from body - Turning")
            self.state = RobotState.TURNING
        elif right_wall_distance is None or right_wall_distance > self.wall_follow_distance * 2:
            # No wall on right, but keep moving forward to find one
            logger.info(f"No right wall detected - Continue moving forward to explore")
            self.state = RobotState.MOVING_FORWARD
        else:
            # Follow wall
            self.state = RobotState.MOVING_FORWARD
    
    def reset_emergency_stop(self) -> bool:
        """Reset emergency stop state"""
        with self._state_lock:
            if self.emergency_stop:
                self.emergency_stop = False
                self.state = RobotState.IDLE
                logger.info("Emergency stop reset")
                return True
            return False
    
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
            # Handle both 'theta' and 'angle' keys
            point_angle = point.get("theta", point.get("angle", 0))
            angle_diff = abs(point_angle - target_angle)
            # Handle angle wrap-around
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff <= angle_tolerance:
                # Handle both 'r' and 'distance' keys
                distance = point.get("r", point.get("distance", 0))
                if distance > 0.1 and distance < 10.0:  # Valid range 0.1m to 10m
                    valid_distances.append(distance * 1000)  # Convert to mm
        
        if valid_distances:
            return np.median(valid_distances)  # Use median for robustness
        return None
    
    def _move_forward(self, scan_data: Dict[str, Any]):
        """Move forward while following wall or exploring"""
        # Get distance to right wall
        right_wall_distance = self._get_wall_distance(scan_data, -90)
        
        if right_wall_distance is None:
            # No wall detected, move forward to explore
            logger.info("No wall detected - Exploring forward")
            self._set_motors("forward", self.base_speed, "forward", self.base_speed)
            return
        
        # Adjust for sensor position - sensor is at center, we want distance from body
        actual_wall_distance = right_wall_distance - self.lidar_offset_side
        
        # If wall is too far, just move forward
        if actual_wall_distance > self.wall_follow_distance * 2:
            logger.info(f"Wall too far ({actual_wall_distance}mm) - Moving forward to explore")
            self._set_motors("forward", self.base_speed, "forward", self.base_speed)
            return
        
        # Wall following mode - use PID control
        # Calculate error from desired wall distance
        error = actual_wall_distance - self.wall_follow_distance
        
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
        
        logger.debug(f"Wall following - Distance: {actual_wall_distance}mm, Error: {error}mm, Speeds: L={left_speed}, R={right_speed}")
        
        # Set motor speeds (reversed due to wiring)
        self._set_motors("forward", left_speed, "forward", right_speed)
    
    def _execute_turn(self, scan_data: Dict[str, Any]):
        """Execute a turn using smooth single-wheel turning"""
        front_distance = self._get_wall_distance(scan_data, 0)
        right_distance = self._get_wall_distance(scan_data, -90)
        left_distance = self._get_wall_distance(scan_data, 90)
        
        # Determine turn direction
        if self.turn_direction == TurnDirection.RIGHT:
            # Check if right turn is possible
            if right_distance and right_distance > self.wall_follow_distance * 1.5:
                # Turn right using smooth turn
                logger.info("Executing smooth right turn")
                self._set_motors_smooth("forward", self.turn_speed, "backward", self.turn_speed)
            elif left_distance and left_distance > self.wall_follow_distance * 1.5:
                # Can't turn right, turn left
                logger.info("Right blocked, executing smooth left turn")
                self._set_motors_smooth("backward", self.turn_speed, "forward", self.turn_speed)
            else:
                # Can't turn either way, rotate in place
                logger.warning("Both sides blocked, rotating in place")
                self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
        
        # Wait a bit for turn to execute
        time.sleep(0.5)
        
        # Check if turn is complete
        new_front = self._get_wall_distance(scan_data, 0)
        if new_front and new_front > self.wall_follow_distance * 1.5:
            self.state = RobotState.MOVING_FORWARD
    
    def _avoid_obstacle(self, scan_data: Dict[str, Any]):
        """Avoid immediate obstacle using smooth turning"""
        # Stop first
        self.motor_controller.stop_all_motors()
        time.sleep(0.2)
        
        # Since we can only see forward 180°, we can't safely back up
        # Determine escape direction based on what we can see
        left_clear = self._check_direction_clear(scan_data, 90)
        right_clear = self._check_direction_clear(scan_data, -90)
        
        if right_clear and (not left_clear or self.turn_direction == TurnDirection.RIGHT):
            # Turn right smoothly
            logger.info("Obstacle avoidance - Smooth right turn")
            self._set_motors_smooth("forward", self.turn_speed, "backward", self.turn_speed)
        elif left_clear:
            # Turn left smoothly
            logger.info("Obstacle avoidance - Smooth left turn")
            self._set_motors_smooth("backward", self.turn_speed, "forward", self.turn_speed)
        else:
            # Both sides blocked, turn in place to find opening
            logger.warning("Both sides blocked - Rotating in place")
            # Use full differential for tight spot
            self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
        
        time.sleep(0.5)
        self.state = RobotState.MOVING_FORWARD
    
    def _check_direction_clear(self, scan_data: Dict[str, Any], angle_deg: float) -> bool:
        """Check if direction is clear for movement"""
        distance = self._get_wall_distance(scan_data, angle_deg)
        return distance is None or distance > self.safety_distance
    
    def calibrate_background(self) -> bool:
        """Calibrate static background/robot components to remove from scans"""
        logger.info("Calibrating lidar background...")
        
        # Collect multiple scans while stationary
        scan_samples = []
        for i in range(self.static_scan_samples):
            scan_data = self.lidar_sensor.get_latest_scan()
            if scan_data and scan_data.get("points"):
                scan_samples.append(scan_data["points"])
                time.sleep(0.1)
        
        if len(scan_samples) < 5:
            logger.warning("Not enough samples for background calibration")
            return False
        
        # Find persistent obstacles (robot components)
        # These appear in the same location across all scans
        angle_bins = {}  # Group by angle
        
        for scan in scan_samples:
            for point in scan:
                angle = point.get("theta", point.get("angle", 0))
                angle_deg = int(math.degrees(angle))
                if angle_deg not in angle_bins:
                    angle_bins[angle_deg] = []
                distance = point.get("r", point.get("distance", 0))
                angle_bins[angle_deg].append(distance)
        
        # Find static obstacles
        self.static_obstacles = {}
        for angle_deg, distances in angle_bins.items():
            if len(distances) >= self.static_scan_samples * 0.8:  # Present in 80% of scans
                # Calculate variance
                mean_dist = np.mean(distances)
                variance = np.var(distances)
                
                # Low variance means static obstacle
                if variance < 0.01:  # 1cm variance threshold
                    self.static_obstacles[angle_deg] = {
                        "distance": mean_dist,
                        "variance": variance
                    }
                    logger.debug(f"Static obstacle at {angle_deg}°: {mean_dist:.2f}m")
        
        logger.info(f"Background calibration complete. Found {len(self.static_obstacles)} static points")
        return True
    
    def filter_static_obstacles(self, scan_data: Dict[str, Any]) -> Dict[str, Any]:
        """Remove static obstacles from scan data"""
        if not self.background_removal_enabled or not self.static_obstacles:
            return scan_data
        
        filtered_points = []
        removed_count = 0
        
        for point in scan_data.get("points", []):
            angle = point.get("theta", point.get("angle", 0))
            angle_deg = int(math.degrees(angle))
            distance = point.get("r", point.get("distance", 0))
            
            # Check if this matches a static obstacle
            is_static = False
            for static_angle in range(angle_deg - 2, angle_deg + 3):  # ±2 degree tolerance
                if static_angle in self.static_obstacles:
                    static_dist = self.static_obstacles[static_angle]["distance"]
                    if abs(distance - static_dist) < 0.05:  # 5cm tolerance
                        is_static = True
                        removed_count += 1
                        break
            
            if not is_static:
                filtered_points.append(point)
        
        # Update scan data
        filtered_data = scan_data.copy()
        filtered_data["points"] = filtered_points
        filtered_data["point_count"] = len(filtered_points)
        
        if removed_count > 0:
            logger.debug(f"Removed {removed_count} static points from scan")
        
        # Recalculate min distance
        if filtered_points:
            min_dist = min(p.get("r", p.get("distance", float('inf'))) for p in filtered_points)
            filtered_data["min_distance"] = min_dist
        
        return filtered_data
    
    def _set_motors_smooth(self, left_dir: str, left_speed: float, 
                          right_dir: str, right_speed: float):
        """
        Set motor speeds with smoother turning (avoid reverse direction)
        Note: Physical left/right are reversed in motor controller
        """
        # For turning, use one wheel stopped instead of reverse
        if left_dir == "forward" and right_dir == "backward":
            # Right turn - stop right wheel instead of reverse
            logger.debug("Right turn - stopping right wheel")
            self._set_motors("forward", left_speed, "brake", 0)
        elif left_dir == "backward" and right_dir == "forward":
            # Left turn - stop left wheel instead of reverse
            logger.debug("Left turn - stopping left wheel")
            self._set_motors("brake", 0, "forward", right_speed)
        else:
            # Normal operation
            self._set_motors(left_dir, left_speed, right_dir, right_speed)
    
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
        """Execute emergency stop with smooth recovery"""
        logger.warning("Emergency stop activated!")
        self.motor_controller.stop_all_motors()
        self.state = RobotState.EMERGENCY_STOP
        
        # Don't back up blindly - we can't see behind
        time.sleep(1.0)
        
        # Try to turn in place to find a clear path
        logger.info("Attempting recovery - smooth rotation to find clear path")
        
        # Use smooth turn first
        self._set_motors_smooth("forward", self.turn_speed, "backward", self.turn_speed)
        time.sleep(1.5)  # Turn about 90 degrees
        self.motor_controller.stop_all_motors()
        
        # Check if we have a clear path now
        scan_data = self.lidar_sensor.get_latest_scan()
        if scan_data:
            filtered_scan = self.filter_static_obstacles(scan_data)
            min_distance = filtered_scan.get("min_distance", 0) * 1000
            if min_distance > self.safety_distance:
                # Path is clear, resume
                if self.mapping_active:
                    self.state = RobotState.MOVING_FORWARD
                    self.emergency_stop = False
                    logger.info("Recovery complete, clear path found")
            else:
                # Still blocked, try full differential turn
                logger.warning("Still blocked, trying full rotation")
                self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
                time.sleep(1.5)
                self.motor_controller.stop_all_motors()
                
                # Try one more time
                if self.mapping_active:
                    self.state = RobotState.IDLE
                    self.emergency_stop = False

# Singleton instance
_auto_mapper = None

def get_auto_mapper(motor_controller=None, lidar_sensor=None):
    """Get or create AutoMapper instance"""
    global _auto_mapper
    if _auto_mapper is None and motor_controller and lidar_sensor:
        _auto_mapper = AutoMapper(motor_controller, lidar_sensor)
    return _auto_mapper