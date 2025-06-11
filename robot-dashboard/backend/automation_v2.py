#!/usr/bin/env python3
"""
Automation module v2 for robot mapping and pathfinding
Implements improved algorithms inspired by PythonRobotics
- Bug2 algorithm for navigation
- Probabilistic occupancy grid mapping
- Improved wall following with proper turning
"""

import logging
import threading
import time
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
from collections import deque
import math
from dataclasses import dataclass
from scipy.ndimage import binary_dilation

logger = logging.getLogger(__name__)

class RobotState(Enum):
    """Robot states for mapping"""
    IDLE = "idle"
    MOVING_TO_GOAL = "moving_to_goal"
    WALL_FOLLOWING = "wall_following"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    ROTATING = "rotating"
    EMERGENCY_STOP = "emergency_stop"
    EXPLORING = "exploring"

class WallFollowSide(Enum):
    """Which side to follow the wall"""
    LEFT = "left"
    RIGHT = "right"

@dataclass
class Point:
    """2D point"""
    x: float
    y: float
    
    def distance_to(self, other: 'Point') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle_to(self, other: 'Point') -> float:
        return math.atan2(other.y - self.y, other.x - self.x)

@dataclass
class Goal:
    """Goal position for navigation"""
    position: Point
    tolerance: float = 100.0  # mm

class OccupancyGrid:
    """Probabilistic occupancy grid map"""
    
    def __init__(self, width: int, height: int, resolution: float):
        """
        Args:
            width: Grid width in cells
            height: Grid height in cells
            resolution: Cell size in meters
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid = np.ones((height, width)) * 0.5  # Initialize as unknown (0.5)
        
        # Log-odds parameters
        self.l_occupied = 0.4  # Log-odds for occupied
        self.l_free = -0.4    # Log-odds for free
        self.l_min = -2.0     # Min log-odds (cap)
        self.l_max = 2.0      # Max log-odds (cap)
        
        # Convert to log-odds
        self.log_odds = np.log(self.grid / (1 - self.grid))
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (mm) to grid indices"""
        grid_x = int((x / 1000.0 + self.width * self.resolution / 2) / self.resolution)
        grid_y = int((y / 1000.0 + self.height * self.resolution / 2) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (mm)"""
        x = (grid_x * self.resolution - self.width * self.resolution / 2) * 1000
        y = (grid_y * self.resolution - self.height * self.resolution / 2) * 1000
        return x, y
    
    def is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid cell is within bounds"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def update_cell(self, grid_x: int, grid_y: int, occupied: bool):
        """Update cell probability using log-odds"""
        if not self.is_valid_cell(grid_x, grid_y):
            return
        
        # Update log-odds
        if occupied:
            self.log_odds[grid_y, grid_x] += self.l_occupied
        else:
            self.log_odds[grid_y, grid_x] += self.l_free
        
        # Clamp log-odds
        self.log_odds[grid_y, grid_x] = np.clip(
            self.log_odds[grid_y, grid_x], self.l_min, self.l_max
        )
    
    def get_probability(self, grid_x: int, grid_y: int) -> float:
        """Get occupancy probability for a cell"""
        if not self.is_valid_cell(grid_x, grid_y):
            return 0.5  # Unknown
        
        # Convert log-odds back to probability
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds[grid_y, grid_x]))
    
    def ray_cast(self, start_x: float, start_y: float, end_x: float, end_y: float):
        """Update cells along a ray (Bresenham's line algorithm)"""
        # Convert to grid coordinates
        x0, y0 = self.world_to_grid(start_x, start_y)
        x1, y1 = self.world_to_grid(end_x, end_y)
        
        # Bresenham's line algorithm
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            if self.is_valid_cell(x, y):
                cells.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        # Mark cells as free except the last one (obstacle)
        for i, (cx, cy) in enumerate(cells[:-1]):
            self.update_cell(cx, cy, occupied=False)
        
        # Mark last cell as occupied if within range
        if cells and len(cells) > 1:
            self.update_cell(cells[-1][0], cells[-1][1], occupied=True)

class AutoMapperV2:
    """Autonomous mapping using improved algorithms"""
    
    def __init__(self, motor_controller, lidar_sensor):
        """
        Initialize the auto mapper v2
        
        Args:
            motor_controller: Motor controller instance
            lidar_sensor: Lidar sensor instance
        """
        self.motor_controller = motor_controller
        self.lidar_sensor = lidar_sensor
        
        # Robot dimensions (in mm)
        self.robot_length = 500  # mm
        self.robot_width = 450   # mm
        self.wheelbase = 400     # mm
        
        # Sensor configuration
        self.lidar_offset_x = 0  # Lidar at center
        self.lidar_offset_y = self.robot_length / 2  # At front
        
        # Safety parameters (mm)
        self.safety_distance = 400      # Safety buffer from obstacles
        self.critical_distance = 250    # Emergency stop distance
        self.wall_follow_distance = 600 # Ideal wall following distance
        self.wall_follow_tolerance = 100
        
        # Speed parameters (0-100%)
        self.base_speed = 80      # Normal forward speed
        self.turn_speed = 70      # Speed during turns
        self.slow_speed = 60      # Slow speed for careful movement
        self.explore_speed = 85   # Speed for exploration
        
        # Control parameters
        self.command_delay = 0.1  # Delay between commands
        self.rotation_time_per_degree = 0.015  # Time to rotate 1 degree
        
        # State management
        self.state = RobotState.IDLE
        self.wall_follow_side = WallFollowSide.RIGHT
        self.mapping_active = False
        self.emergency_stop = False
        self._mapping_thread = None
        self._state_lock = threading.Lock()
        
        # Navigation
        self.current_goal = None
        self.exploration_goals = []
        self.visited_goals = set()
        
        # Odometry (simple dead reckoning)
        self.robot_pose = Point(0.0, 0.0)
        self.robot_theta = 0.0  # radians
        self.last_odometry_update = time.time()
        
        # Mapping
        self.occupancy_grid = OccupancyGrid(200, 200, 0.05)  # 10m x 10m, 5cm resolution
        self.path_history = deque(maxlen=5000)
        
        # Static obstacle removal
        self.static_scan_buffer = deque(maxlen=20)
        self.static_obstacles = {}  # Persistent obstacles to filter
        self.calibration_complete = False
        
        # Wall following state
        self.wall_lost_count = 0
        self.wall_lost_threshold = 10
        self.last_wall_distance = None
        
        # Bug2 algorithm state
        self.m_line_start = None
        self.m_line_goal = None
        self.hit_points = []
        self.leave_points = []
        self.following_wall = False
        
        logger.info("AutoMapper v2 initialized")
    
    def _set_motors(self, left_dir: str, left_speed: float, 
                    right_dir: str, right_speed: float):
        """
        Set motor speeds with proper constraints
        IMPORTANT: Cannot reverse motors - must stop one side for turning
        """
        # Ensure we don't reverse motors for turning
        if (left_dir == "forward" and right_dir == "backward") or \
           (left_dir == "backward" and right_dir == "forward"):
            # Convert to single-wheel turning
            if left_dir == "forward":
                # Turn right - stop right wheel
                self.motor_controller.set_motor(0, "brake", 0)
                time.sleep(self.command_delay)
                self.motor_controller.set_motor(1, left_dir, left_speed)
            else:
                # Turn left - stop left wheel  
                self.motor_controller.set_motor(1, "brake", 0)
                time.sleep(self.command_delay)
                self.motor_controller.set_motor(0, right_dir, right_speed)
        else:
            # Normal operation - both wheels same direction or brake
            self.motor_controller.set_motor(0, right_dir, right_speed)
            time.sleep(self.command_delay)
            self.motor_controller.set_motor(1, left_dir, left_speed)
    
    def start_mapping(self, goal_x: float = None, goal_y: float = None) -> bool:
        """
        Start autonomous mapping
        
        Args:
            goal_x: Optional goal X coordinate in mm
            goal_y: Optional goal Y coordinate in mm
        """
        with self._state_lock:
            if self.mapping_active:
                logger.warning("Mapping already active")
                return False
            
            self.mapping_active = True
            self.emergency_stop = False
            self.state = RobotState.IDLE
            
            # Set goal if provided
            if goal_x is not None and goal_y is not None:
                self.current_goal = Goal(Point(goal_x, goal_y))
                logger.info(f"Goal set to ({goal_x}, {goal_y})")
            
            # Reset state
            self.robot_pose = Point(0.0, 0.0)
            self.robot_theta = 0.0
            self.path_history.clear()
            self.hit_points.clear()
            self.leave_points.clear()
            
            # Calibrate if not done
            if not self.calibration_complete:
                self.calibrate_static_obstacles()
            
            # Start mapping thread
            self._mapping_thread = threading.Thread(target=self._mapping_loop)
            self._mapping_thread.daemon = True
            self._mapping_thread.start()
            
            logger.info("Started autonomous mapping v2")
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
    
    def calibrate_static_obstacles(self):
        """Calibrate static obstacles (robot parts) in lidar view"""
        logger.info("Calibrating static obstacles...")
        
        # Collect scans while stationary
        samples = []
        for _ in range(10):
            scan_data = self.lidar_sensor.get_latest_scan()
            if scan_data and scan_data.get("points"):
                samples.append(scan_data["points"])
            time.sleep(0.1)
        
        if len(samples) < 5:
            logger.warning("Not enough samples for calibration")
            return
        
        # Find persistent points (robot structure)
        angle_bins = {}
        for scan in samples:
            for point in scan:
                angle_deg = int(math.degrees(point.get("theta", 0)))
                distance = point.get("r", 0)
                
                if angle_deg not in angle_bins:
                    angle_bins[angle_deg] = []
                angle_bins[angle_deg].append(distance)
        
        # Identify static obstacles
        self.static_obstacles = {}
        for angle_deg, distances in angle_bins.items():
            if len(distances) >= len(samples) * 0.8:  # Present in 80% of scans
                mean_dist = np.mean(distances)
                std_dist = np.std(distances)
                
                # Low variance = static obstacle
                if std_dist < 0.02:  # 2cm threshold
                    self.static_obstacles[angle_deg] = {
                        "distance": mean_dist,
                        "std": std_dist
                    }
        
        self.calibration_complete = True
        logger.info(f"Calibration complete. Found {len(self.static_obstacles)} static points")
    
    def filter_static_obstacles(self, scan_data: Dict[str, Any]) -> Dict[str, Any]:
        """Remove static obstacles from scan data"""
        if not self.calibration_complete or not scan_data.get("points"):
            return scan_data
        
        filtered_points = []
        for point in scan_data["points"]:
            angle_deg = int(math.degrees(point.get("theta", 0)))
            distance = point.get("r", 0)
            
            # Check if this matches a static obstacle
            is_static = False
            for static_angle in range(angle_deg - 2, angle_deg + 3):
                if static_angle in self.static_obstacles:
                    static_dist = self.static_obstacles[static_angle]["distance"]
                    if abs(distance - static_dist) < 0.05:  # 5cm tolerance
                        is_static = True
                        break
            
            if not is_static:
                filtered_points.append(point)
        
        # Update scan data
        filtered_data = scan_data.copy()
        filtered_data["points"] = filtered_points
        filtered_data["point_count"] = len(filtered_points)
        
        # Recalculate min distance
        if filtered_points:
            min_dist = min(p.get("r", float('inf')) for p in filtered_points)
            filtered_data["min_distance"] = min_dist
        
        return filtered_data
    
    def _mapping_loop(self):
        """Main mapping loop with improved state machine"""
        logger.info("Mapping loop v2 started")
        
        # Wait for sensors
        time.sleep(2.0)
        
        while self.mapping_active and not self.emergency_stop:
            try:
                # Get and filter lidar scan
                scan_data = self.lidar_sensor.get_latest_scan()
                if not scan_data:
                    time.sleep(0.1)
                    continue
                
                # Filter static obstacles
                filtered_scan = self.filter_static_obstacles(scan_data)
                
                # Update occupancy grid
                self._update_occupancy_grid(filtered_scan)
                
                # Process scan and decide action
                self._process_scan(filtered_scan)
                
                # Execute state-based behavior
                if self.state == RobotState.MOVING_TO_GOAL:
                    self._move_to_goal(filtered_scan)
                elif self.state == RobotState.WALL_FOLLOWING:
                    self._follow_wall(filtered_scan)
                elif self.state == RobotState.OBSTACLE_AVOIDANCE:
                    self._avoid_obstacle(filtered_scan)
                elif self.state == RobotState.ROTATING:
                    self._execute_rotation(filtered_scan)
                elif self.state == RobotState.EXPLORING:
                    self._explore(filtered_scan)
                elif self.state == RobotState.EMERGENCY_STOP:
                    self._emergency_stop()
                    break
                
                # Update odometry
                self._update_odometry()
                
                # Record path
                self.path_history.append(Point(self.robot_pose.x, self.robot_pose.y))
                
                # Small delay
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
        min_distance = scan_data.get("min_distance", float('inf')) * 1000  # to mm
        
        # Check for emergency stop
        if min_distance < self.critical_distance:
            logger.warning(f"CRITICAL: Obstacle at {min_distance}mm - Emergency stop!")
            self.state = RobotState.EMERGENCY_STOP
            return
        
        # Check for obstacles requiring avoidance
        if min_distance < self.safety_distance:
            if self.state != RobotState.WALL_FOLLOWING:
                logger.info(f"Obstacle detected at {min_distance}mm - Avoiding")
                self.state = RobotState.OBSTACLE_AVOIDANCE
            return
        
        # Determine behavior based on current state and goal
        if self.current_goal and not self.following_wall:
            # Check if we can move toward goal
            if self._is_path_clear_to_goal(scan_data):
                self.state = RobotState.MOVING_TO_GOAL
            else:
                # Start wall following (Bug2)
                self.following_wall = True
                self.state = RobotState.WALL_FOLLOWING
                self.hit_points.append(Point(self.robot_pose.x, self.robot_pose.y))
                logger.info("Path blocked - starting wall following")
        elif self.following_wall:
            # Continue wall following until we can leave
            if self._can_leave_wall(scan_data):
                self.following_wall = False
                self.state = RobotState.MOVING_TO_GOAL
                self.leave_points.append(Point(self.robot_pose.x, self.robot_pose.y))
                logger.info("Can reach goal - leaving wall")
        else:
            # No goal - explore
            self.state = RobotState.EXPLORING
    
    def _update_occupancy_grid(self, scan_data: Dict[str, Any]):
        """Update occupancy grid with scan data"""
        if not scan_data.get("points"):
            return
        
        # Robot position in world frame
        robot_x = self.robot_pose.x
        robot_y = self.robot_pose.y
        
        # Lidar position in world frame
        lidar_x = robot_x + self.lidar_offset_x * math.cos(self.robot_theta) - \
                  self.lidar_offset_y * math.sin(self.robot_theta)
        lidar_y = robot_y + self.lidar_offset_x * math.sin(self.robot_theta) + \
                  self.lidar_offset_y * math.cos(self.robot_theta)
        
        # Update grid with each scan point
        for point in scan_data["points"]:
            if point["r"] > 0.1 and point["r"] < 10.0:  # Valid range
                # Point in world frame
                world_angle = point["theta"] + self.robot_theta
                end_x = lidar_x + point["r"] * 1000 * math.cos(world_angle)
                end_y = lidar_y + point["r"] * 1000 * math.sin(world_angle)
                
                # Ray cast to update cells
                self.occupancy_grid.ray_cast(lidar_x, lidar_y, end_x, end_y)
    
    def _is_path_clear_to_goal(self, scan_data: Dict[str, Any]) -> bool:
        """Check if path to goal is clear"""
        if not self.current_goal:
            return False
        
        # Calculate angle to goal
        goal_angle = self.robot_pose.angle_to(self.current_goal.position)
        angle_diff = self._normalize_angle(goal_angle - self.robot_theta)
        
        # Check if goal is in front (±45 degrees)
        if abs(angle_diff) > math.pi / 4:
            return False
        
        # Check for obstacles in goal direction
        for point in scan_data.get("points", []):
            point_angle = self._normalize_angle(point["theta"])
            if abs(point_angle - angle_diff) < math.pi / 6:  # ±30 degrees
                if point["r"] * 1000 < self.robot_pose.distance_to(self.current_goal.position):
                    return False
        
        return True
    
    def _can_leave_wall(self, scan_data: Dict[str, Any]) -> bool:
        """Check if robot can leave wall (Bug2 condition)"""
        if not self.current_goal or not self.m_line_start:
            return False
        
        # Check if we're on the M-line
        distance_to_line = self._distance_to_m_line()
        if distance_to_line > 100:  # Not on M-line
            return False
        
        # Check if path to goal is clear
        return self._is_path_clear_to_goal(scan_data)
    
    def _distance_to_m_line(self) -> float:
        """Calculate distance to M-line (line from start to goal)"""
        if not self.m_line_start or not self.m_line_goal:
            return float('inf')
        
        # Point-to-line distance formula
        x0, y0 = self.robot_pose.x, self.robot_pose.y
        x1, y1 = self.m_line_start.x, self.m_line_start.y
        x2, y2 = self.m_line_goal.x, self.m_line_goal.y
        
        num = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
        den = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        
        return num / den if den > 0 else float('inf')
    
    def _move_to_goal(self, scan_data: Dict[str, Any]):
        """Move toward goal using simple proportional control"""
        if not self.current_goal:
            self.state = RobotState.EXPLORING
            return
        
        # Check if reached goal
        distance_to_goal = self.robot_pose.distance_to(self.current_goal.position)
        if distance_to_goal < self.current_goal.tolerance:
            logger.info("Goal reached!")
            self.visited_goals.add((self.current_goal.position.x, self.current_goal.position.y))
            self.current_goal = None
            self.state = RobotState.IDLE
            return
        
        # Calculate angle to goal
        goal_angle = self.robot_pose.angle_to(self.current_goal.position)
        angle_error = self._normalize_angle(goal_angle - self.robot_theta)
        
        # Proportional control for steering
        if abs(angle_error) > math.pi / 6:  # > 30 degrees
            # Need to rotate
            self.state = RobotState.ROTATING
            self.target_angle = goal_angle
        else:
            # Move forward with steering adjustment
            steer_gain = 0.5
            speed_diff = steer_gain * angle_error
            
            left_speed = self.base_speed - speed_diff * 10
            right_speed = self.base_speed + speed_diff * 10
            
            # Clamp speeds
            left_speed = max(self.slow_speed, min(100, left_speed))
            right_speed = max(self.slow_speed, min(100, right_speed))
            
            self._set_motors("forward", left_speed, "forward", right_speed)
    
    def _follow_wall(self, scan_data: Dict[str, Any]):
        """Follow wall using single-wheel turning"""
        # Get wall distance on following side
        if self.wall_follow_side == WallFollowSide.RIGHT:
            wall_angle = -math.pi / 2  # -90 degrees
            front_angle = -math.pi / 4  # -45 degrees
        else:
            wall_angle = math.pi / 2   # 90 degrees
            front_angle = math.pi / 4   # 45 degrees
        
        wall_distance = self._get_distance_at_angle(scan_data, wall_angle)
        front_distance = self._get_distance_at_angle(scan_data, 0)
        diagonal_distance = self._get_distance_at_angle(scan_data, front_angle)
        
        # Check if wall is lost
        if wall_distance is None or wall_distance > self.wall_follow_distance * 2:
            self.wall_lost_count += 1
            if self.wall_lost_count > self.wall_lost_threshold:
                # Wall lost - try to find it again
                logger.info("Wall lost - searching")
                if self.wall_follow_side == WallFollowSide.RIGHT:
                    # Turn right to find wall
                    self._set_motors("forward", self.turn_speed, "brake", 0)
                else:
                    # Turn left to find wall
                    self._set_motors("brake", 0, "forward", self.turn_speed)
                return
        else:
            self.wall_lost_count = 0
        
        # Check front obstacle
        if front_distance and front_distance < self.wall_follow_distance:
            # Turn away from wall
            logger.info("Front obstacle - turning away from wall")
            if self.wall_follow_side == WallFollowSide.RIGHT:
                # Turn left (away from right wall)
                self._set_motors("brake", 0, "forward", self.turn_speed)
            else:
                # Turn right (away from left wall)
                self._set_motors("forward", self.turn_speed, "brake", 0)
            return
        
        # Normal wall following with proportional control
        if wall_distance:
            error = wall_distance - self.wall_follow_distance
            
            # Simple P controller
            kp = 0.1
            correction = kp * error
            
            # Apply correction using single-wheel turning
            if abs(error) > self.wall_follow_tolerance:
                if error > 0:  # Too far from wall
                    if self.wall_follow_side == WallFollowSide.RIGHT:
                        # Turn right slightly
                        left_speed = self.base_speed
                        right_speed = max(0, self.base_speed - abs(correction))
                    else:
                        # Turn left slightly
                        left_speed = max(0, self.base_speed - abs(correction))
                        right_speed = self.base_speed
                else:  # Too close to wall
                    if self.wall_follow_side == WallFollowSide.RIGHT:
                        # Turn left slightly
                        left_speed = max(0, self.base_speed - abs(correction))
                        right_speed = self.base_speed
                    else:
                        # Turn right slightly
                        left_speed = self.base_speed
                        right_speed = max(0, self.base_speed - abs(correction))
                
                self._set_motors("forward", left_speed, "forward", right_speed)
            else:
                # Good distance - go straight
                self._set_motors("forward", self.base_speed, "forward", self.base_speed)
    
    def _avoid_obstacle(self, scan_data: Dict[str, Any]):
        """Avoid obstacle using single-wheel turning"""
        # Stop first
        self.motor_controller.stop_all_motors()
        time.sleep(0.2)
        
        # Find best escape direction
        left_clear = self._check_direction_clear(scan_data, math.pi / 2)
        right_clear = self._check_direction_clear(scan_data, -math.pi / 2)
        
        if right_clear and (not left_clear or self.wall_follow_side == WallFollowSide.RIGHT):
            # Turn right
            logger.info("Obstacle avoidance - turning right")
            self._set_motors("forward", self.turn_speed, "brake", 0)
        elif left_clear:
            # Turn left
            logger.info("Obstacle avoidance - turning left")
            self._set_motors("brake", 0, "forward", self.turn_speed)
        else:
            # Back up carefully (both sides blocked)
            logger.warning("Both sides blocked - backing up")
            self._set_motors("backward", self.slow_speed, "backward", self.slow_speed)
            time.sleep(1.0)
            # Then try turning
            self._set_motors("forward", self.turn_speed, "brake", 0)
        
        time.sleep(0.5)
        self.state = RobotState.IDLE
    
    def _execute_rotation(self, scan_data: Dict[str, Any]):
        """Execute rotation to target angle using single-wheel turning"""
        if not hasattr(self, 'target_angle'):
            self.state = RobotState.IDLE
            return
        
        angle_error = self._normalize_angle(self.target_angle - self.robot_theta)
        
        if abs(angle_error) < math.pi / 18:  # < 10 degrees
            # Close enough
            self.motor_controller.stop_all_motors()
            self.state = RobotState.IDLE
            return
        
        # Calculate rotation time
        rotation_time = abs(angle_error) * self.rotation_time_per_degree * 180 / math.pi
        
        if angle_error > 0:
            # Turn left
            self._set_motors("brake", 0, "forward", self.turn_speed)
        else:
            # Turn right
            self._set_motors("forward", self.turn_speed, "brake", 0)
        
        # Simple timed rotation (can be improved with feedback)
        time.sleep(min(rotation_time, 0.5))  # Max 0.5s per iteration
    
    def _explore(self, scan_data: Dict[str, Any]):
        """Explore unknown areas"""
        # Simple exploration: move forward and turn when blocked
        front_distance = self._get_distance_at_angle(scan_data, 0)
        
        if front_distance and front_distance < self.wall_follow_distance:
            # Obstacle ahead - pick a direction to turn
            left_distance = self._get_distance_at_angle(scan_data, math.pi / 2)
            right_distance = self._get_distance_at_angle(scan_data, -math.pi / 2)
            
            if right_distance and (not left_distance or right_distance > left_distance):
                # Turn right
                self._set_motors("forward", self.turn_speed, "brake", 0)
            else:
                # Turn left
                self._set_motors("brake", 0, "forward", self.turn_speed)
            
            time.sleep(1.0)
        else:
            # Move forward
            self._set_motors("forward", self.explore_speed, "forward", self.explore_speed)
    
    def _get_distance_at_angle(self, scan_data: Dict[str, Any], target_angle: float) -> Optional[float]:
        """Get distance measurement at specific angle"""
        points = scan_data.get("points", [])
        if not points:
            return None
        
        # Find points near target angle
        angle_tolerance = math.pi / 12  # 15 degrees
        valid_distances = []
        
        for point in points:
            angle_diff = abs(self._normalize_angle(point["theta"] - target_angle))
            if angle_diff <= angle_tolerance:
                valid_distances.append(point["r"] * 1000)  # Convert to mm
        
        return np.median(valid_distances) if valid_distances else None
    
    def _check_direction_clear(self, scan_data: Dict[str, Any], angle: float) -> bool:
        """Check if direction is clear for movement"""
        distance = self._get_distance_at_angle(scan_data, angle)
        return distance is None or distance > self.safety_distance
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _update_odometry(self):
        """Update robot position estimate"""
        current_time = time.time()
        dt = current_time - self.last_odometry_update
        self.last_odometry_update = current_time
        
        # Get motor states
        motor_states = self.motor_controller.get_all_motor_states()
        
        # Estimate velocities (simplified)
        left_velocity = 0
        right_velocity = 0
        
        if motor_states[1]["direction"] == "forward":
            left_velocity = motor_states[1]["speed"] * 10  # mm/s at 100%
        elif motor_states[1]["direction"] == "backward":
            left_velocity = -motor_states[1]["speed"] * 10
        
        if motor_states[0]["direction"] == "forward":
            right_velocity = motor_states[0]["speed"] * 10
        elif motor_states[0]["direction"] == "backward":
            right_velocity = -motor_states[0]["speed"] * 10
        
        # Differential drive kinematics
        v = (left_velocity + right_velocity) / 2
        w = (right_velocity - left_velocity) / self.wheelbase
        
        # Update pose
        if abs(w) < 0.001:
            # Straight motion
            self.robot_pose.x += v * dt * math.cos(self.robot_theta)
            self.robot_pose.y += v * dt * math.sin(self.robot_theta)
        else:
            # Arc motion
            self.robot_pose.x += (v/w) * (math.sin(self.robot_theta + w*dt) - math.sin(self.robot_theta))
            self.robot_pose.y += (v/w) * (-math.cos(self.robot_theta + w*dt) + math.cos(self.robot_theta))
            self.robot_theta = self._normalize_angle(self.robot_theta + w * dt)
    
    def _emergency_stop(self):
        """Execute emergency stop"""
        logger.warning("Emergency stop activated!")
        self.motor_controller.stop_all_motors()
        self.state = RobotState.EMERGENCY_STOP
        time.sleep(1.0)
    
    def get_status(self) -> Dict[str, Any]:
        """Get current mapping status"""
        with self._state_lock:
            return {
                "active": self.mapping_active,
                "state": self.state.value,
                "emergency_stop": self.emergency_stop,
                "robot_position": {
                    "x": round(self.robot_pose.x),
                    "y": round(self.robot_pose.y),
                    "theta": round(math.degrees(self.robot_theta), 1)
                },
                "current_goal": {
                    "x": self.current_goal.position.x,
                    "y": self.current_goal.position.y
                } if self.current_goal else None,
                "path_length": len(self.path_history),
                "calibration_complete": self.calibration_complete,
                "static_obstacles_count": len(self.static_obstacles),
                "wall_following": self.following_wall,
                "wall_side": self.wall_follow_side.value
            }
    
    def set_goal(self, x: float, y: float) -> bool:
        """Set navigation goal"""
        self.current_goal = Goal(Point(x, y))
        self.m_line_start = Point(self.robot_pose.x, self.robot_pose.y)
        self.m_line_goal = self.current_goal.position
        logger.info(f"Goal set to ({x}, {y})")
        return True
    
    def get_occupancy_grid(self) -> np.ndarray:
        """Get occupancy grid as probability array"""
        # Convert log-odds to probabilities
        return 1.0 - 1.0 / (1.0 + np.exp(self.occupancy_grid.log_odds))

# Singleton instance
_auto_mapper_v2 = None

def get_auto_mapper_v2(motor_controller=None, lidar_sensor=None):
    """Get or create AutoMapperV2 instance"""
    global _auto_mapper_v2
    if _auto_mapper_v2 is None and motor_controller and lidar_sensor:
        _auto_mapper_v2 = AutoMapperV2(motor_controller, lidar_sensor)
    return _auto_mapper_v2