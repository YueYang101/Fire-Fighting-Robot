#!/usr/bin/env python3
"""
Enhanced Automation module for robot mapping and pathfinding
Implements frontier-based exploration, improved Bug2 algorithm, and advanced grid mapping
"""

import logging
import threading
import time
import numpy as np
from typing import Dict, List, Tuple, Optional, Any, Set
from enum import Enum
from collections import deque
import math
import json
import os
from datetime import datetime
from dataclasses import dataclass, asdict
from scipy.ndimage import distance_transform_edt
import heapq

logger = logging.getLogger(__name__)

class RobotState(Enum):
    """Robot states for mapping"""
    IDLE = "idle"
    EXPLORING = "exploring"
    WALL_FOLLOWING = "wall_following"
    TURNING = "turning"
    MOVING_TO_FRONTIER = "moving_to_frontier"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    EMERGENCY_STOP = "emergency_stop"
    ROTATING_SCAN = "rotating_scan"

class TurnDirection(Enum):
    """Turn direction preference"""
    LEFT = "left"
    RIGHT = "right"

class CellType(Enum):
    """Occupancy grid cell types"""
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2
    FRONTIER = 3
    VISITED = 4

@dataclass
class Frontier:
    """Frontier data structure"""
    cells: List[Tuple[int, int]]
    centroid: Tuple[float, float]
    size: int
    distance: float
    information_gain: float
    
    @property
    def utility(self) -> float:
        """Calculate frontier utility for selection"""
        # Higher utility = better frontier
        # Consider size, distance, and information gain
        if self.distance < 0.1:
            return 0
        return (self.information_gain * self.size) / (self.distance + 1.0)

@dataclass
class PathMemory:
    """Memory of successful paths"""
    start: Tuple[float, float]
    goal: Tuple[float, float]
    path: List[Tuple[float, float]]
    timestamp: float
    success_count: int = 1
    
class EnhancedAutoMapper:
    """Enhanced autonomous mapping using frontier-based exploration"""
    
    def __init__(self, motor_controller, lidar_sensor):
        """Initialize the enhanced auto mapper"""
        self.motor_controller = motor_controller
        self.lidar_sensor = lidar_sensor
        
        # Robot dimensions (in mm)
        self.robot_length = 500
        self.robot_width = 450
        self.wheelbase = 400
        
        # Sensor offsets
        self.lidar_offset_front = self.robot_length / 2
        self.lidar_offset_rear = self.robot_length / 2
        self.lidar_offset_side = self.robot_width / 2
        
        # Safety parameters
        self.safety_distance = 300
        self.wall_follow_distance = 500
        self.wall_follow_tolerance = 150
        self.critical_distance = 200
        
        # Speed parameters
        self.base_speed = 85
        self.turn_speed = 75
        self.slow_speed = 75
        self.exploration_speed = 90
        
        # Control parameters
        self.turn_direction = TurnDirection.RIGHT
        self.command_delay = 0.2
        self.sensor_delay = 0.1
        
        # State management
        self.state = RobotState.IDLE
        self.mapping_active = False
        self.emergency_stop = False
        self._mapping_thread = None
        self._state_lock = threading.Lock()
        self.idle_timeout = 3.0
        self.last_state_change = time.time()
        
        # Enhanced odometry with Kalman filter estimation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.last_odometry_update = time.time()
        self.odometry_covariance = np.eye(3) * 0.1
        
        # Enhanced grid mapping
        self.grid_size = 5000  # 50m x 50m with 1cm resolution
        self.grid_resolution = 10  # mm per cell
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), CellType.UNKNOWN.value)
        self.confidence_grid = np.zeros((self.grid_size, self.grid_size))
        self.visit_count_grid = np.zeros((self.grid_size, self.grid_size), dtype=int)
        
        # Frontier detection
        self.frontiers: List[Frontier] = []
        self.current_frontier: Optional[Frontier] = None
        self.frontier_blacklist: Set[Tuple[int, int]] = set()
        self.frontier_update_interval = 2.0
        self.last_frontier_update = 0
        
        # Path memory system
        self.path_memory: List[PathMemory] = []
        self.max_path_memory = 100
        self.path_memory_file = "path_memory.json"
        self.load_path_memory()
        
        # Decision logging
        self.decision_log: List[Dict] = []
        self.max_decision_log = 1000
        self.decision_file = "mapping_decisions.log"
        
        # Background removal with statistical outlier removal
        self.static_scan_buffer = deque(maxlen=20)
        self.dynamic_threshold = 0.1  # 10cm movement threshold
        self.outlier_threshold = 2.5  # Standard deviations
        self.background_model = {}
        
        # Enhanced PID controller for wall following
        self.kp = 0.8
        self.ki = 0.2
        self.kd = 0.3
        self.integral_error = 0.0
        self.last_error = 0.0
        self.integral_limit = 100.0
        
        # Bug2 algorithm enhancements
        self.start_goal_line = None
        self.hit_points = []
        self.leave_points = []
        self.m_line_threshold = 100  # mm
        self.stuck_detection_window = 5.0
        self.position_history = deque(maxlen=50)
        
        # Performance metrics
        self.start_time = None
        self.total_distance_traveled = 0.0
        self.areas_explored = 0
        self.mapping_efficiency = 0.0
        
        logger.info("Enhanced AutoMapper initialized")
    
    def log_decision(self, decision_type: str, details: Dict[str, Any]):
        """Log mapping decisions for analysis"""
        decision = {
            "timestamp": time.time(),
            "type": decision_type,
            "state": self.state.value,
            "position": {
                "x": round(self.robot_x, 1),
                "y": round(self.robot_y, 1),
                "theta": round(math.degrees(self.robot_theta), 1)
            },
            "details": details
        }
        
        self.decision_log.append(decision)
        
        # Keep log size manageable
        if len(self.decision_log) > self.max_decision_log:
            self.decision_log.pop(0)
        
        # Log to file
        try:
            with open(self.decision_file, 'a') as f:
                f.write(json.dumps(decision) + '\n')
        except Exception as e:
            logger.error(f"Failed to write decision log: {e}")
        
        # Also log to console for real-time monitoring
        logger.info(f"DECISION: {decision_type} - {details}")
    
    def detect_frontiers(self) -> List[Frontier]:
        """Detect frontiers using edge detection on occupancy grid"""
        frontiers = []
        frontier_cells = set()
        
        # Find all frontier cells (free cells adjacent to unknown)
        for y in range(1, self.grid_size - 1):
            for x in range(1, self.grid_size - 1):
                if self.occupancy_grid[y, x] == CellType.FREE.value:
                    # Check 8-connected neighbors
                    has_unknown_neighbor = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dy == 0 and dx == 0:
                                continue
                            ny, nx = y + dy, x + dx
                            if self.occupancy_grid[ny, nx] == CellType.UNKNOWN.value:
                                has_unknown_neighbor = True
                                break
                        if has_unknown_neighbor:
                            break
                    
                    if has_unknown_neighbor:
                        frontier_cells.add((x, y))
        
        # Cluster frontier cells into frontier regions
        visited = set()
        for cell in frontier_cells:
            if cell in visited or cell in self.frontier_blacklist:
                continue
            
            # BFS to find connected frontier cells
            cluster = []
            queue = deque([cell])
            
            while queue:
                current = queue.popleft()
                if current in visited:
                    continue
                
                visited.add(current)
                cluster.append(current)
                
                # Check 8-connected neighbors
                x, y = current
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = x + dx, y + dy
                        neighbor = (nx, ny)
                        if neighbor in frontier_cells and neighbor not in visited:
                            queue.append(neighbor)
            
            # Create frontier if cluster is large enough
            if len(cluster) >= 5:  # Minimum frontier size
                # Calculate centroid
                cx = sum(c[0] for c in cluster) / len(cluster)
                cy = sum(c[1] for c in cluster) / len(cluster)
                
                # Convert to world coordinates
                world_cx = (cx - self.grid_size // 2) * self.grid_resolution
                world_cy = (cy - self.grid_size // 2) * self.grid_resolution
                
                # Calculate distance from robot
                distance = math.sqrt((world_cx - self.robot_x)**2 + 
                                   (world_cy - self.robot_y)**2)
                
                # Calculate information gain (based on unexplored area nearby)
                info_gain = self.calculate_information_gain(cx, cy)
                
                frontier = Frontier(
                    cells=cluster,
                    centroid=(world_cx, world_cy),
                    size=len(cluster),
                    distance=distance,
                    information_gain=info_gain
                )
                frontiers.append(frontier)
        
        # Sort by utility
        frontiers.sort(key=lambda f: f.utility, reverse=True)
        
        self.log_decision("frontier_detection", {
            "num_frontiers": len(frontiers),
            "frontier_cells": len(frontier_cells),
            "top_frontier_utility": frontiers[0].utility if frontiers else 0
        })
        
        return frontiers
    
    def calculate_information_gain(self, cx: int, cy: int, radius: int = 50) -> float:
        """Calculate information gain for a frontier centroid"""
        gain = 0.0
        
        # Count unknown cells within radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    ny, nx = int(cy + dy), int(cx + dx)
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        if self.occupancy_grid[ny, nx] == CellType.UNKNOWN.value:
                            gain += 1.0
                        elif self.visit_count_grid[ny, nx] == 0:
                            gain += 0.5
        
        return gain
    
    def select_best_frontier(self) -> Optional[Frontier]:
        """Select the best frontier to explore"""
        if not self.frontiers:
            return None
        
        # Filter out blacklisted and very close frontiers
        valid_frontiers = [f for f in self.frontiers 
                          if f.distance > self.safety_distance and
                          len(f.cells) > 10]
        
        if not valid_frontiers:
            # If no valid frontiers, try smaller ones
            valid_frontiers = [f for f in self.frontiers if f.distance > self.critical_distance]
        
        if not valid_frontiers:
            return None
        
        # Select based on utility
        best_frontier = max(valid_frontiers, key=lambda f: f.utility)
        
        self.log_decision("frontier_selection", {
            "selected_centroid": best_frontier.centroid,
            "utility": round(best_frontier.utility, 2),
            "size": best_frontier.size,
            "distance": round(best_frontier.distance, 1)
        })
        
        return best_frontier
    
    def plan_path_to_frontier(self, frontier: Frontier) -> Optional[List[Tuple[float, float]]]:
        """Plan path to frontier using A* with dynamic replanning"""
        # Convert positions to grid coordinates
        start_x = int(self.robot_x / self.grid_resolution + self.grid_size // 2)
        start_y = int(self.robot_y / self.grid_resolution + self.grid_size // 2)
        goal_x = int(frontier.centroid[0] / self.grid_resolution + self.grid_size // 2)
        goal_y = int(frontier.centroid[1] / self.grid_resolution + self.grid_size // 2)
        
        # First check path memory
        memory_path = self.check_path_memory((self.robot_x, self.robot_y), frontier.centroid)
        if memory_path:
            self.log_decision("path_planning", {
                "method": "memory",
                "path_length": len(memory_path)
            })
            return memory_path
        
        # A* pathfinding
        path = self.astar_pathfinding((start_x, start_y), (goal_x, goal_y))
        
        if path:
            # Convert back to world coordinates
            world_path = []
            for x, y in path:
                wx = (x - self.grid_size // 2) * self.grid_resolution
                wy = (y - self.grid_size // 2) * self.grid_resolution
                world_path.append((wx, wy))
            
            # Store successful path
            self.store_path_memory((self.robot_x, self.robot_y), frontier.centroid, world_path)
            
            self.log_decision("path_planning", {
                "method": "astar",
                "path_length": len(world_path),
                "start": (self.robot_x, self.robot_y),
                "goal": frontier.centroid
            })
            
            return world_path
        
        return None
    
    def astar_pathfinding(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* pathfinding on occupancy grid"""
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        # Create cost map with safety margins
        cost_map = self.create_cost_map()
        
        # A* implementation
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            # Check neighbors
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    
                    neighbor = (current[0] + dx, current[1] + dy)
                    
                    # Check bounds
                    if (neighbor[0] < 0 or neighbor[0] >= self.grid_size or
                        neighbor[1] < 0 or neighbor[1] >= self.grid_size):
                        continue
                    
                    # Check if traversable
                    if cost_map[neighbor[1], neighbor[0]] >= 1000:
                        continue
                    
                    # Calculate tentative g score
                    tentative_g = g_score[current] + cost_map[neighbor[1], neighbor[0]]
                    
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None
    
    def create_cost_map(self) -> np.ndarray:
        """Create cost map for path planning with safety margins"""
        cost_map = np.ones((self.grid_size, self.grid_size))
        
        # Set costs based on occupancy
        cost_map[self.occupancy_grid == CellType.OCCUPIED.value] = 1000
        cost_map[self.occupancy_grid == CellType.UNKNOWN.value] = 10
        cost_map[self.occupancy_grid == CellType.FREE.value] = 1
        
        # Add safety margin around obstacles
        obstacle_mask = self.occupancy_grid == CellType.OCCUPIED.value
        distance_map = distance_transform_edt(~obstacle_mask)
        
        # Increase cost near obstacles
        safety_cells = int(self.safety_distance / self.grid_resolution)
        for i in range(1, safety_cells):
            mask = (distance_map == i)
            cost_map[mask] = 1 + (safety_cells - i) * 2
        
        return cost_map
    
    def update_occupancy_grid(self, scan_data: Dict[str, Any]):
        """Update occupancy grid with new scan data using probabilistic updates"""
        if not scan_data or 'points' not in scan_data:
            return
        
        # Get robot grid position
        robot_gx = int(self.robot_x / self.grid_resolution + self.grid_size // 2)
        robot_gy = int(self.robot_y / self.grid_resolution + self.grid_size // 2)
        
        # Mark current position as visited
        if 0 <= robot_gx < self.grid_size and 0 <= robot_gy < self.grid_size:
            self.visit_count_grid[robot_gy, robot_gx] += 1
        
        # Process each scan point
        for point in scan_data['points']:
            # Get point in world coordinates
            angle = point.get('theta', point.get('angle', 0)) + self.robot_theta
            distance = point.get('r', point.get('distance', 0)) * 1000  # Convert to mm
            
            if distance < 100 or distance > 10000:  # Skip invalid ranges
                continue
            
            # Point coordinates
            px = self.robot_x + distance * math.cos(angle)
            py = self.robot_y + distance * math.sin(angle)
            
            # Convert to grid
            pgx = int(px / self.grid_resolution + self.grid_size // 2)
            pgy = int(py / self.grid_resolution + self.grid_size // 2)
            
            # Ray tracing - mark free space
            points_along_ray = self.bresenham_line(robot_gx, robot_gy, pgx, pgy)
            
            # Update free space along ray
            for i, (gx, gy) in enumerate(points_along_ray[:-1]):
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    # Probabilistic update
                    self.confidence_grid[gy, gx] += 0.1
                    if self.confidence_grid[gy, gx] > 0.5:
                        self.occupancy_grid[gy, gx] = CellType.FREE.value
            
            # Mark endpoint as occupied (if in range)
            if 0 <= pgx < self.grid_size and 0 <= pgy < self.grid_size:
                if distance < 4000:  # Only mark close obstacles as occupied
                    self.confidence_grid[pgy, pgx] -= 0.3
                    if self.confidence_grid[pgy, pgx] < -0.5:
                        self.occupancy_grid[pgy, pgx] = CellType.OCCUPIED.value
    
    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm for ray tracing"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            
            if x0 == x1 and y0 == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return points
    
    def check_if_stuck(self) -> bool:
        """Check if robot is stuck by analyzing position history"""
        if len(self.position_history) < 20:
            return False
        
        # Calculate movement variance
        positions = list(self.position_history)
        x_values = [p[0] for p in positions]
        y_values = [p[1] for p in positions]
        
        x_var = np.var(x_values)
        y_var = np.var(y_values)
        
        # If variance is very low, robot is stuck
        stuck = (x_var < 100 and y_var < 100)  # Less than 10cm movement
        
        if stuck:
            self.log_decision("stuck_detection", {
                "x_variance": round(x_var, 2),
                "y_variance": round(y_var, 2),
                "position_samples": len(positions)
            })
        
        return stuck
    
    def escape_stuck_situation(self):
        """Execute escape maneuver when stuck"""
        self.log_decision("escape_maneuver", {
            "method": "rotate_and_move"
        })
        
        # Try rotating to find new path
        self.state = RobotState.ROTATING_SCAN
        self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
        time.sleep(2.0)  # Rotate for 2 seconds
        
        # Then try moving in a different direction
        self._set_motors("forward", self.base_speed, "forward", self.base_speed)
        time.sleep(1.0)
        
        # Clear position history
        self.position_history.clear()
        
        # Blacklist current frontier if any
        if self.current_frontier:
            for cell in self.current_frontier.cells[:10]:  # Blacklist some cells
                self.frontier_blacklist.add(cell)
            self.current_frontier = None
    
    def enhanced_wall_following(self, scan_data: Dict[str, Any]) -> Tuple[str, float, float]:
        """Enhanced wall following with better obstacle handling"""
        # Get wall distances
        right_dist = self._get_wall_distance(scan_data, -90)
        front_dist = self._get_wall_distance(scan_data, 0)
        left_dist = self._get_wall_distance(scan_data, 90)
        right_front_dist = self._get_wall_distance(scan_data, -45)
        
        # Adjust for sensor position
        if right_dist is not None:
            right_dist = max(0, right_dist - self.lidar_offset_side)
        
        # Decision logic
        if front_dist and front_dist < self.wall_follow_distance:
            # Wall ahead - need to turn
            if self.turn_direction == TurnDirection.RIGHT and (not right_dist or right_dist > self.wall_follow_distance):
                return "turn_right", self.turn_speed, self.turn_speed
            elif not left_dist or left_dist > self.wall_follow_distance:
                return "turn_left", self.turn_speed, self.turn_speed
            else:
                # Both sides blocked - reverse
                return "reverse", self.slow_speed, self.slow_speed
        
        elif right_dist and right_dist < self.wall_follow_distance * 2:
            # Wall on right - follow it
            error = right_dist - self.wall_follow_distance
            
            # Enhanced PID control
            self.integral_error += error * self.command_delay
            self.integral_error = max(-self.integral_limit, min(self.integral_limit, self.integral_error))
            derivative = (error - self.last_error) / self.command_delay
            self.last_error = error
            
            correction = (self.kp * error + self.ki * self.integral_error + self.kd * derivative)
            
            # Check for corner
            if right_front_dist and right_front_dist < right_dist * 0.8:
                # Inside corner detected - turn left more
                correction -= 20
            
            # Convert to differential speed
            speed_diff = max(-30, min(30, correction / 40))
            
            left_speed = self.base_speed + speed_diff
            right_speed = self.base_speed - speed_diff
            
            # Clamp speeds
            left_speed = max(self.slow_speed, min(100, left_speed))
            right_speed = max(self.slow_speed, min(100, right_speed))
            
            return "wall_follow", left_speed, right_speed
        
        else:
            # No wall - move forward
            return "forward", self.exploration_speed, self.exploration_speed
    
    def load_path_memory(self):
        """Load path memory from file"""
        try:
            if os.path.exists(self.path_memory_file):
                with open(self.path_memory_file, 'r') as f:
                    data = json.load(f)
                    for item in data:
                        self.path_memory.append(PathMemory(**item))
                logger.info(f"Loaded {len(self.path_memory)} paths from memory")
        except Exception as e:
            logger.error(f"Failed to load path memory: {e}")
    
    def save_path_memory(self):
        """Save path memory to file"""
        try:
            data = [asdict(pm) for pm in self.path_memory[-self.max_path_memory:]]
            with open(self.path_memory_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            logger.error(f"Failed to save path memory: {e}")
    
    def check_path_memory(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Check if we have a successful path in memory"""
        threshold = 200  # 20cm threshold
        
        for pm in self.path_memory:
            start_dist = math.sqrt((pm.start[0] - start[0])**2 + (pm.start[1] - start[1])**2)
            goal_dist = math.sqrt((pm.goal[0] - goal[0])**2 + (pm.goal[1] - goal[1])**2)
            
            if start_dist < threshold and goal_dist < threshold:
                # Found similar path
                pm.success_count += 1
                return pm.path
        
        return None
    
    def store_path_memory(self, start: Tuple[float, float], goal: Tuple[float, float], 
                         path: List[Tuple[float, float]]):
        """Store successful path in memory"""
        self.path_memory.append(PathMemory(
            start=start,
            goal=goal,
            path=path,
            timestamp=time.time()
        ))
        
        # Keep memory size limited
        if len(self.path_memory) > self.max_path_memory:
            self.path_memory.pop(0)
        
        # Save periodically
        if len(self.path_memory) % 10 == 0:
            self.save_path_memory()
    
    def calculate_mapping_metrics(self) -> Dict[str, Any]:
        """Calculate mapping performance metrics"""
        if not self.start_time:
            return {}
        
        duration = time.time() - self.start_time
        
        # Count explored cells
        explored_cells = np.sum(self.occupancy_grid != CellType.UNKNOWN.value)
        total_cells = self.grid_size * self.grid_size
        coverage_percentage = (explored_cells / total_cells) * 100
        
        # Calculate efficiency
        if self.total_distance_traveled > 0:
            self.mapping_efficiency = explored_cells / (self.total_distance_traveled / 1000)  # cells per meter
        
        return {
            "duration": round(duration, 1),
            "distance_traveled": round(self.total_distance_traveled / 1000, 2),  # meters
            "coverage_percentage": round(coverage_percentage, 2),
            "explored_cells": explored_cells,
            "frontiers_found": len(self.frontiers),
            "mapping_efficiency": round(self.mapping_efficiency, 2),
            "decisions_made": len(self.decision_log),
            "paths_memorized": len(self.path_memory)
        }
    
    def start_mapping(self) -> bool:
        """Start enhanced autonomous mapping"""
        with self._state_lock:
            if self.mapping_active:
                logger.warning("Mapping already active")
                return False
            
            self.mapping_active = True
            self.emergency_stop = False
            self.state = RobotState.IDLE
            self.start_time = time.time()
            
            # Reset everything
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_theta = 0.0
            self.total_distance_traveled = 0.0
            
            # Clear grids
            self.occupancy_grid.fill(CellType.UNKNOWN.value)
            self.confidence_grid.fill(0)
            self.visit_count_grid.fill(0)
            
            # Clear exploration data
            self.frontiers.clear()
            self.current_frontier = None
            self.frontier_blacklist.clear()
            self.position_history.clear()
            self.decision_log.clear()
            
            # Calibrate background
            if not self.calibrate_background_enhanced():
                logger.warning("Background calibration failed, continuing without it")
            
            # Start mapping thread
            self._mapping_thread = threading.Thread(target=self._enhanced_mapping_loop)
            self._mapping_thread.daemon = True
            self._mapping_thread.start()
            
            self.log_decision("mapping_started", {
                "grid_size": self.grid_size,
                "grid_resolution": self.grid_resolution,
                "safety_distance": self.safety_distance
            })
            
            logger.info("Started enhanced autonomous mapping")
            return True
    
    def _enhanced_mapping_loop(self):
        """Enhanced main mapping loop with frontier-based exploration"""
        logger.info("Enhanced mapping loop started")
        
        # Initial scan
        time.sleep(2.0)
        self._perform_360_scan()
        
        while self.mapping_active and not self.emergency_stop:
            try:
                # Get latest scan
                scan_data = self.lidar_sensor.get_latest_scan()
                
                if not scan_data:
                    logger.warning("No lidar data available")
                    time.sleep(0.1)
                    continue
                
                # Filter static obstacles
                filtered_scan = self.filter_static_obstacles_enhanced(scan_data)
                
                # Update occupancy grid
                self.update_occupancy_grid(filtered_scan)
                
                # Update position history for stuck detection
                self.position_history.append((self.robot_x, self.robot_y))
                
                # Check if stuck
                if self.check_if_stuck():
                    self.escape_stuck_situation()
                    continue
                
                # Update frontiers periodically
                current_time = time.time()
                if current_time - self.last_frontier_update > self.frontier_update_interval:
                    self.frontiers = self.detect_frontiers()
                    self.last_frontier_update = current_time
                
                # State machine
                if self.state == RobotState.IDLE:
                    # Select new frontier to explore
                    self.current_frontier = self.select_best_frontier()
                    if self.current_frontier:
                        self.state = RobotState.MOVING_TO_FRONTIER
                        self.log_decision("state_change", {
                            "from": "idle",
                            "to": "moving_to_frontier",
                            "frontier_size": self.current_frontier.size
                        })
                    else:
                        # No frontiers - try rotating to find new areas
                        self.state = RobotState.ROTATING_SCAN
                        self.log_decision("state_change", {
                            "from": "idle",
                            "to": "rotating_scan",
                            "reason": "no_frontiers"
                        })
                
                elif self.state == RobotState.MOVING_TO_FRONTIER:
                    if self.current_frontier:
                        # Check if reached frontier
                        dist_to_frontier = math.sqrt(
                            (self.robot_x - self.current_frontier.centroid[0])**2 +
                            (self.robot_y - self.current_frontier.centroid[1])**2
                        )
                        
                        if dist_to_frontier < 300:  # Reached frontier
                            self.state = RobotState.EXPLORING
                            self.log_decision("frontier_reached", {
                                "frontier_centroid": self.current_frontier.centroid,
                                "distance": round(dist_to_frontier, 1)
                            })
                        else:
                            # Move towards frontier
                            self._move_towards_frontier(filtered_scan)
                    else:
                        self.state = RobotState.IDLE
                
                elif self.state == RobotState.EXPLORING:
                    # Wall following exploration
                    action, left_speed, right_speed = self.enhanced_wall_following(filtered_scan)
                    
                    if action == "turn_right":
                        self._set_motors("forward", left_speed, "backward", right_speed)
                    elif action == "turn_left":
                        self._set_motors("backward", left_speed, "forward", right_speed)
                    elif action == "reverse":
                        self._set_motors("backward", left_speed, "backward", right_speed)
                    else:
                        self._set_motors("forward", left_speed, "forward", right_speed)
                    
                    # Check if should switch to new frontier
                    if current_time - self.last_state_change > 10.0:
                        self.state = RobotState.IDLE
                
                elif self.state == RobotState.ROTATING_SCAN:
                    # Perform 360 degree scan
                    self._perform_360_scan()
                    self.state = RobotState.IDLE
                
                elif self.state == RobotState.OBSTACLE_AVOIDANCE:
                    self._enhanced_obstacle_avoidance(filtered_scan)
                
                elif self.state == RobotState.EMERGENCY_STOP:
                    self._emergency_stop()
                    if self.emergency_stop:
                        break
                
                # Update odometry
                self._enhanced_update_odometry()
                
                # Log metrics periodically
                if int(current_time) % 10 == 0:
                    metrics = self.calculate_mapping_metrics()
                    self.log_decision("metrics_update", metrics)
                
                time.sleep(0.05)
                
            except Exception as e:
                logger.error(f"Error in mapping loop: {e}")
                self._emergency_stop()
                break
        
        # Cleanup
        self.motor_controller.stop_all_motors()
        self.save_path_memory()
        
        # Final metrics
        final_metrics = self.calculate_mapping_metrics()
        self.log_decision("mapping_completed", final_metrics)
        
        logger.info("Enhanced mapping loop ended")
    
    def _move_towards_frontier(self, scan_data: Dict[str, Any]):
        """Move towards selected frontier with obstacle avoidance"""
        if not self.current_frontier:
            return
        
        # Calculate angle to frontier
        dx = self.current_frontier.centroid[0] - self.robot_x
        dy = self.current_frontier.centroid[1] - self.robot_y
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.robot_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Check for obstacles in path
        front_dist = self._get_wall_distance(scan_data, 0)
        
        if front_dist and front_dist < self.safety_distance:
            # Obstacle in path - switch to wall following
            self.state = RobotState.EXPLORING
            self.log_decision("obstacle_in_path", {
                "front_distance": round(front_dist, 1),
                "switching_to": "exploring"
            })
            return
        
        # Turn towards frontier if needed
        if abs(angle_diff) > 0.2:  # ~11 degrees
            if angle_diff > 0:
                self._set_motors("backward", self.turn_speed, "forward", self.turn_speed)
            else:
                self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
        else:
            # Move forward
            self._set_motors("forward", self.exploration_speed, "forward", self.exploration_speed)
    
    def _perform_360_scan(self):
        """Perform 360 degree scan to update map"""
        self.log_decision("360_scan", {"reason": "exploration"})
        
        # Rotate slowly while scanning
        self._set_motors("forward", self.slow_speed, "backward", self.slow_speed)
        
        start_angle = self.robot_theta
        while abs(self.robot_theta - start_angle) < 2 * math.pi - 0.1:
            scan_data = self.lidar_sensor.get_latest_scan()
            if scan_data:
                filtered_scan = self.filter_static_obstacles_enhanced(scan_data)
                self.update_occupancy_grid(filtered_scan)
            
            time.sleep(0.1)
            self._enhanced_update_odometry()
        
        self.motor_controller.stop_all_motors()
        time.sleep(0.5)
    
    def _enhanced_obstacle_avoidance(self, scan_data: Dict[str, Any]):
        """Enhanced obstacle avoidance with better decision making"""
        # Stop first
        self.motor_controller.stop_all_motors()
        time.sleep(0.2)
        
        # Analyze escape routes
        left_clear = self._check_direction_clear(scan_data, 90)
        right_clear = self._check_direction_clear(scan_data, -90)
        left_front_clear = self._check_direction_clear(scan_data, 45)
        right_front_clear = self._check_direction_clear(scan_data, -45)
        
        self.log_decision("obstacle_avoidance", {
            "left_clear": left_clear,
            "right_clear": right_clear,
            "left_front_clear": left_front_clear,
            "right_front_clear": right_front_clear
        })
        
        # Decision logic
        if right_clear and right_front_clear:
            # Turn right
            self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
            time.sleep(0.8)
        elif left_clear and left_front_clear:
            # Turn left
            self._set_motors("backward", self.turn_speed, "forward", self.turn_speed)
            time.sleep(0.8)
        elif right_clear:
            # Sharp right turn
            self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
            time.sleep(1.2)
        elif left_clear:
            # Sharp left turn
            self._set_motors("backward", self.turn_speed, "forward", self.turn_speed)
            time.sleep(1.2)
        else:
            # Reverse and turn
            self._set_motors("backward", self.slow_speed, "backward", self.slow_speed)
            time.sleep(1.0)
            self._set_motors("forward", self.turn_speed, "backward", self.turn_speed)
            time.sleep(1.0)
        
        self.state = RobotState.EXPLORING
    
    def _enhanced_update_odometry(self):
        """Enhanced odometry update with simple Kalman filtering"""
        current_time = time.time()
        dt = current_time - self.last_odometry_update
        self.last_odometry_update = current_time
        
        # Get motor states
        motor_states = self.motor_controller.get_all_motor_states()
        
        # Calculate velocities
        left_velocity = 0
        right_velocity = 0
        
        if motor_states[1]["direction"] == "forward":
            left_velocity = motor_states[1]["speed"] * 10
        elif motor_states[1]["direction"] == "backward":
            left_velocity = -motor_states[1]["speed"] * 10
        
        if motor_states[0]["direction"] == "forward":
            right_velocity = motor_states[0]["speed"] * 10
        elif motor_states[0]["direction"] == "backward":
            right_velocity = -motor_states[0]["speed"] * 10
        
        # Calculate robot motion
        v = (left_velocity + right_velocity) / 2
        w = (right_velocity - left_velocity) / self.wheelbase
        
        # Store previous position for distance calculation
        prev_x = self.robot_x
        prev_y = self.robot_y
        
        # Update position
        if abs(w) < 0.001:
            self.robot_x += v * dt * math.cos(self.robot_theta)
            self.robot_y += v * dt * math.sin(self.robot_theta)
        else:
            self.robot_x += (v / w) * (math.sin(self.robot_theta + w * dt) - math.sin(self.robot_theta))
            self.robot_y += (v / w) * (-math.cos(self.robot_theta + w * dt) + math.cos(self.robot_theta))
            self.robot_theta += w * dt
        
        # Normalize angle
        self.robot_theta = self.robot_theta % (2 * math.pi)
        
        # Update distance traveled
        distance = math.sqrt((self.robot_x - prev_x)**2 + (self.robot_y - prev_y)**2)
        self.total_distance_traveled += distance
        
        # Update grid position
        grid_x = int(self.robot_x / self.grid_resolution + self.grid_size // 2)
        grid_y = int(self.robot_y / self.grid_resolution + self.grid_size // 2)
        
        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            self.visit_count_grid[grid_y, grid_x] += 1
    
    def calibrate_background_enhanced(self) -> bool:
        """Enhanced background calibration using statistical methods"""
        logger.info("Enhanced background calibration starting...")
        
        # Collect samples
        samples = []
        for i in range(30):
            scan_data = self.lidar_sensor.get_latest_scan()
            if scan_data and scan_data.get("points"):
                samples.append(scan_data["points"])
                time.sleep(0.05)
        
        if len(samples) < 20:
            logger.warning("Not enough samples for calibration")
            return False
        
        # Analyze each angle
        angle_data = {}
        
        for scan in samples:
            for point in scan:
                angle_deg = int(math.degrees(point.get("theta", point.get("angle", 0))))
                distance = point.get("r", point.get("distance", 0))
                
                if angle_deg not in angle_data:
                    angle_data[angle_deg] = []
                angle_data[angle_deg].append(distance)
        
        # Build statistical model
        self.background_model = {}
        
        for angle_deg, distances in angle_data.items():
            if len(distances) >= len(samples) * 0.8:
                # Calculate statistics
                mean_dist = np.mean(distances)
                std_dist = np.std(distances)
                
                # Store model
                self.background_model[angle_deg] = {
                    "mean": mean_dist,
                    "std": std_dist,
                    "min": mean_dist - self.outlier_threshold * std_dist,
                    "max": mean_dist + self.outlier_threshold * std_dist
                }
        
        logger.info(f"Background calibration complete. Modeled {len(self.background_model)} angles")
        
        self.log_decision("background_calibration", {
            "samples": len(samples),
            "angles_modeled": len(self.background_model)
        })
        
        return True
    
    def filter_static_obstacles_enhanced(self, scan_data: Dict[str, Any]) -> Dict[str, Any]:
        """Enhanced filtering using statistical outlier removal"""
        if not self.background_model:
            return scan_data
        
        filtered_points = []
        removed_count = 0
        
        for point in scan_data.get("points", []):
            angle_deg = int(math.degrees(point.get("theta", point.get("angle", 0))))
            distance = point.get("r", point.get("distance", 0))
            
            # Check against model
            keep_point = True
            
            # Check exact angle and neighbors
            for check_angle in range(angle_deg - 2, angle_deg + 3):
                if check_angle in self.background_model:
                    model = self.background_model[check_angle]
                    
                    # Check if within expected range
                    if model["min"] <= distance <= model["max"]:
                        keep_point = False
                        removed_count += 1
                        break
            
            # Additional dynamic filtering
            if keep_point and len(self.static_scan_buffer) > 10:
                # Check temporal consistency
                similar_count = 0
                for prev_scan in self.static_scan_buffer:
                    for prev_point in prev_scan:
                        prev_angle = math.degrees(prev_point.get("theta", prev_point.get("angle", 0)))
                        prev_dist = prev_point.get("r", prev_point.get("distance", 0))
                        
                        if abs(prev_angle - angle_deg) < 3 and abs(prev_dist - distance) < self.dynamic_threshold:
                            similar_count += 1
                            break
                
                # If point appears consistently, it might be static
                if similar_count > len(self.static_scan_buffer) * 0.8:
                    keep_point = False
                    removed_count += 1
            
            if keep_point:
                filtered_points.append(point)
        
        # Update buffer
        self.static_scan_buffer.append(filtered_points)
        
        # Create filtered scan data
        filtered_data = scan_data.copy()
        filtered_data["points"] = filtered_points
        filtered_data["point_count"] = len(filtered_points)
        
        if filtered_points:
            min_dist = min(p.get("r", p.get("distance", float('inf'))) for p in filtered_points)
            filtered_data["min_distance"] = min_dist
        
        return filtered_data
    
    def get_status(self) -> Dict[str, Any]:
        """Get enhanced mapping status"""
        with self._state_lock:
            metrics = self.calculate_mapping_metrics()
            
            return {
                "active": self.mapping_active,
                "state": self.state.value,
                "emergency_stop": self.emergency_stop,
                "robot_position": {
                    "x": round(self.robot_x),
                    "y": round(self.robot_y),
                    "theta": round(math.degrees(self.robot_theta), 1)
                },
                "frontiers": len(self.frontiers),
                "current_frontier": {
                    "size": self.current_frontier.size if self.current_frontier else 0,
                    "distance": round(self.current_frontier.distance, 1) if self.current_frontier else 0
                },
                **metrics
            }
    
    def stop_mapping(self) -> bool:
        """Stop enhanced mapping and save results"""
        with self._state_lock:
            if not self.mapping_active:
                return False
            
            self.mapping_active = False
            self.emergency_stop = True
        
        # Stop motors
        self.motor_controller.stop_all_motors()
        
        # Wait for thread
        if self._mapping_thread:
            self._mapping_thread.join(timeout=5.0)
        
        # Save data
        self.save_path_memory()
        self.save_mapping_results()
        
        logger.info("Stopped enhanced autonomous mapping")
        return True
    
    def save_mapping_results(self):
        """Save mapping results to file"""
        try:
            results = {
                "timestamp": datetime.now().isoformat(),
                "metrics": self.calculate_mapping_metrics(),
                "decision_count": len(self.decision_log),
                "paths_memorized": len(self.path_memory),
                "frontiers_explored": len(self.frontier_blacklist)
            }
            
            with open("mapping_results.json", 'w') as f:
                json.dump(results, f, indent=2)
            
            # Save occupancy grid
            np.save("occupancy_grid.npy", self.occupancy_grid)
            np.save("confidence_grid.npy", self.confidence_grid)
            np.save("visit_count_grid.npy", self.visit_count_grid)
            
            logger.info("Saved mapping results")
        except Exception as e:
            logger.error(f"Failed to save mapping results: {e}")
    
    # Keep existing helper methods from original code
    def _set_motors(self, left_dir: str, left_speed: float, 
                    right_dir: str, right_speed: float):
        """Set motor speeds accounting for reversal"""
        self.motor_controller.set_motor(0, left_dir, left_speed)
        time.sleep(self.command_delay)
        self.motor_controller.set_motor(1, right_dir, right_speed)
    
    def _get_wall_distance(self, scan_data: Dict[str, Any], angle_deg: float) -> Optional[float]:
        """Get distance to wall at specific angle"""
        points = scan_data.get("points", [])
        if not points:
            return None
        
        target_angle = math.radians(angle_deg)
        angle_tolerance = math.radians(15)
        
        valid_distances = []
        for point in points:
            point_angle = point.get("theta", point.get("angle", 0))
            angle_diff = abs(point_angle - target_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff <= angle_tolerance:
                distance = point.get("r", point.get("distance", 0))
                if distance > 0.1 and distance < 10.0:
                    valid_distances.append(distance * 1000)
        
        if valid_distances:
            return np.median(valid_distances)
        return None
    
    def _check_direction_clear(self, scan_data: Dict[str, Any], angle_deg: float) -> bool:
        """Check if direction is clear for movement"""
        distance = self._get_wall_distance(scan_data, angle_deg)
        return distance is None or distance > self.safety_distance
    
    def _emergency_stop(self):
        """Execute emergency stop"""
        logger.warning("Emergency stop activated!")
        self.motor_controller.stop_all_motors()
        self.state = RobotState.EMERGENCY_STOP
        
        self.log_decision("emergency_stop", {
            "reason": "safety",
            "position": (self.robot_x, self.robot_y)
        })
        
        time.sleep(1.0)
        
        if self.mapping_active:
            # Try to recover
            self.state = RobotState.ROTATING_SCAN
            self.emergency_stop = False
            logger.info("Attempting recovery from emergency stop")
    
    def reset_emergency_stop(self) -> bool:
        """Reset emergency stop state"""
        with self._state_lock:
            if self.emergency_stop:
                self.emergency_stop = False
                self.state = RobotState.IDLE
                logger.info("Emergency stop reset")
                return True
            return False
    
    def update_safety_parameters(self, critical: float, safety: float, wall_follow: float) -> bool:
        """Update safety parameters dynamically"""
        try:
            self.critical_distance = max(100, min(500, critical))
            self.safety_distance = max(200, min(800, safety))
            self.wall_follow_distance = max(300, min(1000, wall_follow))
            
            self.log_decision("safety_parameters_updated", {
                "critical": self.critical_distance,
                "safety": self.safety_distance,
                "wall_follow": self.wall_follow_distance
            })
            
            logger.info(f"Updated safety parameters - Critical: {self.critical_distance}mm, "
                       f"Safety: {self.safety_distance}mm, Wall follow: {self.wall_follow_distance}mm")
            return True
        except Exception as e:
            logger.error(f"Failed to update safety parameters: {e}")
            return False

# Singleton instance
_enhanced_auto_mapper = None

def get_enhanced_auto_mapper(motor_controller=None, lidar_sensor=None):
    """Get or create EnhancedAutoMapper instance"""
    global _enhanced_auto_mapper
    if _enhanced_auto_mapper is None and motor_controller and lidar_sensor:
        _enhanced_auto_mapper = EnhancedAutoMapper(motor_controller, lidar_sensor)
    return _enhanced_auto_mapper