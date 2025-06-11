#!/usr/bin/env python3
"""
Grid-based exploration with proper obstacle detection and motor control
"""

import sys
import time
import signal
import logging
import math
import random
import numpy as np
from pathlib import Path
from collections import deque

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from backend.ros_bridge import get_ros_bridge, get_motor_controller
from backend.sensors.lidar import get_lidar_sensor

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Configuration
ROBOT_IP = "192.168.124.39"
ROS_PORT = 9090

# Robot dimensions
ROBOT_WIDTH = 500  # mm
ROBOT_LENGTH = 500  # mm
SAFE_DISTANCE = 600  # mm - minimum distance to obstacles (60cm for safe turning)
CRITICAL_DISTANCE = 400  # mm - emergency stop distance (40cm)

# Speeds
TURN_SPEED = 90  # Increased for better single-wheel turning
FORWARD_SPEED = 80

# Grid mapping parameters
GRID_RESOLUTION = 50  # mm per grid cell
GRID_SIZE = 200  # 200x200 grid = 10m x 10m area
OBSTACLE_THRESHOLD = 0.7  # Probability threshold for obstacle

class GridMapper:
    """Simple occupancy grid mapper"""
    
    def __init__(self, size=GRID_SIZE, resolution=GRID_RESOLUTION):
        self.size = size
        self.resolution = resolution
        self.grid = np.ones((size, size)) * 0.5  # Initialize with unknown (0.5)
        self.robot_x = size // 2  # Start in center
        self.robot_y = size // 2
        self.robot_theta = 0
        
    def world_to_grid(self, x, y):
        """Convert world coordinates (mm) to grid indices"""
        grid_x = int(x / self.resolution) + self.size // 2
        grid_y = int(y / self.resolution) + self.size // 2
        return grid_x, grid_y
    
    def update_map(self, lidar_points, robot_x, robot_y, robot_theta):
        """Update occupancy grid with lidar data"""
        self.robot_x, self.robot_y = self.world_to_grid(robot_x, robot_y)
        self.robot_theta = robot_theta
        
        for point in lidar_points:
            r = point['r'] * 1000  # Convert to mm
            theta = point['theta'] + robot_theta
            
            # Point in world coordinates
            px = robot_x + r * math.cos(theta)
            py = robot_y + r * math.sin(theta)
            
            # Convert to grid
            gx, gy = self.world_to_grid(px, py)
            
            # Mark as obstacle if within grid bounds
            if 0 <= gx < self.size and 0 <= gy < self.size:
                self.grid[gy, gx] = min(0.9, self.grid[gy, gx] + 0.1)
            
            # Ray trace to mark free space
            self._ray_trace(self.robot_x, self.robot_y, gx, gy)
    
    def _ray_trace(self, x0, y0, x1, y1):
        """Mark cells as free along a ray"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while x != x1 or y != y1:
            if 0 <= x < self.size and 0 <= y < self.size:
                self.grid[y, x] = max(0.1, self.grid[y, x] - 0.05)
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def is_obstacle_ahead(self, distance=SAFE_DISTANCE):
        """Check if there's an obstacle ahead of the robot"""
        check_dist = int(distance / self.resolution)
        
        for d in range(1, check_dist):
            check_x = self.robot_x + int(d * math.cos(self.robot_theta))
            check_y = self.robot_y + int(d * math.sin(self.robot_theta))
            
            if 0 <= check_x < self.size and 0 <= check_y < self.size:
                if self.grid[check_y, check_x] > OBSTACLE_THRESHOLD:
                    return True, d * self.resolution
        
        return False, distance

class SmartExplorer:
    """Explorer with grid mapping and better control"""
    
    def __init__(self):
        """Initialize with grid mapper"""
        logger.info(f"Initializing - Robot IP: {ROBOT_IP}, Port: {ROS_PORT}")
        
        # ROS connection
        self.ros_bridge = get_ros_bridge(ROBOT_IP, ROS_PORT)
        if not self.ros_bridge.test_connection():
            logger.error("Failed to connect to ROS bridge!")
            sys.exit(1)
        
        logger.info("✓ Connected to ROS bridge")
        
        # Components
        self.motor_controller = get_motor_controller()
        self.lidar_sensor = get_lidar_sensor()
        self.grid_mapper = GridMapper()
        
        # Signal handler
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # State
        self.running = True
        self.exploring = False
        self._last_lidar_data = None
        self._last_motor_cmd = {'left': None, 'right': None}
        self._motor_cmd_time = 0
        
        # Robot pose
        self.x = 0.0  # mm
        self.y = 0.0  # mm
        self.theta = 0.0  # radians
        
        # Calibration
        self.robot_body_points = []
        self.calibration_complete = False
        
        logger.info("✓ All components initialized")
    
    def _signal_handler(self, sig, frame):
        """Handle shutdown"""
        logger.info("\nShutdown requested...")
        self.running = False
        self.exploring = False
        self.stop()
        sys.exit(0)
    
    def calibrate_robot_body(self):
        """Quick calibration to identify robot body points"""
        logger.info("\n=== Starting robot body calibration ===")
        
        calibration_points = []
        all_scan_data = []
        
        def calibration_callback(scan_data):
            if scan_data and 'points' in scan_data:
                points = []
                for p in scan_data['points']:
                    if p['r'] < 0.6:  # Only close points (< 600mm)
                        points.append({'r': p['r'], 'theta': p['theta']})
                calibration_points.append(points)
                all_scan_data.append(scan_data)
        
        if not self.lidar_sensor.subscribe(callback=calibration_callback, processed_data=True):
            logger.error("Failed to start lidar")
            return False
        
        logger.info("Rotating for calibration...")
        time.sleep(1)
        
        # Rotate slowly using one wheel only (no counter-rotation)
        self.set_motors(85, 0)  # Left wheel forward at 85%, right wheel stopped
        time.sleep(6)  # 6 seconds for full rotation
        self.stop_motors()
        
        # Analyze data
        if len(calibration_points) > 10:
            # Find consistent points
            all_points = []
            for scan in calibration_points[5:]:  # Skip first few
                all_points.extend(scan)
            
            # Simple clustering - points that appear frequently
            self.robot_body_points = []
            for p in all_points[:20]:  # Take first 20 close points
                self.robot_body_points.append(p)
            
            # Also update grid map with scan data from calibration
            logger.info("Updating grid map with calibration data...")
            for scan_data in all_scan_data[-5:]:  # Use last few scans
                filtered_points = []
                for point in scan_data.get('points', []):
                    if not self.is_robot_body(point):
                        filtered_points.append(point)
                self.grid_mapper.update_map(filtered_points, self.x, self.y, self.theta)
        
        self.lidar_sensor.unsubscribe()
        self.calibration_complete = True
        
        logger.info(f"✓ Calibration complete! Found {len(self.robot_body_points)} body points")
        return True
    
    def is_robot_body(self, point):
        """Check if point belongs to robot"""
        r = point['r']
        theta = point['theta']
        
        # Simple check - very close points behind or to sides
        if r < 0.3:  # Less than 300mm
            if abs(theta) > 2.0:  # Behind (more than ~115 degrees)
                return True
            if 0.5 < abs(theta) < 2.5:  # Sides
                return True
        
        # Check against calibrated points if available
        if self.calibration_complete:
            for bp in self.robot_body_points:
                if abs(r - bp['r']) < 0.1 and abs(theta - bp['theta']) < 0.2:
                    return True
        
        return False
    
    def set_motors(self, left_speed, right_speed):
        """Set motor speeds with rate limiting"""
        current_time = time.time()
        
        # Rate limit motor commands (max 5Hz)
        if current_time - self._motor_cmd_time < 0.2:
            return
        
        # Only send if changed
        if (self._last_motor_cmd['left'] != left_speed or 
            self._last_motor_cmd['right'] != right_speed):
            
            if left_speed == 0:
                self.motor_controller.set_motor(0, "brake", 0)
            elif left_speed < 0:
                self.motor_controller.set_motor(0, "backward", abs(left_speed))
            else:
                direction = "forward" if left_speed > 0 else "backward"
                self.motor_controller.set_motor(0, direction, abs(left_speed))
            
            if right_speed == 0:
                self.motor_controller.set_motor(1, "brake", 0)
            elif right_speed < 0:
                self.motor_controller.set_motor(1, "backward", abs(right_speed))
            else:
                direction = "forward" if right_speed > 0 else "backward"
                self.motor_controller.set_motor(1, direction, abs(right_speed))
            
            self._last_motor_cmd = {'left': left_speed, 'right': right_speed}
            self._motor_cmd_time = current_time
    
    def stop_motors(self):
        """Stop all motors"""
        self.set_motors(0, 0)
        self.motor_controller.stop_all_motors()
    
    def process_lidar_data(self, scan_data):
        """Process lidar data and update grid map"""
        if not scan_data or 'points' not in scan_data:
            return
        
        # Filter robot body points
        filtered_points = []
        for point in scan_data['points']:
            if not self.is_robot_body(point):
                filtered_points.append(point)
        
        # Update grid map
        self.grid_mapper.update_map(filtered_points, self.x, self.y, self.theta)
        
        # Store for other uses
        self._last_lidar_data = filtered_points
        
        # Check for obstacles in different directions
        front_clear = True
        min_front = float('inf')
        
        for point in filtered_points:
            r = point['r'] * 1000  # mm
            theta = point['theta']
            
            # Front cone (±30 degrees)
            if -0.52 < theta < 0.52:
                if r < min_front:
                    min_front = r
                if r < SAFE_DISTANCE:
                    front_clear = False
        
        return front_clear, min_front
    
    def explore(self, duration=60):
        """Main exploration routine"""
        logger.info(f"\n=== Starting EXPLORATION for {duration} seconds ===")
        
        # Calibrate if needed
        if not self.calibration_complete:
            self.calibrate_robot_body()
        
        # Start lidar
        front_clear = True
        min_front_dist = float('inf')
        
        def lidar_callback(scan_data):
            nonlocal front_clear, min_front_dist
            result = self.process_lidar_data(scan_data)
            if result:
                front_clear, min_front_dist = result
        
        if not self.lidar_sensor.subscribe(callback=lidar_callback, processed_data=True):
            logger.error("Failed to start lidar")
            return
        
        logger.info("✓ Lidar started")
        time.sleep(1)  # Let lidar stabilize
        
        # Exploration state machine
        start_time = time.time()
        last_log_time = 0
        state = "forward"
        turn_start = None
        turn_count = 0  # Track consecutive turns
        stuck_count = 0  # Track if we're stuck
        self.exploring = True
        
        # Movement tracking
        last_move_time = time.time()
        
        while self.running and self.exploring and (time.time() - start_time) < duration:
            current_time = time.time()
            dt = current_time - last_move_time
            last_move_time = current_time
            
            # Update position estimate (simple dead reckoning)
            if state == "forward":
                self.x += FORWARD_SPEED * dt * math.cos(self.theta)
                self.y += FORWARD_SPEED * dt * math.sin(self.theta)
            elif state == "backward":
                self.x -= FORWARD_SPEED * 0.8 * dt * math.cos(self.theta)  # Slower backward
                self.y -= FORWARD_SPEED * 0.8 * dt * math.sin(self.theta)
            elif state == "turn_left":
                self.theta += 0.5 * dt  # ~0.5 rad/s (slower turning)
            elif state == "turn_right":
                self.theta -= 0.5 * dt
            
            # Normalize angle
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Check grid map for obstacles
            grid_obstacle, grid_dist = self.grid_mapper.is_obstacle_ahead()
            
            # Status logging
            if current_time - last_log_time > 3.0:
                front_dist_str = "inf" if min_front_dist == float('inf') else f"{int(min_front_dist)}"
                logger.info(f"[{int(current_time - start_time)}s/{duration}s] "
                          f"State: {state} | "
                          f"Pos: ({int(self.x)}, {int(self.y)})mm, {int(math.degrees(self.theta))}° | "
                          f"Front: {front_dist_str}mm | "
                          f"Grid obstacle: {grid_obstacle} at {int(grid_dist)}mm")
                last_log_time = current_time
            
            # State machine
            if state == "forward":
                # Check both direct lidar and grid map
                if not front_clear or min_front_dist < SAFE_DISTANCE or grid_obstacle:
                    logger.warning(f"Obstacle detected! Lidar: {int(min_front_dist)}mm, "
                                 f"Grid: {grid_obstacle} at {int(grid_dist)}mm")
                    self.stop_motors()
                    time.sleep(0.3)  # Longer pause before turning
                    
                    # If very close, back up first
                    if min_front_dist < CRITICAL_DISTANCE:
                        state = "backward"
                        turn_start = current_time
                        stuck_count += 1
                        logger.warning(f"Too close ({int(min_front_dist)}mm)! Backing up...")
                    else:
                        # Choose turn direction - alternate if stuck
                        if stuck_count > 2:
                            state = "turn_right" if turn_count % 2 == 0 else "turn_left"
                        else:
                            state = "turn_left" if random.random() < 0.5 else "turn_right"
                        turn_start = current_time
                        turn_count += 1
                else:
                    self.set_motors(FORWARD_SPEED, FORWARD_SPEED)
                    turn_count = 0  # Reset turn count when moving forward
                    stuck_count = 0
            
            elif state == "backward":
                # Move backward for longer when stuck
                self.set_motors(-FORWARD_SPEED, -FORWARD_SPEED)  # Both wheels backward
                
                # Back up longer if we're really stuck
                backup_duration = 2.0 if stuck_count > 2 else 1.5
                
                if current_time - turn_start > backup_duration:
                    self.stop_motors()
                    time.sleep(0.2)
                    # After backing up, turn
                    state = "turn_right" if random.random() < 0.5 else "turn_left"
                    turn_start = current_time
                    logger.info(f"Backed up for {backup_duration}s, now turning...")
            
            elif state in ["turn_left", "turn_right"]:
                if state == "turn_left":
                    # Turn left by moving only left wheel
                    self.set_motors(TURN_SPEED, 0)
                else:
                    # Turn right by moving only right wheel
                    self.set_motors(0, TURN_SPEED)
                
                # Turn for longer if we've been stuck
                turn_duration = random.uniform(2.0, 3.0) if stuck_count > 2 else random.uniform(1.5, 2.5)
                
                if current_time - turn_start > turn_duration:
                    self.stop_motors()
                    time.sleep(0.2)
                    state = "forward"
                    logger.info("Turn complete, checking path...")
            
            time.sleep(0.05)  # 20Hz control loop
        
        # Cleanup
        self.exploring = False
        self.stop_motors()
        self.lidar_sensor.unsubscribe()
        
        logger.info(f"\n✓ Exploration completed!")
        logger.info(f"Distance traveled: ~{int(math.sqrt(self.x**2 + self.y**2))}mm")
    
    def stop(self):
        """Clean shutdown"""
        logger.info("Stopping all systems...")
        self.exploring = False
        
        if hasattr(self, 'motor_controller'):
            try:
                self.motor_controller.stop_all_motors()
            except:
                pass
        
        if hasattr(self, 'lidar_sensor'):
            try:
                self.lidar_sensor.unsubscribe()
            except:
                pass
        
        logger.info("✓ All systems stopped")

def main():
    """Main function"""
    print("=== Smart Grid-Based Explorer ===")
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Safe distance: {SAFE_DISTANCE}mm")
    print(f"Grid resolution: {GRID_RESOLUTION}mm/cell")
    print()
    
    try:
        explorer = SmartExplorer()
    except Exception as e:
        logger.error(f"Failed to initialize: {e}")
        return
    
    while True:
        print("\n=== OPTIONS ===")
        print("1. Quick exploration (20 seconds)")
        print("2. Standard exploration (60 seconds)")
        print("3. Extended exploration (120 seconds)")
        print("0. Exit")
        
        choice = input("\nSelect option: ").strip()
        
        try:
            if choice == '0':
                break
            elif choice == '1':
                explorer.explore(20)
            elif choice == '2':
                explorer.explore(60)
            elif choice == '3':
                explorer.explore(120)
            else:
                print("Invalid option")
        
        except KeyboardInterrupt:
            logger.info("\nInterrupted by user")
        except Exception as e:
            logger.error(f"Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            explorer.stop_motors()
            time.sleep(0.5)
    
    explorer.stop()
    print("\nDemo completed!")

if __name__ == "__main__":
    main()