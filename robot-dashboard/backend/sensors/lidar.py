#!/usr/bin/env python3
"""
Lidar sensor module for ROS 2
Handles lidar data subscription and processing
"""

import json
import logging
import threading
import time
from typing import Dict, Any, Optional, Callable, List
import numpy as np
from websocket import create_connection

logger = logging.getLogger(__name__)

class LidarSensor:
    """Subscribe to and process lidar scan data via ROSBridge"""
    
    def __init__(self, ros_bridge):
        """
        Initialize lidar sensor
        
        Args:
            ros_bridge: ROSBridgeConnection instance
        """
        self.ros_bridge = ros_bridge
        self.scan_callback = None
        self.latest_scan = None
        self.subscription_active = False
        self._ws = None
        self._ws_thread = None
        self._stop_thread = False
        
    def subscribe(self, callback: Optional[Callable] = None, 
                 processed_data: bool = True) -> bool:
        """
        Subscribe to /scan topic
        
        Args:
            callback: Optional callback function for new scan data
            processed_data: If True, return processed data; if False, return raw
        
        Returns:
            bool: Success status
        """
        try:
            # Create a dedicated connection for subscription
            self._ws = create_connection(self.ros_bridge.url, timeout=5)
            
            # Subscribe message
            subscribe_msg = {
                "op": "subscribe",
                "topic": "/scan",
                "type": "sensor_msgs/LaserScan"
            }
            
            self._ws.send(json.dumps(subscribe_msg))
            logger.info("Subscribed to /scan topic")
            
            # Start receiving thread
            self.subscription_active = True
            self._stop_thread = False
            self._ws_thread = threading.Thread(
                target=self._receive_scan_data, 
                args=(callback, processed_data)
            )
            self._ws_thread.daemon = True
            self._ws_thread.start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to subscribe to scan: {e}")
            return False
    
    def _receive_scan_data(self, callback, processed_data):
        """Receive scan data in separate thread"""
        while self.subscription_active and not self._stop_thread:
            try:
                response = self._ws.recv()
                data = json.loads(response)
                
                if data.get("op") == "publish" and data.get("topic") == "/scan":
                    msg = data["msg"]
                    
                    if processed_data:
                        # Process the data for easier use
                        processed = self._process_scan_data(msg)
                        self.latest_scan = processed
                        
                        if callback:
                            callback(processed)
                    else:
                        # Return raw data
                        self.latest_scan = msg
                        
                        if callback:
                            callback(msg)
                            
            except Exception as e:
                if self.subscription_active:
                    logger.error(f"Error receiving scan data: {e}")
                break
        
        if self._ws:
            self._ws.close()
        logger.info("Scan subscription thread ended")
    
    def _process_scan_data(self, scan_msg: Dict) -> Dict[str, Any]:
        """
        Process raw scan data into more useful format
        
        Args:
            scan_msg: Raw LaserScan message
            
        Returns:
            Dict with processed data including points for visualization
        """
        ranges = scan_msg["ranges"]
        angle_min = scan_msg["angle_min"]
        angle_increment = scan_msg["angle_increment"]
        
        # Filter out invalid readings (0.0 or inf)
        valid_ranges = [(i, r) for i, r in enumerate(ranges) 
                       if r > 0.1 and r < 10.0]
        
        if not valid_ranges:
            return {
                "min_distance": float('inf'),
                "min_angle": 0,
                "obstacles": [],
                "safe_directions": ["all"],
                "points": [],
                "timestamp": time.time(),
                "point_count": 0
            }
        
        # Find minimum distance
        min_idx, min_dist = min(valid_ranges, key=lambda x: x[1])
        min_angle = angle_min + min_idx * angle_increment
        
        # Convert to cartesian coordinates for visualization
        points = []
        obstacles = []
        
        for idx, dist in valid_ranges:
            angle = angle_min + idx * angle_increment
            x = dist * np.cos(angle)
            y = dist * np.sin(angle)
            
            points.append({
                "x": float(x),
                "y": float(y),
                "r": float(dist),
                "theta": float(angle),
                "theta_deg": float(np.degrees(angle))
            })
            
            # Detect obstacles (anything closer than 0.5m)
            if dist < 0.5:
                obstacles.append({
                    "angle": float(angle),
                    "angle_deg": float(np.degrees(angle)),
                    "distance": float(dist),
                    "x": float(x),
                    "y": float(y)
                })
        
        # Determine safe directions
        safe_directions = self._calculate_safe_directions(valid_ranges, angle_min, angle_increment)
        
        return {
            "min_distance": float(min_dist),
            "min_angle": float(min_angle),
            "min_angle_deg": float(np.degrees(min_angle)),
            "obstacles": obstacles,
            "safe_directions": safe_directions,
            "obstacle_count": len(obstacles),
            "points": points,
            "point_count": len(points),
            "timestamp": time.time()
        }
    
    def _calculate_safe_directions(self, valid_ranges, angle_min, angle_increment):
        """Calculate safe movement directions based on scan data"""
        safe_directions = []
        
        # Check sectors (front, left, right, back)
        sectors = {
            "front": (-30, 30),
            "left": (60, 120),
            "right": (-120, -60),
            "back": (150, 180)  # and (-180, -150)
        }
        
        for direction, (start_deg, end_deg) in sectors.items():
            start_rad = np.radians(start_deg)
            end_rad = np.radians(end_deg)
            
            # Check if sector is clear
            sector_clear = True
            for idx, dist in valid_ranges:
                angle = angle_min + idx * angle_increment
                
                # Handle back sector wraparound
                if direction == "back":
                    if (angle > np.radians(150) or angle < np.radians(-150)):
                        if dist < 0.5:
                            sector_clear = False
                            break
                else:
                    if start_rad <= angle <= end_rad and dist < 0.5:
                        sector_clear = False
                        break
            
            if sector_clear:
                safe_directions.append(direction)
        
        return safe_directions
    
    def get_latest_scan(self) -> Optional[Dict[str, Any]]:
        """Get the most recent scan data"""
        return self.latest_scan
    
    def unsubscribe(self):
        """Stop subscription"""
        self.subscription_active = False
        self._stop_thread = True
        if self._ws_thread:
            self._ws_thread.join(timeout=2)
        logger.info("Unsubscribed from scan topic")

# Singleton instance
_lidar_sensor = None

def get_lidar_sensor():
    """Get or create LidarSensor instance"""
    global _lidar_sensor
    if _lidar_sensor is None:
        from backend.ros_bridge import get_ros_bridge
        _lidar_sensor = LidarSensor(get_ros_bridge())
    return _lidar_sensor