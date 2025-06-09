#!/usr/bin/env python3
"""
Thermal camera sensor module for MLX90640
Handles thermal data subscription and processing
"""

import json
import logging
import threading
import time
from typing import Dict, Any, Optional, Callable
import numpy as np
from websocket import create_connection

logger = logging.getLogger(__name__)

class ThermalCameraSensor:
    """Subscribe to and process thermal camera data via ROSBridge"""
    
    def __init__(self, ros_bridge):
        """
        Initialize thermal camera sensor
        
        Args:
            ros_bridge: ROSBridgeConnection instance
        """
        self.ros_bridge = ros_bridge
        self.thermal_callback = None
        self.latest_frame = None
        self.subscription_active = False
        self._ws = None
        self._ws_thread = None
        self._stop_thread = False
        
    def subscribe(self, callback: Optional[Callable] = None) -> bool:
        """
        Subscribe to /thermal_frame topic
        
        Args:
            callback: Optional callback function for new thermal data
        
        Returns:
            bool: Success status
        """
        try:
            # Create a dedicated connection for subscription
            self._ws = create_connection(self.ros_bridge.url, timeout=5)
            
            # Subscribe message
            subscribe_msg = {
                "op": "subscribe",
                "topic": "/thermal_frame",
                "type": "mlx90640_interfaces/ThermalFrame"
            }
            
            self._ws.send(json.dumps(subscribe_msg))
            logger.info("Subscribed to /thermal_frame topic")
            
            # Start receiving thread
            self.subscription_active = True
            self._stop_thread = False
            self._ws_thread = threading.Thread(
                target=self._receive_thermal_data, 
                args=(callback,)
            )
            self._ws_thread.daemon = True
            self._ws_thread.start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to subscribe to thermal camera: {e}")
            return False
    
    def _receive_thermal_data(self, callback):
        """Receive thermal data in separate thread"""
        while self.subscription_active and not self._stop_thread:
            try:
                response = self._ws.recv()
                data = json.loads(response)
                
                if data.get("op") == "publish" and data.get("topic") == "/thermal_frame":
                    msg = data["msg"]
                    
                    # Process the thermal frame data
                    processed = self._process_thermal_data(msg)
                    self.latest_frame = processed
                    
                    if callback:
                        callback(processed)
                        
            except Exception as e:
                if self.subscription_active:
                    logger.error(f"Error receiving thermal data: {e}")
                break
        
        if self._ws:
            self._ws.close()
        logger.info("Thermal camera subscription thread ended")
    
    def _process_thermal_data(self, frame_msg: Dict) -> Dict[str, Any]:
        """
        Process raw thermal frame data for visualization
        
        Args:
            frame_msg: Raw ThermalFrame message
            
        Returns:
            Dict with processed data including heatmap
        """
        try:
            # Extract frame data
            width = frame_msg.get("width", 32)
            height = frame_msg.get("height", 24)
            data = frame_msg.get("data", [])
            
            # Reshape to 2D array
            thermal_array = np.array(data).reshape((height, width))
            
            # Calculate temperature ranges for better visualization
            min_temp = frame_msg.get("min_temp", 20.0)
            max_temp = frame_msg.get("max_temp", 30.0)
            avg_temp = frame_msg.get("avg_temp", 25.0)
            center_temp = frame_msg.get("center_temp", 25.0)
            
            # Normalize data to 0-255 for color mapping
            if max_temp > min_temp:
                normalized = ((thermal_array - min_temp) / (max_temp - min_temp) * 255).astype(np.uint8)
            else:
                normalized = np.full((height, width), 128, dtype=np.uint8)
            
            # Find hotspots (temperatures above average + threshold)
            threshold = (max_temp - min_temp) * 0.7 + min_temp
            hotspots = []
            for y in range(height):
                for x in range(width):
                    if thermal_array[y, x] > threshold:
                        hotspots.append({
                            "x": x,
                            "y": y,
                            "temp": float(thermal_array[y, x])
                        })
            
            # Convert to list for JSON serialization
            thermal_list = thermal_array.tolist()
            normalized_list = normalized.tolist()
            
            return {
                "width": width,
                "height": height,
                "thermal_data": thermal_list,
                "normalized_data": normalized_list,
                "min_temp": min_temp,
                "max_temp": max_temp,
                "avg_temp": avg_temp,
                "center_temp": center_temp,
                "frame_count": frame_msg.get("frame_count", 0),
                "hotspots": hotspots,
                "hotspot_count": len(hotspots),
                "timestamp": time.time()
            }
            
        except Exception as e:
            logger.error(f"Error processing thermal data: {e}")
            return {
                "width": 32,
                "height": 24,
                "thermal_data": [[20.0] * 32 for _ in range(24)],
                "normalized_data": [[128] * 32 for _ in range(24)],
                "min_temp": 20.0,
                "max_temp": 30.0,
                "avg_temp": 25.0,
                "center_temp": 25.0,
                "frame_count": 0,
                "hotspots": [],
                "hotspot_count": 0,
                "timestamp": time.time()
            }
    
    def get_latest_frame(self) -> Optional[Dict[str, Any]]:
        """Get the most recent thermal frame data"""
        return self.latest_frame
    
    def get_thermal_frame_once(self) -> Optional[Dict[str, Any]]:
        """Request a single thermal frame via service call"""
        try:
            # Call the get_thermal_frame service
            success, response = self.ros_bridge.call_service(
                "/get_thermal_frame",
                {},
                "get_thermal_frame"
            )
            
            if success and response.get("values", {}).get("success"):
                frame_data = response["values"]["frame"]
                return self._process_thermal_data(frame_data)
            else:
                logger.error("Failed to get thermal frame via service")
                return None
                
        except Exception as e:
            logger.error(f"Error calling thermal frame service: {e}")
            return None
    
    def unsubscribe(self):
        """Stop subscription"""
        self.subscription_active = False
        self._stop_thread = True
        if self._ws_thread:
            self._ws_thread.join(timeout=2)
        logger.info("Unsubscribed from thermal camera topic")

# Singleton instance
_thermal_camera_sensor = None

def get_thermal_camera_sensor():
    """Get or create ThermalCameraSensor instance"""
    global _thermal_camera_sensor
    if _thermal_camera_sensor is None:
        from backend.ros_bridge import get_ros_bridge
        _thermal_camera_sensor = ThermalCameraSensor(get_ros_bridge())
    return _thermal_camera_sensor