#!/usr/bin/env python3
"""
System monitor module for ROS 2
Handles system status subscription and processing
"""

import json
import logging
import threading
import time
from typing import Dict, Any, Optional, Callable
from websocket import create_connection

logger = logging.getLogger(__name__)

class SystemMonitor:
    """Subscribe to and process system monitor data via ROSBridge"""
    
    def __init__(self, ros_bridge):
        """
        Initialize system monitor
        
        Args:
            ros_bridge: ROSBridgeConnection instance
        """
        self.ros_bridge = ros_bridge
        self.status_callback = None
        self.latest_status = None
        self.subscription_active = False
        self._ws = None
        self._ws_thread = None
        self._stop_thread = False
        
        # Default values
        self.default_status = {
            "cpu_usage": 0.0,
            "memory_usage": 0.0,
            "disk_usage": 0.0,
            "temperature": 0.0,
            "timestamp": time.time()
        }
        
    def subscribe(self, callback: Optional[Callable] = None) -> bool:
        """
        Subscribe to system status topics
        
        Args:
            callback: Optional callback function for new status data
        
        Returns:
            bool: Success status
        """
        try:
            # Create a dedicated connection for subscription
            self._ws = create_connection(self.ros_bridge.url, timeout=5)
            
            # Subscribe to all system topics
            topics = [
                ("/system/cpu_usage", "std_msgs/Float32"),
                ("/system/memory_usage", "std_msgs/Float32"),
                ("/system/temperature", "sensor_msgs/Temperature"),
                ("/system/status_json", "std_msgs/String")
            ]
            
            for topic, msg_type in topics:
                subscribe_msg = {
                    "op": "subscribe",
                    "topic": topic,
                    "type": msg_type
                }
                self._ws.send(json.dumps(subscribe_msg))
                logger.info(f"Subscribed to {topic}")
            
            # Start receiving thread
            self.subscription_active = True
            self._stop_thread = False
            self._ws_thread = threading.Thread(
                target=self._receive_status_data, 
                args=(callback,)
            )
            self._ws_thread.daemon = True
            self._ws_thread.start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to subscribe to system monitor: {e}")
            return False
    
    def _receive_status_data(self, callback):
        """Receive status data in separate thread"""
        current_status = self.default_status.copy()
        
        while self.subscription_active and not self._stop_thread:
            try:
                response = self._ws.recv()
                data = json.loads(response)
                
                if data.get("op") == "publish":
                    topic = data.get("topic")
                    msg = data.get("msg")
                    
                    # Update status based on topic
                    if topic == "/system/cpu_usage":
                        current_status["cpu_usage"] = msg.get("data", 0.0)
                    elif topic == "/system/memory_usage":
                        current_status["memory_usage"] = msg.get("data", 0.0)
                    elif topic == "/system/temperature":
                        # Temperature message has a different structure
                        current_status["temperature"] = msg.get("temperature", 0.0)
                    elif topic == "/system/status_json":
                        # Parse JSON status if available
                        try:
                            status_data = json.loads(msg.get("data", "{}"))
                            current_status.update(status_data)
                        except:
                            pass
                    
                    # Update timestamp
                    current_status["timestamp"] = time.time()
                    
                    # Process the status data
                    processed = self._process_status_data(current_status)
                    self.latest_status = processed
                    
                    if callback:
                        callback(processed)
                        
            except Exception as e:
                if self.subscription_active:
                    logger.error(f"Error receiving status data: {e}")
                break
        
        if self._ws:
            self._ws.close()
        logger.info("System monitor subscription thread ended")
    
    def _process_status_data(self, status: Dict) -> Dict[str, Any]:
        """
        Process raw status data for easier use
        
        Args:
            status: Raw status data
            
        Returns:
            Dict with processed data
        """
        return {
            "cpu_usage": round(status.get("cpu_usage", 0.0), 1),
            "memory_usage": round(status.get("memory_usage", 0.0), 1),
            "disk_usage": round(status.get("disk_usage", 0.0), 1),
            "temperature": round(status.get("temperature", 0.0), 1),
            "cpu_percent": f"{round(status.get('cpu_usage', 0.0), 1)}%",
            "memory_percent": f"{round(status.get('memory_usage', 0.0), 1)}%",
            "disk_percent": f"{round(status.get('disk_usage', 0.0), 1)}%",
            "temp_celsius": f"{round(status.get('temperature', 0.0), 1)}°C",
            "timestamp": status.get("timestamp", time.time()),
            "healthy": status.get("cpu_usage", 0) < 80 and status.get("temperature", 0) < 70
        }
    
    def get_latest_status(self) -> Optional[Dict[str, Any]]:
        """Get the most recent system status"""
        return self.latest_status
    
    def get_status_once(self) -> Optional[Dict[str, Any]]:
        """Request system status via service calls (if available)"""
        try:
            # Try to get status from service
            success, response = self.ros_bridge.call_service(
                "/get_system_status",
                {},
                "get_system_status"
            )
            
            if success and response.get("values"):
                status_data = response["values"]
                return self._process_status_data({
                    "cpu_usage": status_data.get("cpu_usage", 0.0),
                    "memory_usage": status_data.get("memory_usage", 0.0),
                    "disk_usage": status_data.get("disk_usage", 0.0),
                    "temperature": status_data.get("temperature", 0.0),
                    "timestamp": time.time()
                })
            else:
                logger.error("Failed to get system status via service")
                return None
                
        except Exception as e:
            logger.error(f"Error calling system status service: {e}")
            return None
    
    def unsubscribe(self):
        """Stop subscription"""
        self.subscription_active = False
        self._stop_thread = True
        if self._ws_thread:
            self._ws_thread.join(timeout=2)
        logger.info("Unsubscribed from system monitor")

# Singleton instance
_system_monitor = None

def get_system_monitor():
    """Get or create SystemMonitor instance"""
    global _system_monitor
    if _system_monitor is None:
        from backend.ros_bridge import get_ros_bridge
        _system_monitor = SystemMonitor(get_ros_bridge())
    return _system_monitor