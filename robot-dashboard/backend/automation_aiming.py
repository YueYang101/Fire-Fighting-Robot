#!/usr/bin/env python3
"""
Automated Aiming System
Uses thermal camera to detect hot spots and controls servos to aim at targets
"""

import logging
import threading
import time
import math
from typing import Dict, Any, Optional, Tuple, List
from datetime import datetime

logger = logging.getLogger(__name__)

class AutoAiming:
    """Automated aiming system for targeting heat sources"""
    
    def __init__(self, servo_controller, thermal_camera_sensor):
        """
        Initialize auto aiming system
        
        Args:
            servo_controller: ServoController instance
            thermal_camera_sensor: ThermalCameraSensor instance
        """
        self.servo_controller = servo_controller
        self.thermal_camera = thermal_camera_sensor
        
        # State management
        self.enabled = False
        self.tracking_active = False
        self._tracking_thread = None
        self._stop_thread = False
        
        # Target parameters
        self.target_threshold = 100.0  # Celsius
        self.current_target = None
        self.last_target_time = None
        
        # Servo limits (±60 degrees from center)
        self.pan_center = 135.0
        self.tilt_center = 135.0
        self.max_deviation = 60.0  # Maximum degrees from center
        
        # Servo range mapping
        # Servo range: 75-195 degrees (120 degree range)
        # We limit to ±60 degrees from center (135)
        self.pan_min = max(75.0, self.pan_center - self.max_deviation)    # 75
        self.pan_max = min(195.0, self.pan_center + self.max_deviation)   # 195
        self.tilt_min = max(75.0, self.tilt_center - self.max_deviation)  # 75
        self.tilt_max = min(195.0, self.tilt_center + self.max_deviation) # 195
        
        # Thermal camera FOV (approximate)
        self.thermal_fov_horizontal = 55.0  # degrees
        self.thermal_fov_vertical = 35.0    # degrees
        
        # Control parameters
        self.update_rate = 10  # Hz for checking targets
        self.movement_threshold = 5.0  # Minimum angle change to trigger movement (degrees)
        self.smoothing_factor = 0.3  # For exponential smoothing
        
        # Rate limiting for servo commands
        self.min_command_interval = 2.0  # Minimum seconds between servo commands
        self.last_command_time = 0  # Time of last servo command
        self.command_cooldown = False  # Whether we're in cooldown period
        
        # Dead zone to prevent jitter
        self.dead_zone = 3.0  # Degrees - don't move if within this range
        
        # Tracking state
        self.last_pan = self.pan_center
        self.last_tilt = self.tilt_center
        self.target_pan = self.pan_center
        self.target_tilt = self.tilt_center
        
        # Target stability
        self.target_stable_count = 0
        self.target_stability_threshold = 3  # Number of consistent detections before moving
        self.last_stable_target = None
        
        # Movement tracking for logging
        self.last_movement_angles = {
            'pan_change': 0.0,
            'tilt_change': 0.0,
            'total_movement': 0.0
        }
        
        logger.info("Auto aiming system initialized with rate limiting")
    
    def enable(self) -> Dict[str, Any]:
        """Enable auto aiming"""
        if self.enabled:
            return {
                "success": False,
                "error": "Auto aiming already enabled"
            }
        
        # Subscribe to thermal camera if not already
        if not self.thermal_camera.subscription_active:
            success = self.thermal_camera.subscribe()
            if not success:
                return {
                    "success": False,
                    "error": "Failed to subscribe to thermal camera"
                }
        
        self.enabled = True
        self.tracking_active = True
        self._stop_thread = False
        
        # Reset rate limiting
        self.last_command_time = 0
        self.command_cooldown = False
        
        # Start tracking thread
        self._tracking_thread = threading.Thread(target=self._tracking_loop)
        self._tracking_thread.daemon = True
        self._tracking_thread.start()
        
        logger.info("Auto aiming enabled")
        return {
            "success": True,
            "message": "Auto aiming enabled"
        }
    
    def disable(self) -> Dict[str, Any]:
        """Disable auto aiming"""
        if not self.enabled:
            return {
                "success": False,
                "error": "Auto aiming not enabled"
            }
        
        self.enabled = False
        self.tracking_active = False
        self._stop_thread = True
        
        # Wait for thread to stop
        if self._tracking_thread:
            self._tracking_thread.join(timeout=2.0)
        
        # Return servos to center
        self.servo_controller.center_position()
        
        logger.info("Auto aiming disabled")
        return {
            "success": True,
            "message": "Auto aiming disabled"
        }
    
    def _tracking_loop(self):
        """Main tracking loop with rate limiting"""
        logger.info("Auto aiming tracking loop started")
        
        while self.tracking_active and not self._stop_thread:
            try:
                # Get latest thermal frame
                frame_data = self.thermal_camera.get_latest_frame()
                
                if frame_data:
                    # Find targets
                    targets = self._find_targets(frame_data)
                    
                    if targets:
                        # Select leftmost target if multiple
                        target = self._select_target(targets)
                        
                        # Check target stability
                        if self._is_target_stable(target):
                            self.current_target = target
                            self.last_target_time = datetime.now()
                            
                            # Calculate servo angles
                            pan_angle, tilt_angle = self._calculate_servo_angles(
                                target, frame_data['width'], frame_data['height']
                            )
                            
                            # Apply smoothing
                            pan_angle = self._smooth_angle(pan_angle, self.target_pan)
                            tilt_angle = self._smooth_angle(tilt_angle, self.target_tilt)
                            
                            # Update target angles
                            self.target_pan = pan_angle
                            self.target_tilt = tilt_angle
                            
                            # Check if we should move servos
                            if self._should_move_servos(pan_angle, tilt_angle):
                                # Calculate movement angles for logging
                                self._calculate_movement_angles(pan_angle, tilt_angle)
                                
                                # Send servo command with rate limiting
                                self._send_servo_command(pan_angle, tilt_angle)
                    else:
                        self.current_target = None
                        self.target_stable_count = 0
                        self.last_stable_target = None
                
                # Sleep for update rate
                time.sleep(1.0 / self.update_rate)
                
            except Exception as e:
                logger.error(f"Error in tracking loop: {e}")
                time.sleep(0.1)
        
        logger.info("Auto aiming tracking loop ended")
    
    def _is_target_stable(self, target: Dict[str, Any]) -> bool:
        """Check if target is stable enough to track"""
        if self.last_stable_target is None:
            self.last_stable_target = target
            self.target_stable_count = 1
            return False
        
        # Check if target is similar to last one
        distance = math.sqrt(
            (target['x'] - self.last_stable_target['x'])**2 + 
            (target['y'] - self.last_stable_target['y'])**2
        )
        
        if distance < 2:  # Within 2 pixels
            self.target_stable_count += 1
            if self.target_stable_count >= self.target_stability_threshold:
                return True
        else:
            # Target moved, reset stability
            self.last_stable_target = target
            self.target_stable_count = 1
        
        return False
    
    def _should_move_servos(self, pan_angle: float, tilt_angle: float) -> bool:
        """Determine if servos should be moved based on dead zone and movement threshold"""
        # Calculate angle differences
        pan_diff = abs(pan_angle - self.last_pan)
        tilt_diff = abs(tilt_angle - self.last_tilt)
        
        # Check dead zone
        if pan_diff < self.dead_zone and tilt_diff < self.dead_zone:
            return False
        
        # Check movement threshold
        if pan_diff < self.movement_threshold and tilt_diff < self.movement_threshold:
            return False
        
        # Check rate limiting
        current_time = time.time()
        time_since_last = current_time - self.last_command_time
        
        if time_since_last < self.min_command_interval:
            # Still in cooldown period
            if not self.command_cooldown:
                self.command_cooldown = True
                logger.debug(f"Rate limiting: waiting {self.min_command_interval - time_since_last:.1f}s before next command")
            return False
        
        # Ready to send command
        self.command_cooldown = False
        return True
    
    def _calculate_movement_angles(self, new_pan: float, new_tilt: float):
        """Calculate the movement angles for logging"""
        pan_change = new_pan - self.last_pan
        tilt_change = new_tilt - self.last_tilt
        total_movement = math.sqrt(pan_change**2 + tilt_change**2)
        
        self.last_movement_angles = {
            'pan_change': pan_change,
            'tilt_change': tilt_change,
            'total_movement': total_movement
        }
    
    def _send_servo_command(self, pan_angle: float, tilt_angle: float):
        """Send servo command with rate limiting"""
        current_time = time.time()
        
        # Double-check rate limiting
        if current_time - self.last_command_time < self.min_command_interval:
            logger.warning("Rate limit check failed, skipping command")
            return
        
        # Send command
        result = self.servo_controller.move_to_position(pan_angle, tilt_angle)
        
        if result['success']:
            self.last_pan = pan_angle
            self.last_tilt = tilt_angle
            self.last_command_time = current_time
            
            # Log with movement angles
            logger.info(f"Servo command sent: Pan={pan_angle:.1f}° (Δ{self.last_movement_angles['pan_change']:+.1f}°), "
                       f"Tilt={tilt_angle:.1f}° (Δ{self.last_movement_angles['tilt_change']:+.1f}°) "
                       f"Total movement: {self.last_movement_angles['total_movement']:.1f}° "
                       f"(Target at {self.current_target['x']}, {self.current_target['y']} "
                       f"@ {self.current_target['temp']:.1f}°C)")
        else:
            logger.error(f"Failed to send servo command: {result.get('error', 'Unknown error')}")
    
    def _find_targets(self, frame_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Find all heat targets above threshold"""
        targets = []
        
        if not frame_data or 'thermal_data' not in frame_data:
            return targets
        
        thermal_data = frame_data['thermal_data']
        width = frame_data['width']
        height = frame_data['height']
        
        # Scan thermal data for hot spots
        for y in range(height):
            for x in range(width):
                temp = thermal_data[y][x]
                
                if temp >= self.target_threshold:
                    flipped_y = height - 1 - y
                    targets.append({
                        'x': x,
                        'y': flipped_y,
                        'temp': temp,
                        'grid_x': x,
                        'grid_y': flipped_y
                    })
        
        # Sort by temperature (highest first)
        targets.sort(key=lambda t: t['temp'], reverse=True)
        
        return targets
    
    def _select_target(self, targets: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Select target - leftmost if multiple"""
        if not targets:
            return None
        
        # If only one target, return it
        if len(targets) == 1:
            return targets[0]
        
        # Find leftmost target (minimum x coordinate)
        hottest = max(targets, key=lambda t: t['temp'])
        return hottest
    
    def _calculate_servo_angles(self, target: Dict[str, Any], 
                           frame_width: int, frame_height: int) -> Tuple[float, float]:
        """Calculate servo angles to aim at target"""
        # Get target position as fraction of frame (0-1)
        target_x_norm = target['x'] / (frame_width - 1)
        target_y_norm = target['y'] / (frame_height - 1)
        
        # Convert to angle offset from center (-0.5 to 0.5)
        # Note: X is inverted because left in image = right in real world
        x_offset = 0.5 - target_x_norm
        y_offset = target_y_norm - 0.5
        
        # Convert to degrees based on FOV
        # TRIPLE THE MOVEMENT COEFFICIENT HERE
        pan_movement_multiplier = 2.2
        tilt_movement_multiplier = 1.5
        pan_offset = x_offset * self.thermal_fov_horizontal * pan_movement_multiplier
        tilt_offset = y_offset * self.thermal_fov_vertical * tilt_movement_multiplier
        
        # Calculate target servo angles
        target_pan = self.pan_center + pan_offset
        target_tilt = self.tilt_center + tilt_offset
        
        # Apply limits (±60 degrees from center) - This ensures it stays in range
        target_pan = max(self.pan_min, min(self.pan_max, target_pan))
        target_tilt = max(self.tilt_min, min(self.tilt_max, target_tilt))
        
        return target_pan, target_tilt
    
    def _smooth_angle(self, new_angle: float, last_angle: float) -> float:
        """Apply exponential smoothing to angle"""
        return last_angle + self.smoothing_factor * (new_angle - last_angle)
    
    def get_status(self) -> Dict[str, Any]:
        """Get auto aiming status"""
        status = {
            "enabled": self.enabled,
            "tracking_active": self.tracking_active,
            "target_threshold": self.target_threshold,
            "current_target": None,
            "last_target_time": None,
            "servo_position": {
                "pan": self.last_pan,
                "tilt": self.last_tilt
            },
            "rate_limit_status": {
                "min_interval": self.min_command_interval,
                "in_cooldown": self.command_cooldown,
                "time_since_last": time.time() - self.last_command_time if self.last_command_time > 0 else None
            },
            "last_movement": self.last_movement_angles
        }
        
        if self.current_target:
            status["current_target"] = {
                "x": self.current_target['x'],
                "y": self.current_target['y'],
                "temperature": self.current_target['temp']
            }
        
        if self.last_target_time:
            status["last_target_time"] = self.last_target_time.isoformat()
        
        return status
    
    def set_threshold(self, threshold: float) -> Dict[str, Any]:
        """Set temperature threshold for targeting"""
        if threshold < 30.0 or threshold > 300.0:
            return {
                "success": False,
                "error": "Threshold must be between 30°C and 300°C"
            }
        
        self.target_threshold = threshold
        logger.info(f"Target threshold set to {threshold}°C")
        
        return {
            "success": True,
            "threshold": threshold
        }
    
    def set_rate_limit(self, interval: float) -> Dict[str, Any]:
        """Set minimum interval between servo commands"""
        if interval < 0.5 or interval > 10.0:
            return {
                "success": False,
                "error": "Interval must be between 0.5 and 10 seconds"
            }
        
        self.min_command_interval = interval
        logger.info(f"Rate limit set to {interval} seconds")
        
        return {
            "success": True,
            "interval": interval
        }

# Singleton instance
_auto_aiming = None

def get_auto_aiming(servo_controller=None, thermal_camera_sensor=None):
    """Get or create AutoAiming instance"""
    global _auto_aiming
    if _auto_aiming is None and servo_controller and thermal_camera_sensor:
        _auto_aiming = AutoAiming(servo_controller, thermal_camera_sensor)
    return _auto_aiming