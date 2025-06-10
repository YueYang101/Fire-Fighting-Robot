#!/usr/bin/env python3
"""
Improved ROS2 node for smoother servo control
Save this as servo_controller_node_smooth.py
"""

import rclpy
from rclpy.node import Node
from servo_interfaces.msg import ServoPosition, ServoState
from servo_interfaces.srv import SetServoPosition, SetServoSpeed
import RPi.GPIO as GPIO
import time
import threading
import math


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # Declare parameters
        self.declare_parameter('pan_pin', 13)
        self.declare_parameter('tilt_pin', 12)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('min_angle', 0)
        self.declare_parameter('max_angle', 270)
        self.declare_parameter('default_speed', 50)  # degrees per second
        self.declare_parameter('step_resolution', 0.5)  # degrees per step
        self.declare_parameter('keep_pwm_active', True)  # Keep PWM signal active
        
        # Get parameters
        self.pan_pin = self.get_parameter('pan_pin').value
        self.tilt_pin = self.get_parameter('tilt_pin').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.default_speed = self.get_parameter('default_speed').value
        self.step_resolution = self.get_parameter('step_resolution').value
        self.keep_pwm_active = self.get_parameter('keep_pwm_active').value
        
        # Current servo states
        self.current_pan = 135.0  # Center position
        self.current_tilt = 135.0  # Center position
        self.target_pan = 135.0
        self.target_tilt = 135.0
        self.pan_speed = self.default_speed
        self.tilt_speed = self.default_speed
        
        # Movement flags
        self.pan_moving = False
        self.tilt_moving = False
        
        # Threading locks for thread safety
        self.pan_lock = threading.Lock()
        self.tilt_lock = threading.Lock()
        
        # Initialize GPIO
        self.init_gpio()
        
        # Create publishers
        self.state_publisher = self.create_publisher(ServoState, 'servo_state', 10)
        
        # Create subscribers
        self.position_subscriber = self.create_subscription(
            ServoPosition,
            'servo_position_cmd',
            self.position_callback,
            10
        )
        
        # Create services
        self.set_position_service = self.create_service(
            SetServoPosition,
            'set_servo_position',
            self.set_position_callback
        )
        
        self.set_speed_service = self.create_service(
            SetServoSpeed,
            'set_servo_speed',
            self.set_speed_callback
        )
        
        # Create timer for publishing state (reduced frequency)
        self.state_timer = self.create_timer(0.2, self.publish_state)
        
        # Start continuous movement threads
        self.pan_thread = threading.Thread(target=self.pan_control_loop, daemon=True)
        self.tilt_thread = threading.Thread(target=self.tilt_control_loop, daemon=True)
        self.pan_thread.start()
        self.tilt_thread.start()
        
        # Move to initial position
        self.set_servo_angle(self.pan_pwm, self.current_pan)
        self.set_servo_angle(self.tilt_pwm, self.current_tilt)
        
        self.get_logger().info(
            f'Servo controller initialized - Pan: GPIO{self.pan_pin}, Tilt: GPIO{self.tilt_pin}, '
            f'Step resolution: {self.step_resolution}°'
        )
    
    def init_gpio(self):
        """Initialize GPIO and PWM"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        
        self.pan_pwm = GPIO.PWM(self.pan_pin, self.pwm_frequency)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, self.pwm_frequency)
        
        self.pan_pwm.start(0)
        self.tilt_pwm.start(0)
        
        time.sleep(0.1)
    
    def angle_to_duty_cycle(self, angle):
        """Convert angle to duty cycle percentage"""
        # Map angle (0-270) to duty cycle (2.5-12.5)
        duty_cycle = 2.5 + (angle / 270.0) * 10.0
        return duty_cycle
    
    def set_servo_angle(self, servo_pwm, angle):
        """Set servo to specific angle"""
        if self.min_angle <= angle <= self.max_angle:
            duty_cycle = self.angle_to_duty_cycle(angle)
            servo_pwm.ChangeDutyCycle(duty_cycle)
            
            # Only stop PWM if configured to do so
            if not self.keep_pwm_active:
                time.sleep(0.02)
                servo_pwm.ChangeDutyCycle(0)
            return True
        return False
    
    def pan_control_loop(self):
        """Continuous control loop for pan servo"""
        while rclpy.ok():
            with self.pan_lock:
                if abs(self.current_pan - self.target_pan) > self.step_resolution / 2:
                    self.pan_moving = True
                    
                    # Calculate step size based on speed
                    max_step = self.pan_speed * 0.01  # Convert to degrees per 10ms
                    diff = self.target_pan - self.current_pan
                    step = max(-max_step, min(max_step, diff))
                    
                    # Apply easing for smoother motion
                    if abs(diff) < 10:  # Slow down near target
                        step *= 0.5
                    
                    self.current_pan += step
                    self.set_servo_angle(self.pan_pwm, self.current_pan)
                else:
                    self.pan_moving = False
                    self.current_pan = self.target_pan
                    self.set_servo_angle(self.pan_pwm, self.current_pan)
            
            time.sleep(0.01)  # 10ms update rate for smooth motion
    
    def tilt_control_loop(self):
        """Continuous control loop for tilt servo"""
        while rclpy.ok():
            with self.tilt_lock:
                if abs(self.current_tilt - self.target_tilt) > self.step_resolution / 2:
                    self.tilt_moving = True
                    
                    # Calculate step size based on speed
                    max_step = self.tilt_speed * 0.01  # Convert to degrees per 10ms
                    diff = self.target_tilt - self.current_tilt
                    step = max(-max_step, min(max_step, diff))
                    
                    # Apply easing for smoother motion
                    if abs(diff) < 10:  # Slow down near target
                        step *= 0.5
                    
                    self.current_tilt += step
                    self.set_servo_angle(self.tilt_pwm, self.current_tilt)
                else:
                    self.tilt_moving = False
                    self.current_tilt = self.target_tilt
                    self.set_servo_angle(self.tilt_pwm, self.current_tilt)
            
            time.sleep(0.01)  # 10ms update rate for smooth motion
    
    def position_callback(self, msg):
        """Handle position command messages"""
        with self.pan_lock:
            self.target_pan = max(self.min_angle, min(self.max_angle, msg.pan_angle))
        
        with self.tilt_lock:
            self.target_tilt = max(self.min_angle, min(self.max_angle, msg.tilt_angle))
    
    def set_position_callback(self, request, response):
        """Handle set position service requests"""
        # Validate angles
        if not (self.min_angle <= request.pan_angle <= self.max_angle):
            response.success = False
            response.message = f"Pan angle must be between {self.min_angle} and {self.max_angle}"
            return response
        
        if not (self.min_angle <= request.tilt_angle <= self.max_angle):
            response.success = False
            response.message = f"Tilt angle must be between {self.min_angle} and {self.max_angle}"
            return response
        
        # Set targets
        with self.pan_lock:
            self.target_pan = request.pan_angle
        with self.tilt_lock:
            self.target_tilt = request.tilt_angle
        
        response.success = True
        response.message = f"Moving to Pan: {request.pan_angle}°, Tilt: {request.tilt_angle}°"
        return response
    
    def set_speed_callback(self, request, response):
        """Handle set speed service requests"""
        if request.pan_speed > 0:
            self.pan_speed = request.pan_speed
        if request.tilt_speed > 0:
            self.tilt_speed = request.tilt_speed
        
        response.success = True
        response.message = f"Speed set - Pan: {self.pan_speed}°/s, Tilt: {self.tilt_speed}°/s"
        return response
    
    def publish_state(self):
        """Publish current servo state"""
        state_msg = ServoState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        
        with self.pan_lock:
            state_msg.pan_angle = self.current_pan
            state_msg.pan_moving = self.pan_moving
        
        with self.tilt_lock:
            state_msg.tilt_angle = self.current_tilt
            state_msg.tilt_moving = self.tilt_moving
        
        self.state_publisher.publish(state_msg)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        # Stop PWM
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        servo_controller = ServoControllerNode()
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.cleanup()
        servo_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()