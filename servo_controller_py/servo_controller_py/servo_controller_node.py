#!/usr/bin/env python3
"""
ROS2 node for controlling pan-tilt servos on Raspberry Pi
"""

import rclpy
from rclpy.node import Node
from servo_interfaces.msg import ServoPosition, ServoState
from servo_interfaces.srv import SetServoPosition, SetServoSpeed
import RPi.GPIO as GPIO
import time
import threading


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
        
        # Get parameters
        self.pan_pin = self.get_parameter('pan_pin').value
        self.tilt_pin = self.get_parameter('tilt_pin').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.default_speed = self.get_parameter('default_speed').value
        
        # Current servo states
        self.current_pan = 135.0  # Center position
        self.current_tilt = 135.0  # Center position
        self.pan_speed = self.default_speed
        self.tilt_speed = self.default_speed
        
        # Movement flags
        self.pan_moving = False
        self.tilt_moving = False
        
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
        
        # Create timer for publishing state
        self.state_timer = self.create_timer(0.1, self.publish_state)
        
        # Move to initial position
        self.move_servo_immediate(self.pan_pwm, self.current_pan)
        self.move_servo_immediate(self.tilt_pwm, self.current_tilt)
        
        self.get_logger().info(f'Servo controller initialized - Pan: GPIO{self.pan_pin}, Tilt: GPIO{self.tilt_pin}')
    
    def init_gpio(self):
        """Initialize GPIO and PWM"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        
        self.pan_pwm = GPIO.PWM(self.pan_pin, self.pwm_frequency)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, self.pwm_frequency)
        
        self.pan_pwm.start(0)
        self.tilt_pwm.start(0)
        
        time.sleep(0.5)
    
    def angle_to_duty_cycle(self, angle):
        """Convert angle to duty cycle percentage"""
        # Map angle (0-270) to duty cycle (2.5-12.5)
        duty_cycle = 2.5 + (angle / 270.0) * 10.0
        return duty_cycle
    
    def move_servo_immediate(self, servo_pwm, angle):
        """Move servo immediately to specified angle"""
        if self.min_angle <= angle <= self.max_angle:
            duty_cycle = self.angle_to_duty_cycle(angle)
            servo_pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.02)  # Small delay for signal
            servo_pwm.ChangeDutyCycle(0)  # Stop signal to reduce jitter
            return True
        return False
    
    def move_servo_smooth(self, servo_pwm, start_angle, target_angle, speed, is_pan=True):
        """Move servo smoothly from start to target angle at specified speed"""
        if is_pan:
            self.pan_moving = True
        else:
            self.tilt_moving = True
        
        current = start_angle
        step = 1.0 if target_angle > start_angle else -1.0
        delay = 1.0 / speed  # Time between steps
        
        while abs(current - target_angle) > 0.5:
            current += step
            if step > 0:
                current = min(current, target_angle)
            else:
                current = max(current, target_angle)
            
            self.move_servo_immediate(servo_pwm, current)
            
            if is_pan:
                self.current_pan = current
            else:
                self.current_tilt = current
            
            time.sleep(delay)
        
        # Final position
        self.move_servo_immediate(servo_pwm, target_angle)
        
        if is_pan:
            self.current_pan = target_angle
            self.pan_moving = False
        else:
            self.current_tilt = target_angle
            self.tilt_moving = False
    
    def position_callback(self, msg):
        """Handle position command messages"""
        # Start pan movement in thread
        if msg.pan_angle != self.current_pan and not self.pan_moving:
            pan_thread = threading.Thread(
                target=self.move_servo_smooth,
                args=(self.pan_pwm, self.current_pan, msg.pan_angle, self.pan_speed, True)
            )
            pan_thread.daemon = True
            pan_thread.start()
        
        # Start tilt movement in thread
        if msg.tilt_angle != self.current_tilt and not self.tilt_moving:
            tilt_thread = threading.Thread(
                target=self.move_servo_smooth,
                args=(self.tilt_pwm, self.current_tilt, msg.tilt_angle, self.tilt_speed, False)
            )
            tilt_thread.daemon = True
            tilt_thread.start()
    
    def set_position_callback(self, request, response):
        """Handle set position service requests"""
        success = True
        
        # Validate angles
        if not (self.min_angle <= request.pan_angle <= self.max_angle):
            response.success = False
            response.message = f"Pan angle must be between {self.min_angle} and {self.max_angle}"
            return response
        
        if not (self.min_angle <= request.tilt_angle <= self.max_angle):
            response.success = False
            response.message = f"Tilt angle must be between {self.min_angle} and {self.max_angle}"
            return response
        
        # Create and publish position message
        pos_msg = ServoPosition()
        pos_msg.pan_angle = request.pan_angle
        pos_msg.tilt_angle = request.tilt_angle
        self.position_callback(pos_msg)
        
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
        state_msg.pan_angle = self.current_pan
        state_msg.tilt_angle = self.current_tilt
        state_msg.pan_moving = self.pan_moving
        state_msg.tilt_moving = self.tilt_moving
        self.state_publisher.publish(state_msg)
    
    def cleanup(self):
        """Clean up GPIO resources"""
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
