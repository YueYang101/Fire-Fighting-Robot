#!/usr/bin/env python3
"""
Simple ROS2 node for controlling pan-tilt servos on Raspberry Pi
Works like the original demo - no continuous updates, no vibration
"""

import rclpy
from rclpy.node import Node
from servo_interfaces.msg import ServoPosition, ServoState
from servo_interfaces.srv import SetServoPosition, SetServoSpeed
import RPi.GPIO as GPIO
import time


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # Declare parameters
        self.declare_parameter('pan_pin', 13)
        self.declare_parameter('tilt_pin', 12)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('min_angle', 0)
        self.declare_parameter('max_angle', 270)
        
        # Get parameters
        self.pan_pin = self.get_parameter('pan_pin').value
        self.tilt_pin = self.get_parameter('tilt_pin').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        
        # Current servo states
        self.current_pan = 135.0  # Center position
        self.current_tilt = 135.0  # Center position
        
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
        
        # Create timer for publishing state
        self.state_timer = self.create_timer(0.5, self.publish_state)  # 2Hz is enough
        
        # Move to initial position
        self.move_servo(self.pan_pwm, self.current_pan)
        self.move_servo(self.tilt_pwm, self.current_tilt)
        
        self.get_logger().info(f'Servo controller initialized - Pan: GPIO{self.pan_pin}, Tilt: GPIO{self.tilt_pin}')
    
    def init_gpio(self):
        """Initialize GPIO and PWM"""
        # Use BCM pin numbering
        GPIO.setmode(GPIO.BCM)
        
        # Setup GPIO pins as outputs
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        
        # Create PWM instances (50Hz is standard for servos)
        self.pan_pwm = GPIO.PWM(self.pan_pin, self.pwm_frequency)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, self.pwm_frequency)
        
        # Start PWM with 0% duty cycle
        self.pan_pwm.start(0)
        self.tilt_pwm.start(0)
        
        # Small delay to let servos initialize
        time.sleep(0.5)
    
    def angle_to_duty_cycle(self, angle):
        """
        Convert angle (0-270 degrees) to duty cycle percentage
        
        For 50Hz PWM:
        - 0.5ms pulse (2.5% duty cycle) = 0 degrees
        - 2.5ms pulse (12.5% duty cycle) = 270 degrees
        """
        # Map angle (0-270) to duty cycle (2.5-12.5)
        duty_cycle = 2.5 + (angle / 270.0) * 10.0
        return duty_cycle
    
    def move_servo(self, servo_pwm, angle):
        """
        Move a servo to specified angle
        Same as original demo code
        """
        if self.min_angle <= angle <= self.max_angle:
            duty_cycle = self.angle_to_duty_cycle(angle)
            servo_pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.5)  # Give servo time to move
            # Stop sending signal to reduce jitter/vibration
            servo_pwm.ChangeDutyCycle(0)
            return True
        return False
    
    def position_callback(self, msg):
        """Handle position command messages"""
        # Move pan servo if angle changed
        if msg.pan_angle != self.current_pan:
            if self.move_servo(self.pan_pwm, msg.pan_angle):
                self.current_pan = msg.pan_angle
                self.get_logger().info(f'Pan servo moved to {msg.pan_angle}°')
            else:
                self.get_logger().warn(f'Invalid pan angle: {msg.pan_angle}')
        
        # Move tilt servo if angle changed
        if msg.tilt_angle != self.current_tilt:
            if self.move_servo(self.tilt_pwm, msg.tilt_angle):
                self.current_tilt = msg.tilt_angle
                self.get_logger().info(f'Tilt servo moved to {msg.tilt_angle}°')
            else:
                self.get_logger().warn(f'Invalid tilt angle: {msg.tilt_angle}')
    
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
        
        # Move servos
        self.move_servo(self.pan_pwm, request.pan_angle)
        self.current_pan = request.pan_angle
        
        self.move_servo(self.tilt_pwm, request.tilt_angle)
        self.current_tilt = request.tilt_angle
        
        response.success = True
        response.message = f"Moved to Pan: {request.pan_angle}°, Tilt: {request.tilt_angle}°"
        self.get_logger().info(response.message)
        return response
    
    def publish_state(self):
        """Publish current servo state"""
        state_msg = ServoState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.pan_angle = self.current_pan
        state_msg.tilt_angle = self.current_tilt
        state_msg.pan_moving = False  # Simple version - no movement tracking
        state_msg.tilt_moving = False
        self.state_publisher.publish(state_msg)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup()
        self.get_logger().info('GPIO cleaned up')


def main(args=None):
    rclpy.init(args=args)
    
    servo_controller = ServoControllerNode()
    
    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        servo_controller.get_logger().info('Shutting down servo controller...')
    finally:
        servo_controller.cleanup()
        servo_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()