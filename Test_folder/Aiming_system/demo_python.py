#!/usr/bin/env python3
"""
Servo Control for Raspberry Pi
Controls two servo motors (pan and tilt) with 0-270 degree range
"""

import RPi.GPIO as GPIO
import time

class ServoController:
    def __init__(self, servo1_pin=12, servo2_pin=13):
        """
        Initialize servo controller
        servo1_pin: GPIO pin for tilt servo (default: GPIO 12, physical pin 32)
        servo2_pin: GPIO pin for pan servo (default: GPIO 13, physical pin 33)
        """
        # Use BCM pin numbering
        GPIO.setmode(GPIO.BCM)
        
        # Store pin numbers
        self.servo1_pin = servo1_pin  # Tilt
        self.servo2_pin = servo2_pin  # Pan
        
        # Setup GPIO pins as outputs
        GPIO.setup(self.servo1_pin, GPIO.OUT)
        GPIO.setup(self.servo2_pin, GPIO.OUT)
        
        # Create PWM instances (50Hz is standard for servos)
        self.servo1_pwm = GPIO.PWM(self.servo1_pin, 50)
        self.servo2_pwm = GPIO.PWM(self.servo2_pin, 50)
        
        # Start PWM with 0% duty cycle
        self.servo1_pwm.start(0)
        self.servo2_pwm.start(0)
        
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
        """
        if 0 <= angle <= 270:
            duty_cycle = self.angle_to_duty_cycle(angle)
            servo_pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.5)  # Give servo time to move
            # Stop sending signal to reduce jitter
            servo_pwm.ChangeDutyCycle(0)
            return True
        return False
    
    def cleanup(self):
        """
        Clean up GPIO resources
        """
        self.servo1_pwm.stop()
        self.servo2_pwm.stop()
        GPIO.cleanup()

def main():
    # Create servo controller instance
    controller = ServoController(servo1_pin=12, servo2_pin=13)
    
    print("Servo Control Program")
    print("Enter angles for Servo 1 (Tilt) and Servo 2 (Pan) (0-270°)")
    print("Type 'quit' to exit")
    
    try:
        while True:
            # Get pan angle
            pan_input = input("\nEnter angle for pan (0-270): ")
            
            if pan_input.lower() == 'quit':
                break
            
            try:
                pan_angle = int(pan_input)
                if controller.move_servo(controller.servo2_pwm, pan_angle):
                    print(f"Servo 2 (Pan) moved to {pan_angle} degrees.")
                else:
                    print("Invalid input. Enter a value between 0 and 270.")
            except ValueError:
                print("Invalid input. Please enter a number.")
            
            # Get tilt angle
            tilt_input = input("Enter angle for tilt (0-270): ")
            
            if tilt_input.lower() == 'quit':
                break
            
            try:
                tilt_angle = int(tilt_input)
                if controller.move_servo(controller.servo1_pwm, tilt_angle):
                    print(f"Servo 1 (Tilt) moved to {tilt_angle} degrees.")
                else:
                    print("Invalid input. Enter a value between 0 and 270.")
            except ValueError:
                print("Invalid input. Please enter a number.")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    finally:
        # Always clean up GPIO on exit
        controller.cleanup()
        print("GPIO cleaned up. Exiting...")

if __name__ == "__main__":
    main()