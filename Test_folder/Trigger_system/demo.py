#!/usr/bin/env python3
"""
Linear Actuator Control using PCA9685
Controls a DC linear actuator through motor driver connected to PCA9685
Using channels 4 and 5 on the PCA9685
"""

import time
import sys

try:
    import board
    import busio
    import adafruit_pca9685
    HARDWARE = True
except ImportError:
    print("Warning: PCA9685 libraries not found. Running in simulation mode.")
    HARDWARE = False
    
    # Dummy classes for testing without hardware
    class _DummyChannel:
        def __init__(self):
            self.duty_cycle = 0
    
    class _DummyPCA:
        def __init__(self, i2c=None):  # Accept i2c parameter but ignore it
            self.channels = [_DummyChannel() for _ in range(16)]
            self.frequency = 100
    
    class _DummyI2C: pass
    
    class _DummyBusio:
        @staticmethod
        def I2C(*_):
            return _DummyI2C()
    
    class _DummyBoard:
        SCL = None
        SDA = None
    
    board = _DummyBoard()
    busio = _DummyBusio()
    adafruit_pca9685 = type("adafruit_pca9685", (), {"PCA9685": _DummyPCA})


class LinearActuatorPCA9685:
    def __init__(self, in1_channel=4, in2_channel=5, frequency=1000, max_extend_time=8):
        """
        Initialize linear actuator controller using PCA9685
        in1_channel: PCA9685 channel for IN1 (default: 4)
        in2_channel: PCA9685 channel for IN2 (default: 5)
        frequency: PWM frequency in Hz (default: 1000)
        max_extend_time: Maximum time allowed for extension in seconds (default: 8)
        """
        # Initialize I2C and PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.pca.frequency = frequency
        
        # Store channel numbers
        self.in1_channel = in1_channel
        self.in2_channel = in2_channel
        
        # Safety limit for extension only
        self.max_extend_time = max_extend_time
        
        # Track movement start time
        self.movement_start_time = None
        self.current_direction = None
        
        # Initialize channels to 0 (stopped)
        self.stop()
        
        print(f"PCA9685 initialized - IN1: Channel {in1_channel}, IN2: Channel {in2_channel}")
        print(f"PWM Frequency: {frequency}Hz")
        print(f"Safety limit - Max extend: {max_extend_time}s (no limit on retraction)")
    
    def _check_safety_timeout(self):
        """
        Check if extension movement has exceeded safety time limit
        Only applies to extension, not retraction
        """
        if self.movement_start_time and self.current_direction == "extend":
            elapsed = time.time() - self.movement_start_time
            
            if elapsed >= self.max_extend_time:
                print(f"\n⚠️  SAFETY STOP: Extension time limit ({self.max_extend_time}s) reached!")
                self.stop()
                return True
        return False
    
    def _set_duty_cycle(self, channel, value):
        """
        Set duty cycle for a channel (0-65535)
        """
        value = max(0, min(65535, value))  # Clamp to valid range
        self.pca.channels[channel].duty_cycle = value
    
    def extend(self, speed=100):
        """
        Extend the linear actuator
        speed: 0-100 (percentage of max speed)
        """
        # Check if already at safety limit
        if self._check_safety_timeout():
            return
        
        # Convert percentage to 16-bit duty cycle
        duty_cycle = int((speed / 100.0) * 65535)
        
        # IN1 = HIGH (PWM), IN2 = LOW
        self._set_duty_cycle(self.in1_channel, duty_cycle)
        self._set_duty_cycle(self.in2_channel, 0)
        
        # Track movement for safety
        self.movement_start_time = time.time()
        self.current_direction = "extend"
        
        print(f"Extending actuator at {speed}% speed (duty cycle: {duty_cycle})")
        if speed == 100:
            print(f"⚠️  Auto-stop will engage after {self.max_extend_time} seconds for safety")
    
    def retract(self, speed=100):
        """
        Retract the linear actuator (no time limit)
        speed: 0-100 (percentage of max speed)
        """
        # Convert percentage to 16-bit duty cycle
        duty_cycle = int((speed / 100.0) * 65535)
        
        # IN1 = LOW, IN2 = HIGH (PWM)
        self._set_duty_cycle(self.in1_channel, 0)
        self._set_duty_cycle(self.in2_channel, duty_cycle)
        
        # Track movement for consistency (but no safety limit)
        self.movement_start_time = time.time()
        self.current_direction = "retract"
        
        print(f"Retracting actuator at {speed}% speed (duty cycle: {duty_cycle})")
        print("No time limit on retraction - will run until stopped")
    
    def stop(self):
        """
        Stop the linear actuator
        """
        # Both channels to 0
        self._set_duty_cycle(self.in1_channel, 0)
        self._set_duty_cycle(self.in2_channel, 0)
        
        # Clear movement tracking
        self.movement_start_time = None
        self.current_direction = None
        
        print("Actuator stopped")
    
    def extend_for_duration(self, duration, speed=100):
        """
        Extend actuator for a specific duration (in seconds)
        """
        self.extend(speed)
        time.sleep(duration)
        self.stop()
    
    def retract_for_duration(self, duration, speed=100):
        """
        Retract actuator for a specific duration (in seconds)
        """
        self.retract(speed)
        time.sleep(duration)
        self.stop()
    
    def cleanup(self):
        """
        Clean up - ensure actuator is stopped
        """
        self.stop()
        # Note: PCA9685 doesn't require GPIO cleanup like RPi.GPIO


def main():
    # Create actuator controller using PCA9685 channels 4 and 5
    actuator = LinearActuatorPCA9685(in1_channel=4, in2_channel=5)
    
    print("\n" + "="*50)
    print("Linear Actuator Control (PCA9685)")
    print("="*50)
    print("\nCommands:")
    print("  e     - Extend at full speed")
    print("  r     - Retract at full speed")
    print("  s     - Stop")
    print("  1-9   - Extend/Retract for 1-9 seconds")
    print("  v     - Variable speed control (0-100%)")
    print("  t     - Test sequence")
    print("  quit  - Exit program")
    print("-"*50)
    
    # Start safety monitor thread
    import threading
    
    def safety_monitor():
        while True:
            actuator._check_safety_timeout()
            time.sleep(0.1)  # Check every 100ms
    
    safety_thread = threading.Thread(target=safety_monitor, daemon=True)
    safety_thread.start()
    
    try:
        while True:
            command = input("\nEnter command: ").lower().strip()
            
            if command == 'quit':
                break
                
            elif command == 'e':
                actuator.extend()
                
            elif command == 'r':
                actuator.retract()
                
            elif command == 's':
                actuator.stop()
                
            elif command in '123456789':
                duration = int(command)
                direction = input(f"Extend (e) or Retract (r) for {duration} seconds? ").lower()
                
                if direction == 'e':
                    if duration > actuator.max_extend_time:
                        print(f"⚠️  Warning: Extension duration {duration}s exceeds safety limit of {actuator.max_extend_time}s")
                        print(f"Duration will be capped at {actuator.max_extend_time}s")
                        duration = actuator.max_extend_time
                    actuator.extend_for_duration(duration)
                elif direction == 'r':
                    # No limit on retraction
                    actuator.retract_for_duration(duration)
                else:
                    print("Invalid direction. Use 'e' or 'r'")
                    
            elif command == 'v':
                try:
                    speed = int(input("Enter speed (0-100): "))
                    if 0 <= speed <= 100:
                        direction = input("Extend (e) or Retract (r)? ").lower()
                        if direction == 'e':
                            actuator.extend(speed)
                        elif direction == 'r':
                            actuator.retract(speed)
                        else:
                            print("Invalid direction. Use 'e' or 'r'")
                    else:
                        print("Speed must be between 0 and 100")
                except ValueError:
                    print("Invalid speed value")
                    
            elif command == 't':
                print("\nRunning test sequence...")
                print("1. Extending at 50% speed for 2 seconds")
                actuator.extend_for_duration(2, 50)
                time.sleep(1)
                
                print("2. Retracting at 100% speed for 2 seconds")
                actuator.retract_for_duration(2, 100)
                time.sleep(1)
                
                print("3. Extending at 25% speed for 1 second")
                actuator.extend_for_duration(1, 25)
                
                print("Test sequence complete!")
                
            else:
                print("Unknown command. Type 'quit' to exit.")
                
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
        
    finally:
        actuator.cleanup()
        print("Actuator stopped. Exiting...")


if __name__ == "__main__":
    if not HARDWARE:
        print("\n⚠️  WARNING: Running in simulation mode - no real hardware detected!")
        print("Install adafruit-circuitpython-pca9685 to use with real hardware.\n")
    
    main()