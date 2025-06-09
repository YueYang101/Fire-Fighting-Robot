#!/usr/bin/env python3
"""
Linear Actuator Control using PCA9685 with Keyboard Control
W key - Extend (hold to continue, release to stop)
S key - Retract (hold to continue, release to stop)
Q key - Quit program
Total extension time is limited to 8 seconds cumulative
"""

import time
import sys
import threading
import select
import termios
import tty

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
        def __init__(self, i2c=None):
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


class KeyboardController:
    """Non-blocking keyboard input handler"""
    def __init__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
    def get_key(self):
        """Get a single keypress without blocking"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1).lower()
        return None
    
    def restore(self):
        """Restore terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


class LinearActuatorPCA9685:
    def __init__(self, in1_channel=4, in2_channel=5, frequency=1000, max_extend_time=8):
        """
        Initialize linear actuator controller using PCA9685
        in1_channel: PCA9685 channel for IN1 (default: 4)
        in2_channel: PCA9685 channel for IN2 (default: 5)
        frequency: PWM frequency in Hz (default: 1000)
        max_extend_time: Maximum CUMULATIVE time allowed for extension in seconds (default: 8)
        """
        # Initialize I2C and PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.pca.frequency = frequency
        
        # Store channel numbers
        self.in1_channel = in1_channel
        self.in2_channel = in2_channel
        
        # Safety limit for cumulative extension time
        self.max_extend_time = max_extend_time
        self.total_extend_time = 0.0  # Track cumulative extend time
        
        # Track movement
        self.movement_start_time = None
        self.current_direction = None
        self.is_moving = False
        
        # Initialize channels to 0 (stopped)
        self.stop()
        
        print(f"\nPCA9685 initialized - IN1: Channel {in1_channel}, IN2: Channel {in2_channel}")
        print(f"PWM Frequency: {frequency}Hz")
        print(f"Safety limit - Max cumulative extend time: {max_extend_time}s")
    
    def _set_duty_cycle(self, channel, value):
        """Set duty cycle for a channel (0-65535)"""
        value = max(0, min(65535, value))
        self.pca.channels[channel].duty_cycle = value
    
    def get_remaining_extend_time(self):
        """Get remaining extension time available"""
        return max(0, self.max_extend_time - self.total_extend_time)
    
    def can_extend(self):
        """Check if extension is allowed"""
        return self.get_remaining_extend_time() > 0
    
    def extend(self, speed=100):
        """Extend the linear actuator"""
        if not self.can_extend():
            print(f"\n⚠️  Cannot extend: Cumulative limit of {self.max_extend_time}s reached!")
            return False
        
        # Convert percentage to 16-bit duty cycle
        duty_cycle = int((speed / 100.0) * 65535)
        
        # IN1 = HIGH (PWM), IN2 = LOW
        self._set_duty_cycle(self.in1_channel, duty_cycle)
        self._set_duty_cycle(self.in2_channel, 0)
        
        # Track movement
        self.movement_start_time = time.time()
        self.current_direction = "extend"
        self.is_moving = True
        
        remaining = self.get_remaining_extend_time()
        print(f"\nExtending... (Remaining time: {remaining:.1f}s)")
        return True
    
    def retract(self, speed=100):
        """Retract the linear actuator (no time limit)"""
        # Convert percentage to 16-bit duty cycle
        duty_cycle = int((speed / 100.0) * 65535)
        
        # IN1 = LOW, IN2 = HIGH (PWM)
        self._set_duty_cycle(self.in1_channel, 0)
        self._set_duty_cycle(self.in2_channel, duty_cycle)
        
        # Track movement
        self.movement_start_time = time.time()
        self.current_direction = "retract"
        self.is_moving = True
        
        print(f"\nRetracting... (No time limit)")
        return True
    
    def stop(self):
        """Stop the linear actuator"""
        # Update cumulative extend time if we were extending
        if self.is_moving and self.current_direction == "extend" and self.movement_start_time:
            elapsed = time.time() - self.movement_start_time
            self.total_extend_time += elapsed
            print(f"\nStopped. Extended for {elapsed:.1f}s (Total: {self.total_extend_time:.1f}s)")
        elif self.is_moving and self.current_direction == "retract":
            elapsed = time.time() - self.movement_start_time
            print(f"\nStopped. Retracted for {elapsed:.1f}s")
        
        # Both channels to 0
        self._set_duty_cycle(self.in1_channel, 0)
        self._set_duty_cycle(self.in2_channel, 0)
        
        # Clear movement tracking
        self.movement_start_time = None
        self.current_direction = None
        self.is_moving = False
    
    def check_safety(self):
        """Check if extension time limit reached during movement"""
        if self.is_moving and self.current_direction == "extend" and self.movement_start_time:
            elapsed = time.time() - self.movement_start_time
            if self.total_extend_time + elapsed >= self.max_extend_time:
                print(f"\n⚠️  SAFETY STOP: Cumulative extension limit ({self.max_extend_time}s) reached!")
                self.stop()
                return True
        return False
    
    def cleanup(self):
        """Clean up - ensure actuator is stopped"""
        self.stop()


def main():
    # Create actuator controller
    actuator = LinearActuatorPCA9685(in1_channel=4, in2_channel=5)
    
    print("\n" + "="*50)
    print("Linear Actuator Keyboard Control (PCA9685)")
    print("="*50)
    print("\nControls:")
    print("  W     - Hold to EXTEND (release to stop)")
    print("  S     - Hold to RETRACT (release to stop)")
    print("  Q     - Quit program")
    print("  Space - Emergency STOP")
    print("\n⚠️  Total extension time is limited to 8 seconds")
    print("-"*50)
    
    # Initialize keyboard controller
    kb = KeyboardController()
    
    # Track key states
    key_states = {'w': False, 's': False}
    running = True
    
    # Safety monitor thread
    def safety_monitor():
        while running:
            actuator.check_safety()
            time.sleep(0.1)
    
    safety_thread = threading.Thread(target=safety_monitor, daemon=True)
    safety_thread.start()
    
    # Status display thread
    def status_display():
        while running:
            if actuator.is_moving:
                remaining = actuator.get_remaining_extend_time()
                if actuator.current_direction == "extend":
                    elapsed = time.time() - actuator.movement_start_time
                    print(f"\r[EXTENDING] Time: {elapsed:.1f}s | Remaining: {remaining:.1f}s | Total used: {actuator.total_extend_time + elapsed:.1f}s", end='', flush=True)
                elif actuator.current_direction == "retract":
                    elapsed = time.time() - actuator.movement_start_time
                    print(f"\r[RETRACTING] Time: {elapsed:.1f}s | Extension budget remaining: {remaining:.1f}s", end='', flush=True)
            time.sleep(0.1)
    
    status_thread = threading.Thread(target=status_display, daemon=True)
    status_thread.start()
    
    try:
        print("\nReady! Use W/S keys to control actuator, Q to quit")
        
        while running:
            key = kb.get_key()
            
            if key == 'q':
                print("\nQuitting...")
                running = False
                break
                
            elif key == ' ':  # Space for emergency stop
                actuator.stop()
                key_states = {'w': False, 's': False}
                print("\n⚠️  EMERGENCY STOP!")
                
            elif key == 'w':
                if not key_states['w'] and not key_states['s']:  # Not already extending and not retracting
                    if actuator.extend():
                        key_states['w'] = True
                        
            elif key == 's':
                if not key_states['s'] and not key_states['w']:  # Not already retracting and not extending
                    if actuator.retract():
                        key_states['s'] = True
            
            # Check for key releases
            if key_states['w'] and key != 'w':
                # W was pressed but now released
                actuator.stop()
                key_states['w'] = False
                
            if key_states['s'] and key != 's':
                # S was pressed but now released
                actuator.stop()
                key_states['s'] = False
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
        
    finally:
        running = False
        actuator.cleanup()
        kb.restore()
        print(f"\nFinal stats: Total extension time used: {actuator.total_extend_time:.1f}s / {actuator.max_extend_time}s")
        print("Actuator stopped. Exiting...")


if __name__ == "__main__":
    if not HARDWARE:
        print("\n⚠️  WARNING: Running in simulation mode - no real hardware detected!")
        print("Install adafruit-circuitpython-pca9685 to use with real hardware.\n")
    
    main()