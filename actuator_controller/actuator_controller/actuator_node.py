#!/usr/bin/env python3
"""
ROS2 Node for Linear Actuator Control using PCA9685
Controls a DC linear actuator through channels 4-5 on PCA9685
"""

import rclpy
from rclpy.node import Node
from actuator_interfaces.srv import SetActuator
import time
import threading

try:
    import board
    import busio
    import adafruit_pca9685
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("Warning: PCA9685 hardware not available - running in simulation mode")
    
    # Dummy classes for testing
    class DummyChannel:
        def __init__(self):
            self.duty_cycle = 0
    
    class DummyPCA:
        def __init__(self):
            self.channels = [DummyChannel() for _ in range(16)]
            self.frequency = 1000


class LinearActuatorNode(Node):
    def __init__(self):
        super().__init__('linear_actuator_node')
        
        # Declare parameters
        self.declare_parameter('in1_channel', 4)
        self.declare_parameter('in2_channel', 5)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('max_extend_time', 8.0)
        
        # Get parameters
        self.in1_channel = self.get_parameter('in1_channel').value
        self.in2_channel = self.get_parameter('in2_channel').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.max_extend_time = self.get_parameter('max_extend_time').value
        
        # Initialize PCA9685
        if HARDWARE_AVAILABLE:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = adafruit_pca9685.PCA9685(self.i2c)
            self.pca.frequency = self.pwm_frequency
        else:
            self.get_logger().warn('Hardware not available - running in simulation mode')
            self.pca = DummyPCA()
        
        # State tracking
        self.current_action = "stop"
        self.current_speed = 0
        self.extend_start_time = None
        self.is_extending = False
        self.movement_thread = None
        
        # Create service
        self.srv = self.create_service(
            SetActuator,
            'set_actuator',
            self.handle_actuator_request
        )
        
        # Create timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check_callback)
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(
            f'Linear actuator node initialized - IN1: ch{self.in1_channel}, '
            f'IN2: ch{self.in2_channel}, Max extend: {self.max_extend_time}s'
        )
        
        # Initialize to stopped state
        self.stop()
    
    def handle_actuator_request(self, request, response):
        """Handle actuator control service requests"""
        try:
            action = request.action.lower()
            speed = max(0, min(100, request.speed))
            duration = max(0, request.duration)
            
            self.get_logger().info(
                f'Received command: action={action}, speed={speed}, duration={duration}'
            )
            
            # Stop any existing movement
            if self.movement_thread and self.movement_thread.is_alive():
                self.stop()
                time.sleep(0.1)
            
            # Execute action
            if action == 'extend':
                self.extend(speed)
                if duration > 0:
                    # Start timed operation in thread
                    self.movement_thread = threading.Thread(
                        target=self._timed_operation,
                        args=(duration,)
                    )
                    self.movement_thread.daemon = True
                    self.movement_thread.start()
                    
            elif action == 'retract':
                self.retract(speed)
                if duration > 0:
                    self.movement_thread = threading.Thread(
                        target=self._timed_operation,
                        args=(duration,)
                    )
                    self.movement_thread.daemon = True
                    self.movement_thread.start()
                    
            elif action == 'stop':
                self.stop()
            else:
                response.success = False
                response.message = f"Unknown action: {action}. Use 'extend', 'retract', or 'stop'"
                return response
            
            response.success = True
            response.message = f"Actuator {action} at {speed}% speed"
            if duration > 0:
                response.message += f" for {duration} seconds"
            
        except Exception as e:
            self.get_logger().error(f'Error handling request: {str(e)}')
            response.success = False
            response.message = str(e)
        
        return response
    
    def _timed_operation(self, duration):
        """Run operation for specified duration then stop"""
        time.sleep(duration)
        self.stop()
        self.get_logger().info(f'Timed operation completed after {duration} seconds')
    
    def extend(self, speed=100):
        """Extend the linear actuator"""
        duty_cycle = int((speed / 100.0) * 65535)
        
        # Set PWM values
        self.pca.channels[self.in1_channel].duty_cycle = duty_cycle
        self.pca.channels[self.in2_channel].duty_cycle = 0
        
        # Update state
        self.current_action = "extend"
        self.current_speed = speed
        self.is_extending = True
        self.extend_start_time = time.time()
        
        self.get_logger().info(f'Extending at {speed}% (duty cycle: {duty_cycle})')
    
    def retract(self, speed=100):
        """Retract the linear actuator"""
        duty_cycle = int((speed / 100.0) * 65535)
        
        # Set PWM values
        self.pca.channels[self.in1_channel].duty_cycle = 0
        self.pca.channels[self.in2_channel].duty_cycle = duty_cycle
        
        # Update state
        self.current_action = "retract"
        self.current_speed = speed
        self.is_extending = False
        self.extend_start_time = None
        
        self.get_logger().info(f'Retracting at {speed}% (duty cycle: {duty_cycle})')
    
    def stop(self):
        """Stop the linear actuator"""
        # Set both channels to 0
        self.pca.channels[self.in1_channel].duty_cycle = 0
        self.pca.channels[self.in2_channel].duty_cycle = 0
        
        # Update state
        self.current_action = "stop"
        self.current_speed = 0
        self.is_extending = False
        self.extend_start_time = None
        
        self.get_logger().info('Actuator stopped')
    
    def safety_check_callback(self):
        """Check for safety timeout on extension"""
        if self.is_extending and self.extend_start_time:
            elapsed = time.time() - self.extend_start_time
            
            if elapsed >= self.max_extend_time:
                self.get_logger().warn(
                    f'SAFETY STOP: Extension time limit ({self.max_extend_time}s) reached!'
                )
                self.stop()
    
    def publish_status(self):
        """Publish current status (for debugging)"""
        status = {
            'action': self.current_action,
            'speed': self.current_speed,
            'is_extending': self.is_extending,
            'hardware_available': HARDWARE_AVAILABLE
        }
        
        if self.is_extending and self.extend_start_time:
            status['extend_elapsed'] = time.time() - self.extend_start_time
            status['extend_remaining'] = max(0, self.max_extend_time - status['extend_elapsed'])
        
        self.get_logger().debug(f'Status: {status}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down - stopping actuator')
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = LinearActuatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()