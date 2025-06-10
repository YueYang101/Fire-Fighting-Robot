#!/usr/bin/env python3
"""
Test script for linear actuator ROS2 service
"""

import rclpy
from rclpy.node import Node
from actuator_interfaces.srv import SetActuator
import sys


class ActuatorTestClient(Node):
    def __init__(self):
        super().__init__('actuator_test_client')
        self.cli = self.create_client(SetActuator, 'set_actuator')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self, action, speed=100, duration=0):
        request = SetActuator.Request()
        request.action = action
        request.speed = speed
        request.duration = duration
        
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        response = self.future.result()
        self.get_logger().info(f'Response: success={response.success}, message="{response.message}"')
        return response


def main():
    rclpy.init()
    client = ActuatorTestClient()
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run actuator_controller test_actuator <action> [speed] [duration]")
        print("  action: extend, retract, stop")
        print("  speed: 0-100 (default: 100)")
        print("  duration: seconds (default: 0 = continuous)")
        return
    
    action = sys.argv[1]
    speed = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 0
    
    client.send_request(action, speed, duration)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()