#!/usr/bin/env python3
"""
Simple System Info Publisher
Publishes basic system information at regular intervals
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json
from datetime import datetime


class SystemInfoPublisher(Node):
    def __init__(self):
        super().__init__('system_info_publisher')
        self.publisher_ = self.create_publisher(String, 'system_info', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('System Info Publisher started')

    def timer_callback(self):
        msg = String()
        
        # Collect system info
        info = {
            'timestamp': datetime.now().isoformat(),
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent,
            'boot_time': datetime.fromtimestamp(psutil.boot_time()).isoformat(),
            'network_bytes_sent': psutil.net_io_counters().bytes_sent,
            'network_bytes_recv': psutil.net_io_counters().bytes_recv
        }
        
        msg.data = json.dumps(info, indent=2)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Publishing: CPU={info["cpu_percent"]:.1f}%, MEM={info["memory_percent"]:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    publisher = SystemInfoPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
