#!/usr/bin/env python3
"""
ROS2 System Monitor Node
Publishes system metrics for Raspberry Pi
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Temperature
import psutil
import os
import subprocess
import json
from datetime import datetime


class SystemMonitorNode(Node):
   def __init__(self):
       super().__init__('system_monitor_node')
       
       # Declare parameters
       self.declare_parameter('publish_rate', 1.0)  # Hz
       self.declare_parameter('enable_detailed_logs', False)
       
       # Get parameters
       self.publish_rate = self.get_parameter('publish_rate').value
       self.enable_detailed_logs = self.get_parameter('enable_detailed_logs').value
       
       # Create publishers (only standard messages for now)
       self.temp_pub = self.create_publisher(Temperature, 'system/temperature', 10)
       self.cpu_pub = self.create_publisher(Float32, 'system/cpu_usage', 10)
       self.memory_pub = self.create_publisher(Float32, 'system/memory_usage', 10)
       self.json_pub = self.create_publisher(String, 'system/status_json', 10)
       
       # Create timer
       self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
       
       self.get_logger().info(f'System Monitor Node started (rate: {self.publish_rate} Hz)')
   
   def get_cpu_temperature(self):
       """Get CPU temperature"""
       try:
           with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
               return float(f.read()) / 1000.0
       except:
           try:
               result = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
               return float(result.replace('temp=', '').replace("'C\n", ''))
           except:
               return -1.0
   
   def get_gpu_temperature(self):
       """Get GPU temperature for Raspberry Pi"""
       try:
           result = subprocess.check_output(['vcgencmd', 'measure_temp', 'gpu']).decode()
           return float(result.replace('temp=', '').replace("'C\n", ''))
       except:
           return -1.0
   
   def timer_callback(self):
       """Main timer callback to publish all metrics"""
       # Temperature
       temp_msg = Temperature()
       temp_msg.header.stamp = self.get_clock().now().to_msg()
       temp_msg.header.frame_id = "cpu"
       temp_msg.temperature = self.get_cpu_temperature()
       temp_msg.variance = 0.0
       self.temp_pub.publish(temp_msg)
       
       # Simple CPU and Memory
       cpu_msg = Float32()
       cpu_msg.data = psutil.cpu_percent(interval=0.1)
       self.cpu_pub.publish(cpu_msg)
       
       memory_msg = Float32()
       memory_msg.data = psutil.virtual_memory().percent
       self.memory_pub.publish(memory_msg)
       
       # Get system info
       system_info = {
           'timestamp': datetime.now().isoformat(),
           'hostname': os.uname().nodename,
           'cpu_temperature': self.get_cpu_temperature(),
           'gpu_temperature': self.get_gpu_temperature(),
           'cpu_percent': cpu_msg.data,
           'memory_percent': memory_msg.data,
           'load_average': psutil.getloadavg(),
           'disk_usage': psutil.disk_usage('/').percent,
           'uptime_hours': (psutil.time.time() - psutil.boot_time()) / 3600.0,
           'process_count': len(psutil.pids())
       }
       
       # Publish as JSON
       json_msg = String()
       json_msg.data = json.dumps(system_info, indent=2)
       self.json_pub.publish(json_msg)
       
       if self.enable_detailed_logs:
           self.get_logger().info(
               f'CPU: {cpu_msg.data:.1f}% | '
               f'Memory: {memory_msg.data:.1f}% | '
               f'Temp: {system_info["cpu_temperature"]:.1f}°C'
           )


def main(args=None):
   rclpy.init(args=args)
   node = SystemMonitorNode()
   
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()