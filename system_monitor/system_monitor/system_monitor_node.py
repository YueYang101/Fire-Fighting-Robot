#!/usr/bin/env python3
"""
ROS2 System Monitor Node
Publishes system metrics for Raspberry Pi
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32, String
from sensor_msgs.msg import Temperature
import psutil
import os
import subprocess
import json
from datetime import datetime

# Import custom messages (will be generated)
from system_monitor.msg import (
    SystemLoad, ProcessInfo, NetworkStats, 
    DiskUsage, SystemStatus
)


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('enable_detailed_logs', False)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_detailed_logs = self.get_parameter('enable_detailed_logs').value
        
        # Create publishers
        self.temp_pub = self.create_publisher(Temperature, 'system/temperature', 10)
        self.cpu_pub = self.create_publisher(Float32, 'system/cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, 'system/memory_usage', 10)
        self.system_load_pub = self.create_publisher(SystemLoad, 'system/load', 10)
        self.network_pub = self.create_publisher(NetworkStats, 'system/network', 10)
        self.status_pub = self.create_publisher(SystemStatus, 'system/status', 10)
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
                return float(result.replace('temp=', '').replace("'C\\n", ''))
            except:
                return -1.0
    
    def get_gpu_temperature(self):
        """Get GPU temperature for Raspberry Pi"""
        try:
            result = subprocess.check_output(['vcgencmd', 'measure_temp', 'gpu']).decode()
            return float(result.replace('temp=', '').replace("'C\\n", ''))
        except:
            return -1.0
    
    def get_system_load(self):
        """Get system load information"""
        msg = SystemLoad()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # CPU info
        msg.cpu_percent = psutil.cpu_percent(interval=0.1)
        msg.cpu_percent_per_core = list(psutil.cpu_percent(interval=0.1, percpu=True))
        
        # Memory info
        mem = psutil.virtual_memory()
        msg.memory_percent = mem.percent
        msg.memory_used_gb = mem.used / (1024**3)
        msg.memory_total_gb = mem.total / (1024**3)
        
        # Swap info
        swap = psutil.swap_memory()
        msg.swap_percent = swap.percent
        
        # Load average
        msg.load_average = list(psutil.getloadavg())
        
        return msg
    
    def get_network_stats(self):
        """Get network statistics"""
        msg = NetworkStats()
        net_io = psutil.net_io_counters()
        
        msg.bytes_sent_mb = net_io.bytes_sent / (1024**2)
        msg.bytes_recv_mb = net_io.bytes_recv / (1024**2)
        msg.packets_sent = net_io.packets_sent
        msg.packets_recv = net_io.packets_recv
        msg.errors_in = net_io.errin
        msg.errors_out = net_io.errout
        
        return msg
    
    def get_disk_usage(self):
        """Get disk usage for all partitions"""
        disk_msgs = []
        partitions = psutil.disk_partitions()
        
        for partition in partitions:
            try:
                usage = psutil.disk_usage(partition.mountpoint)
                msg = DiskUsage()
                msg.mount_point = partition.mountpoint
                msg.device = partition.device
                msg.percent = usage.percent
                msg.used_gb = usage.used / (1024**3)
                msg.total_gb = usage.total / (1024**3)
                msg.free_gb = usage.free / (1024**3)
                disk_msgs.append(msg)
            except PermissionError:
                continue
        
        return disk_msgs
    
    def get_top_processes(self, by='cpu', top_n=5):
        """Get top N processes by CPU or memory"""
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
            try:
                processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        
        key = 'cpu_percent' if by == 'cpu' else 'memory_percent'
        sorted_procs = sorted(processes, key=lambda x: x[key], reverse=True)[:top_n]
        
        process_msgs = []
        for proc in sorted_procs:
            msg = ProcessInfo()
            msg.name = proc['name']
            msg.pid = proc['pid']
            msg.cpu_percent = proc['cpu_percent']
            msg.memory_percent = proc['memory_percent']
            process_msgs.append(msg)
        
        return process_msgs
    
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
        
        # Detailed system load
        system_load = self.get_system_load()
        self.system_load_pub.publish(system_load)
        
        # Network stats
        network_stats = self.get_network_stats()
        self.network_pub.publish(network_stats)
        
        # Complete system status
        status_msg = SystemStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.hostname = os.uname().nodename
        status_msg.cpu_temperature = self.get_cpu_temperature()
        status_msg.gpu_temperature = self.get_gpu_temperature()
        status_msg.system_load = system_load
        status_msg.network_stats = network_stats
        status_msg.disk_usage = self.get_disk_usage()
        status_msg.top_cpu_processes = self.get_top_processes('cpu')
        status_msg.top_memory_processes = self.get_top_processes('memory')
        status_msg.process_count = len(psutil.pids())
        status_msg.thread_count = sum([p.num_threads() for p in psutil.process_iter(['num_threads'])])
        status_msg.uptime_hours = (psutil.time.time() - psutil.boot_time()) / 3600.0
        
        self.status_pub.publish(status_msg)
        
        # Also publish as JSON for easy consumption
        json_data = {
            'timestamp': datetime.now().isoformat(),
            'hostname': status_msg.hostname,
            'cpu_temperature': status_msg.cpu_temperature,
            'gpu_temperature': status_msg.gpu_temperature,
            'cpu_percent': cpu_msg.data,
            'memory_percent': memory_msg.data,
            'uptime_hours': status_msg.uptime_hours,
            'process_count': status_msg.process_count
        }
        json_msg = String()
        json_msg.data = json.dumps(json_data)
        self.json_pub.publish(json_msg)
        
        if self.enable_detailed_logs:
            self.get_logger().info(
                f'CPU: {cpu_msg.data:.1f}% | '
                f'Memory: {memory_msg.data:.1f}% | '
                f'Temp: {status_msg.cpu_temperature:.1f}°C'
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
