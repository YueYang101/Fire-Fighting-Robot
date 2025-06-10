#!/usr/bin/env python3
"""
Raspberry Pi System and ROS Monitor
Monitors system load, temperature, runtime, and ROS metrics
Can be run as a standalone script or as a ROS node
"""

import psutil
import os
import time
import json
import subprocess
from datetime import datetime, timedelta
from collections import defaultdict

# ROS imports (optional - will work without ROS)
try:
    import rospy
    from std_msgs.msg import String, Float32
    import rostopic
    import rosnode
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS not available - running in standalone mode")


class SystemMonitor:
    """Monitor system metrics on Raspberry Pi"""
    
    def __init__(self):
        self.start_time = time.time()
        self.cpu_temps = []
        self.ros_topic_stats = defaultdict(dict)
        
    def get_cpu_temperature(self):
        """Get CPU temperature on Raspberry Pi"""
        temp = None
        
        # Method 1: Try thermal zone (works on most Linux systems)
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
                return temp
        except:
            pass
        
        # Method 2: Try vcgencmd (Raspberry Pi specific)
        try:
            result = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
            temp = float(result.replace('temp=', '').replace("'C\n", ''))
            return temp
        except:
            pass
        
        # Method 3: Try sensors command
        try:
            result = subprocess.check_output(['sensors']).decode()
            for line in result.split('\n'):
                if 'Core 0' in line or 'temp1' in line:
                    temp_str = line.split('+')[1].split('°')[0]
                    temp = float(temp_str)
                    return temp
        except:
            pass
        
        return -1  # Temperature not available
    
    def get_system_load(self):
        """Get comprehensive system load information"""
        return {
            'cpu_percent': psutil.cpu_percent(interval=1),
            'cpu_percent_per_core': psutil.cpu_percent(interval=1, percpu=True),
            'cpu_count': psutil.cpu_count(),
            'cpu_freq': psutil.cpu_freq()._asdict() if psutil.cpu_freq() else None,
            'load_average': {
                '1min': psutil.getloadavg()[0],
                '5min': psutil.getloadavg()[1],
                '15min': psutil.getloadavg()[2]
            },
            'memory': {
                'percent': psutil.virtual_memory().percent,
                'used_gb': psutil.virtual_memory().used / (1024**3),
                'total_gb': psutil.virtual_memory().total / (1024**3),
                'available_gb': psutil.virtual_memory().available / (1024**3)
            },
            'swap': {
                'percent': psutil.swap_memory().percent,
                'used_gb': psutil.swap_memory().used / (1024**3),
                'total_gb': psutil.swap_memory().total / (1024**3)
            },
            'disk': {
                'percent': psutil.disk_usage('/').percent,
                'used_gb': psutil.disk_usage('/').used / (1024**3),
                'total_gb': psutil.disk_usage('/').total / (1024**3),
                'free_gb': psutil.disk_usage('/').free / (1024**3)
            }
        }
    
    def get_runtime_info(self):
        """Get system runtime and process information"""
        current_time = time.time()
        uptime_seconds = current_time - psutil.boot_time()
        script_runtime = current_time - self.start_time
        
        return {
            'system_uptime': str(timedelta(seconds=int(uptime_seconds))),
            'system_uptime_seconds': uptime_seconds,
            'script_runtime': str(timedelta(seconds=int(script_runtime))),
            'script_runtime_seconds': script_runtime,
            'boot_time': datetime.fromtimestamp(psutil.boot_time()).strftime('%Y-%m-%d %H:%M:%S'),
            'current_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'process_count': len(psutil.pids()),
            'network_connections': len(psutil.net_connections())
        }
    
    def get_network_stats(self):
        """Get network statistics"""
        net_io = psutil.net_io_counters()
        return {
            'bytes_sent_mb': net_io.bytes_sent / (1024**2),
            'bytes_recv_mb': net_io.bytes_recv / (1024**2),
            'packets_sent': net_io.packets_sent,
            'packets_recv': net_io.packets_recv,
            'errors_in': net_io.errin,
            'errors_out': net_io.errout
        }
    
    def get_process_info(self, top_n=5):
        """Get top N processes by CPU and memory usage"""
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
            try:
                processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        
        # Sort by CPU usage
        top_cpu = sorted(processes, key=lambda x: x['cpu_percent'], reverse=True)[:top_n]
        
        # Sort by memory usage
        top_memory = sorted(processes, key=lambda x: x['memory_percent'], reverse=True)[:top_n]
        
        return {
            'top_cpu_processes': top_cpu,
            'top_memory_processes': top_memory
        }
    
    def get_ros_load(self):
        """Get ROS-specific load information"""
        if not ROS_AVAILABLE:
            return {'error': 'ROS not available'}
        
        ros_info = {
            'ros_available': False,
            'nodes': [],
            'topics': [],
            'services': [],
            'topic_stats': {},
            'node_stats': {}
        }
        
        try:
            # Check if roscore is running
            master = rospy.get_master()
            master.getPid()
            ros_info['ros_available'] = True
            
            # Get list of nodes
            ros_info['nodes'] = rosnode.get_node_names()
            
            # Get list of topics with their types
            topics_and_types = rospy.get_published_topics()
            ros_info['topics'] = [{'name': t[0], 'type': t[1]} for t in topics_and_types]
            
            # Get services
            ros_info['services'] = sorted(rospy.get_services())
            
            # Get topic statistics (bandwidth, frequency)
            for topic, msg_type in topics_and_types[:10]:  # Limit to first 10 topics
                try:
                    # Get topic bandwidth
                    topic_info = subprocess.check_output(
                        ['rostopic', 'bw', topic], 
                        timeout=2
                    ).decode().strip()
                    
                    if 'average:' in topic_info:
                        bandwidth = topic_info.split('average:')[1].strip()
                        ros_info['topic_stats'][topic] = {'bandwidth': bandwidth}
                except:
                    pass
            
            # Get node CPU and memory usage
            for node in ros_info['nodes'][:10]:  # Limit to first 10 nodes
                try:
                    # Find process ID for the node
                    node_info = rosnode.get_node_info_description(node)
                    
                    # Try to find the process
                    for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
                        if node in proc.info['name'] or proc.info['name'] in node:
                            ros_info['node_stats'][node] = {
                                'pid': proc.info['pid'],
                                'cpu_percent': proc.info['cpu_percent'],
                                'memory_percent': proc.info['memory_percent']
                            }
                            break
                except:
                    pass
            
        except Exception as e:
            ros_info['error'] = str(e)
        
        return ros_info
    
    def get_all_metrics(self):
        """Get all system and ROS metrics"""
        metrics = {
            'timestamp': datetime.now().isoformat(),
            'temperature': self.get_cpu_temperature(),
            'system_load': self.get_system_load(),
            'runtime': self.get_runtime_info(),
            'network': self.get_network_stats(),
            'processes': self.get_process_info(),
            'ros_load': self.get_ros_load() if ROS_AVAILABLE else None
        }
        return metrics
    
    def print_summary(self, metrics):
        """Print a formatted summary of the metrics"""
        print("\n" + "="*60)
        print(f"System Monitor Report - {metrics['timestamp']}")
        print("="*60)
        
        # Temperature
        print(f"\n🌡️  CPU Temperature: {metrics['temperature']:.1f}°C")
        
        # System Load
        load = metrics['system_load']
        print(f"\n💻 System Load:")
        print(f"   CPU Usage: {load['cpu_percent']:.1f}%")
        print(f"   Load Average: {load['load_average']['1min']:.2f} (1m), "
              f"{load['load_average']['5min']:.2f} (5m), "
              f"{load['load_average']['15min']:.2f} (15m)")
        print(f"   Memory: {load['memory']['percent']:.1f}% "
              f"({load['memory']['used_gb']:.1f}/{load['memory']['total_gb']:.1f} GB)")
        print(f"   Disk: {load['disk']['percent']:.1f}% "
              f"({load['disk']['used_gb']:.1f}/{load['disk']['total_gb']:.1f} GB)")
        
        # Runtime
        runtime = metrics['runtime']
        print(f"\n⏱️  Runtime Info:")
        print(f"   System Uptime: {runtime['system_uptime']}")
        print(f"   Script Runtime: {runtime['script_runtime']}")
        print(f"   Process Count: {runtime['process_count']}")
        
        # Top Processes
        processes = metrics['processes']
        print(f"\n📊 Top CPU Processes:")
        for proc in processes['top_cpu_processes'][:3]:
            print(f"   {proc['name']}: {proc['cpu_percent']:.1f}%")
        
        # ROS Info
        if metrics['ros_load'] and metrics['ros_load'].get('ros_available'):
            ros = metrics['ros_load']
            print(f"\n🤖 ROS Status:")
            print(f"   Active Nodes: {len(ros['nodes'])}")
            print(f"   Active Topics: {len(ros['topics'])}")
            print(f"   Services: {len(ros['services'])}")
            
            if ros['node_stats']:
                print(f"\n   Node CPU Usage:")
                for node, stats in list(ros['node_stats'].items())[:3]:
                    print(f"     {node}: {stats['cpu_percent']:.1f}% CPU, "
                          f"{stats['memory_percent']:.1f}% Memory")


class ROSSystemPublisher:
    """Publish system metrics as ROS topics"""
    
    def __init__(self, monitor):
        self.monitor = monitor
        
        # Initialize ROS node
        rospy.init_node('system_monitor', anonymous=True)
        
        # Publishers
        self.temp_pub = rospy.Publisher('/system/temperature', Float32, queue_size=1)
        self.cpu_pub = rospy.Publisher('/system/cpu_usage', Float32, queue_size=1)
        self.memory_pub = rospy.Publisher('/system/memory_usage', Float32, queue_size=1)
        self.full_status_pub = rospy.Publisher('/system/full_status', String, queue_size=1)
        
        # Timer for periodic publishing
        self.timer = rospy.Timer(rospy.Duration(2.0), self.publish_metrics)
        
    def publish_metrics(self, event):
        """Publish metrics to ROS topics"""
        metrics = self.monitor.get_all_metrics()
        
        # Publish individual metrics
        self.temp_pub.publish(Float32(metrics['temperature']))
        self.cpu_pub.publish(Float32(metrics['system_load']['cpu_percent']))
        self.memory_pub.publish(Float32(metrics['system_load']['memory']['percent']))
        
        # Publish full status as JSON
        self.full_status_pub.publish(String(json.dumps(metrics, indent=2)))


def main():
    """Main function to run the monitor"""
    monitor = SystemMonitor()
    
    # Check if we should run as ROS node
    use_ros = '--ros' in os.sys.argv and ROS_AVAILABLE
    
    if use_ros:
        print("Starting ROS System Monitor Node...")
        publisher = ROSSystemPublisher(monitor)
        
        # Keep publishing until shutdown
        rospy.spin()
    else:
        print("Starting System Monitor (Standalone Mode)...")
        print("Add --ros flag to publish metrics to ROS topics")
        
        try:
            while True:
                # Get all metrics
                metrics = monitor.get_all_metrics()
                
                # Print summary
                monitor.print_summary(metrics)
                
                # Optional: Save to file
                if '--save' in os.sys.argv:
                    filename = f"system_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                    with open(filename, 'w') as f:
                        json.dump(metrics, f, indent=2)
                    print(f"\nMetrics saved to {filename}")
                
                # Wait before next update
                time.sleep(5)
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped.")


if __name__ == '__main__':
    main()