#!/usr/bin/env python3
"""
Raspberry Pi System Monitor
Monitors system load, temperature, runtime, and other metrics
Pure system monitoring without ROS dependencies
"""

import psutil
import os
import time
import json
import subprocess
from datetime import datetime, timedelta
from collections import defaultdict


class SystemMonitor:
    """Monitor system metrics on Raspberry Pi"""
    
    def __init__(self):
        self.start_time = time.time()
        self.cpu_temps = []
        self.cpu_history = []
        self.memory_history = []
        
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
    
    def get_gpu_temperature(self):
        """Get GPU temperature on Raspberry Pi"""
        try:
            result = subprocess.check_output(['vcgencmd', 'measure_temp', 'gpu']).decode()
            temp = float(result.replace('temp=', '').replace("'C\n", ''))
            return temp
        except:
            return -1
    
    def get_system_load(self):
        """Get comprehensive system load information"""
        return {
            'cpu_percent': psutil.cpu_percent(interval=1),
            'cpu_percent_per_core': psutil.cpu_percent(interval=1, percpu=True),
            'cpu_count': psutil.cpu_count(),
            'cpu_count_logical': psutil.cpu_count(logical=True),
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
                'available_gb': psutil.virtual_memory().available / (1024**3),
                'free_gb': psutil.virtual_memory().free / (1024**3),
                'cached_gb': psutil.virtual_memory().cached / (1024**3) if hasattr(psutil.virtual_memory(), 'cached') else 0,
                'buffers_gb': psutil.virtual_memory().buffers / (1024**3) if hasattr(psutil.virtual_memory(), 'buffers') else 0
            },
            'swap': {
                'percent': psutil.swap_memory().percent,
                'used_gb': psutil.swap_memory().used / (1024**3),
                'total_gb': psutil.swap_memory().total / (1024**3),
                'free_gb': psutil.swap_memory().free / (1024**3)
            },
            'disk': self.get_disk_usage()
        }
    
    def get_disk_usage(self):
        """Get disk usage for all mounted partitions"""
        disk_info = {}
        partitions = psutil.disk_partitions()
        
        for partition in partitions:
            try:
                usage = psutil.disk_usage(partition.mountpoint)
                disk_info[partition.mountpoint] = {
                    'device': partition.device,
                    'fstype': partition.fstype,
                    'percent': usage.percent,
                    'used_gb': usage.used / (1024**3),
                    'total_gb': usage.total / (1024**3),
                    'free_gb': usage.free / (1024**3)
                }
            except PermissionError:
                # This can happen on some mount points
                continue
        
        return disk_info
    
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
            'thread_count': sum([p.num_threads() for p in psutil.process_iter(['num_threads'])]),
            'network_connections': len(psutil.net_connections()),
            'users': [user._asdict() for user in psutil.users()]
        }
    
    def get_network_stats(self):
        """Get network statistics"""
        net_io = psutil.net_io_counters()
        net_if_addrs = psutil.net_if_addrs()
        net_if_stats = psutil.net_if_stats()
        
        interfaces = {}
        for interface, addrs in net_if_addrs.items():
            if interface in net_if_stats:
                stats = net_if_stats[interface]
                interfaces[interface] = {
                    'is_up': stats.isup,
                    'speed': stats.speed,
                    'mtu': stats.mtu,
                    'addresses': [{'family': addr.family.name, 'address': addr.address} for addr in addrs]
                }
        
        return {
            'total': {
                'bytes_sent_mb': net_io.bytes_sent / (1024**2),
                'bytes_recv_mb': net_io.bytes_recv / (1024**2),
                'packets_sent': net_io.packets_sent,
                'packets_recv': net_io.packets_recv,
                'errors_in': net_io.errin,
                'errors_out': net_io.errout,
                'drop_in': net_io.dropin,
                'drop_out': net_io.dropout
            },
            'interfaces': interfaces
        }
    
    def get_process_info(self, top_n=5):
        """Get top N processes by CPU and memory usage"""
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent', 'create_time']):
            try:
                proc_info = proc.info
                proc_info['runtime'] = time.time() - proc_info['create_time']
                processes.append(proc_info)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        
        # Sort by CPU usage
        top_cpu = sorted(processes, key=lambda x: x['cpu_percent'], reverse=True)[:top_n]
        
        # Sort by memory usage
        top_memory = sorted(processes, key=lambda x: x['memory_percent'], reverse=True)[:top_n]
        
        return {
            'top_cpu_processes': top_cpu,
            'top_memory_processes': top_memory,
            'total_processes': len(processes)
        }
    
    def get_raspberry_pi_info(self):
        """Get Raspberry Pi specific information"""
        rpi_info = {}
        
        # Get Pi model
        try:
            with open('/proc/device-tree/model', 'r') as f:
                rpi_info['model'] = f.read().strip('\x00')
        except:
            rpi_info['model'] = 'Unknown'
        
        # Get throttling status
        try:
            result = subprocess.check_output(['vcgencmd', 'get_throttled']).decode()
            throttled = int(result.split('=')[1].strip(), 16)
            rpi_info['throttled'] = {
                'under_voltage': bool(throttled & 0x1),
                'frequency_capped': bool(throttled & 0x2),
                'currently_throttled': bool(throttled & 0x4),
                'soft_temp_limit': bool(throttled & 0x8),
                'under_voltage_occurred': bool(throttled & 0x10000),
                'frequency_cap_occurred': bool(throttled & 0x20000),
                'throttling_occurred': bool(throttled & 0x40000),
                'soft_temp_limit_occurred': bool(throttled & 0x80000)
            }
        except:
            rpi_info['throttled'] = None
        
        # Get voltage
        try:
            result = subprocess.check_output(['vcgencmd', 'measure_volts']).decode()
            rpi_info['core_voltage'] = float(result.split('=')[1].strip().replace('V', ''))
        except:
            rpi_info['core_voltage'] = None
        
        # Get clock speeds
        clocks = ['arm', 'core', 'h264', 'isp', 'v3d', 'uart', 'pwm', 'emmc', 'pixel', 'vec', 'hdmi', 'dpi']
        rpi_info['clocks'] = {}
        for clock in clocks:
            try:
                result = subprocess.check_output(['vcgencmd', 'measure_clock', clock]).decode()
                freq = int(result.split('=')[1].strip())
                rpi_info['clocks'][clock] = freq / 1000000  # Convert to MHz
            except:
                pass
        
        return rpi_info
    
    def get_all_metrics(self):
        """Get all system metrics"""
        metrics = {
            'timestamp': datetime.now().isoformat(),
            'hostname': os.uname().nodename,
            'temperature': {
                'cpu': self.get_cpu_temperature(),
                'gpu': self.get_gpu_temperature()
            },
            'system_load': self.get_system_load(),
            'runtime': self.get_runtime_info(),
            'network': self.get_network_stats(),
            'processes': self.get_process_info(),
            'raspberry_pi': self.get_raspberry_pi_info()
        }
        return metrics
    
    def print_summary(self, metrics):
        """Print a formatted summary of the metrics"""
        print("\n" + "="*60)
        print(f"System Monitor Report - {metrics['timestamp']}")
        print(f"Hostname: {metrics['hostname']}")
        print("="*60)
        
        # Temperature
        temp = metrics['temperature']
        print(f"\n🌡️  Temperature:")
        print(f"   CPU: {temp['cpu']:.1f}°C")
        if temp['gpu'] > 0:
            print(f"   GPU: {temp['gpu']:.1f}°C")
        
        # System Load
        load = metrics['system_load']
        print(f"\n💻 System Load:")
        print(f"   CPU Usage: {load['cpu_percent']:.1f}% (Cores: {load['cpu_count']})")
        if load['cpu_percent_per_core']:
            print(f"   Per Core: {[f'{x:.1f}%' for x in load['cpu_percent_per_core']]}")
        print(f"   Load Average: {load['load_average']['1min']:.2f} (1m), "
              f"{load['load_average']['5min']:.2f} (5m), "
              f"{load['load_average']['15min']:.2f} (15m)")
        print(f"   Memory: {load['memory']['percent']:.1f}% "
              f"({load['memory']['used_gb']:.1f}/{load['memory']['total_gb']:.1f} GB)")
        if load['swap']['total_gb'] > 0:
            print(f"   Swap: {load['swap']['percent']:.1f}% "
                  f"({load['swap']['used_gb']:.1f}/{load['swap']['total_gb']:.1f} GB)")
        
        # Disk Usage
        print(f"\n💾 Disk Usage:")
        for mount, disk in load['disk'].items():
            if mount == '/':
                print(f"   Root: {disk['percent']:.1f}% "
                      f"({disk['used_gb']:.1f}/{disk['total_gb']:.1f} GB)")
            elif disk['total_gb'] > 1:  # Only show significant partitions
                print(f"   {mount}: {disk['percent']:.1f}% "
                      f"({disk['used_gb']:.1f}/{disk['total_gb']:.1f} GB)")
        
        # Runtime
        runtime = metrics['runtime']
        print(f"\n⏱️  Runtime Info:")
        print(f"   System Uptime: {runtime['system_uptime']}")
        print(f"   Script Runtime: {runtime['script_runtime']}")
        print(f"   Processes: {runtime['process_count']} | Threads: {runtime['thread_count']}")
        print(f"   Network Connections: {runtime['network_connections']}")
        
        # Network
        net = metrics['network']['total']
        print(f"\n🌐 Network Stats:")
        print(f"   Sent: {net['bytes_sent_mb']:.1f} MB | Received: {net['bytes_recv_mb']:.1f} MB")
        print(f"   Packets: {net['packets_sent']:,} sent | {net['packets_recv']:,} received")
        
        # Top Processes
        processes = metrics['processes']
        print(f"\n📊 Top CPU Processes:")
        for proc in processes['top_cpu_processes'][:3]:
            runtime_str = str(timedelta(seconds=int(proc['runtime'])))
            print(f"   {proc['name'][:20]:20} PID:{proc['pid']:6} CPU:{proc['cpu_percent']:5.1f}% Runtime:{runtime_str}")
        
        print(f"\n📊 Top Memory Processes:")
        for proc in processes['top_memory_processes'][:3]:
            print(f"   {proc['name'][:20]:20} PID:{proc['pid']:6} MEM:{proc['memory_percent']:5.1f}%")
        
        # Raspberry Pi Info
        rpi = metrics['raspberry_pi']
        if rpi['model'] != 'Unknown':
            print(f"\n🥧 Raspberry Pi Info:")
            print(f"   Model: {rpi['model']}")
            if rpi['core_voltage']:
                print(f"   Core Voltage: {rpi['core_voltage']:.2f}V")
            if rpi['throttled']:
                throttle_status = []
                if rpi['throttled']['currently_throttled']:
                    throttle_status.append('THROTTLED')
                if rpi['throttled']['under_voltage']:
                    throttle_status.append('UNDER-VOLTAGE')
                if rpi['throttled']['frequency_capped']:
                    throttle_status.append('FREQ-CAPPED')
                if throttle_status:
                    print(f"   ⚠️  Status: {', '.join(throttle_status)}")
                else:
                    print(f"   ✅ Status: OK")
            if rpi['clocks']:
                print(f"   Clock Speeds: ARM={rpi['clocks'].get('arm', 0):.0f}MHz")

    def export_metrics(self, metrics, format='json', filename=None):
        """Export metrics to file in various formats"""
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"system_metrics_{timestamp}.{format}"
        
        if format == 'json':
            with open(filename, 'w') as f:
                json.dump(metrics, f, indent=2)
        elif format == 'csv':
            # Flatten the metrics for CSV export
            import csv
            flat_metrics = self._flatten_dict(metrics)
            with open(filename, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=flat_metrics.keys())
                writer.writeheader()
                writer.writerow(flat_metrics)
        
        return filename
    
    def _flatten_dict(self, d, parent_key='', sep='_'):
        """Flatten nested dictionary for CSV export"""
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            elif isinstance(v, list):
                items.append((new_key, str(v)))
            else:
                items.append((new_key, v))
        return dict(items)


def main():
    """Main function to run the monitor"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Raspberry Pi System Monitor')
    parser.add_argument('--interval', type=int, default=5, help='Update interval in seconds')
    parser.add_argument('--save', action='store_true', help='Save metrics to file')
    parser.add_argument('--format', choices=['json', 'csv'], default='json', help='Export format')
    parser.add_argument('--once', action='store_true', help='Run once and exit')
    
    args = parser.parse_args()
    
    monitor = SystemMonitor()
    
    print("Starting System Monitor...")
    print(f"Update interval: {args.interval} seconds")
    if args.save:
        print(f"Saving metrics in {args.format} format")
    
    try:
        while True:
            # Get all metrics
            metrics = monitor.get_all_metrics()
            
            # Print summary
            monitor.print_summary(metrics)
            
            # Save to file if requested
            if args.save:
                filename = monitor.export_metrics(metrics, format=args.format)
                print(f"\n💾 Metrics saved to {filename}")
            
            # Exit if running once
            if args.once:
                break
            
            # Wait before next update
            time.sleep(args.interval)
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")


if __name__ == '__main__':
    main()