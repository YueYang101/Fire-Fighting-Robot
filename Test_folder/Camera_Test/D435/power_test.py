#!/usr/bin/env python3
"""
Intel RealSense D435 Power and Connection Diagnostics for Raspberry Pi
Checks power status, USB connection, and helps diagnose frame timeout issues
"""

import pyrealsense2 as rs
import numpy as np
import time
import subprocess
import os
import json
from datetime import datetime

class RealSenseDiagnostics:
    def __init__(self):
        self.results = {
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'tests': {}
        }
        
    def run_command(self, cmd):
        """Run system command and return output"""
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            return result.stdout.strip()
        except:
            return "Command failed"
    
    def check_usb_power(self):
        """Check USB power status and current draw"""
        print("\n" + "="*60)
        print("USB POWER DIAGNOSTICS")
        print("="*60)
        
        results = {}
        
        # Check USB devices and their power status
        print("\n1. USB Device Tree:")
        lsusb_output = self.run_command("lsusb -t")
        print(lsusb_output)
        
        # Find RealSense device
        print("\n2. RealSense USB Details:")
        lsusb_verbose = self.run_command("lsusb -v 2>/dev/null | grep -A 10 -B 5 'Intel.*RealSense'")
        if lsusb_verbose:
            print(lsusb_verbose)
            # Extract power information
            max_power = self.run_command("lsusb -v 2>/dev/null | grep -A 20 'Intel.*RealSense' | grep MaxPower")
            if max_power:
                print(f"\nMax Power Required: {max_power}")
                results['max_power'] = max_power
        
        # Check USB port power limits
        print("\n3. USB Port Power Status:")
        usb_power_files = []
        for root, dirs, files in os.walk("/sys/bus/usb/devices/"):
            if "bMaxPower" in files:
                usb_power_files.append(os.path.join(root, "bMaxPower"))
        
        for power_file in usb_power_files:
            try:
                with open(power_file, 'r') as f:
                    power_value = f.read().strip()
                device_path = os.path.dirname(power_file)
                product_file = os.path.join(device_path, "product")
                if os.path.exists(product_file):
                    with open(product_file, 'r') as f:
                        product = f.read().strip()
                        if "RealSense" in product or "Intel" in product:
                            print(f"  Device: {product}")
                            print(f"  Max Power: {power_value} mA")
                            results['device_power'] = f"{power_value} mA"
            except:
                pass
        
        # Check if using USB 3.0
        print("\n4. USB Speed Check:")
        usb_speed = self.run_command("lsusb -t | grep -A 1 -B 1 'Intel'")
        print(usb_speed)
        if "5000M" in usb_speed:
            print("✓ Running at USB 3.0 speed (5000M)")
            results['usb_speed'] = "USB 3.0 (5000M)"
        elif "480M" in usb_speed:
            print("⚠ Running at USB 2.0 speed (480M) - This may cause issues!")
            results['usb_speed'] = "USB 2.0 (480M) - WARNING"
        
        self.results['tests']['usb_power'] = results
        return results
    
    def check_realsense_connection(self):
        """Test basic RealSense connection with detailed error handling"""
        print("\n" + "="*60)
        print("REALSENSE CONNECTION TEST")
        print("="*60)
        
        results = {'status': 'unknown', 'details': {}}
        
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            
            if len(devices) == 0:
                print("❌ No RealSense devices found!")
                results['status'] = 'no_device'
                
                # Check if device appears in system
                print("\nChecking system recognition:")
                lsusb = self.run_command("lsusb | grep Intel")
                if lsusb:
                    print(f"System sees device: {lsusb}")
                    print("Device visible to system but not to RealSense SDK")
                    results['details']['system_visible'] = True
                else:
                    print("Device not visible to system")
                    results['details']['system_visible'] = False
                
                return results
            
            print(f"✓ Found {len(devices)} RealSense device(s)")
            results['status'] = 'connected'
            results['details']['device_count'] = len(devices)
            
            for i, dev in enumerate(devices):
                print(f"\nDevice {i}:")
                device_info = {}
                
                try:
                    device_info['name'] = dev.get_info(rs.camera_info.name)
                    device_info['serial'] = dev.get_info(rs.camera_info.serial_number)
                    device_info['firmware'] = dev.get_info(rs.camera_info.firmware_version)
                    device_info['usb_type'] = dev.get_info(rs.camera_info.usb_type_descriptor)
                    
                    for key, value in device_info.items():
                        print(f"  {key}: {value}")
                    
                    results['details'][f'device_{i}'] = device_info
                    
                    # Check if firmware update recommended
                    if device_info['firmware'] < "05.13.00":
                        print("  ⚠ Firmware update recommended for better stability")
                        
                except Exception as e:
                    print(f"  Error getting device info: {e}")
                    
        except Exception as e:
            print(f"❌ Connection error: {e}")
            results['status'] = 'error'
            results['details']['error'] = str(e)
            
        self.results['tests']['connection'] = results
        return results
    
    def test_minimal_stream(self):
        """Test with minimal configuration to isolate power issues"""
        print("\n" + "="*60)
        print("MINIMAL STREAM TEST")
        print("="*60)
        
        test_configs = [
            {
                'name': 'Ultra Low - Depth 240x180 @ 6fps',
                'stream': rs.stream.depth,
                'width': 240,
                'height': 180,
                'format': rs.format.z16,
                'fps': 6
            },
            {
                'name': 'Low - Depth 480x270 @ 15fps',
                'stream': rs.stream.depth,
                'width': 480,
                'height': 270,
                'format': rs.format.z16,
                'fps': 15
            },
            {
                'name': 'Low - Color 424x240 @ 15fps',
                'stream': rs.stream.color,
                'width': 424,
                'height': 240,
                'format': rs.format.bgr8,
                'fps': 15
            },
            {
                'name': 'Standard - Depth 640x480 @ 30fps',
                'stream': rs.stream.depth,
                'width': 640,
                'height': 480,
                'format': rs.format.z16,
                'fps': 30
            }
        ]
        
        results = []
        
        for config in test_configs:
            print(f"\nTesting: {config['name']}")
            result = {
                'config': config['name'],
                'status': 'failed',
                'frames_received': 0,
                'errors': []
            }
            
            pipeline = rs.pipeline()
            rs_config = rs.config()
            
            try:
                # Enable single stream with specific configuration
                rs_config.enable_stream(
                    config['stream'], 
                    config['width'], 
                    config['height'], 
                    config['format'], 
                    config['fps']
                )
                
                # Start with custom timeout
                pipeline_profile = pipeline.start(rs_config)
                
                print("  Stream started, attempting to capture frames...")
                
                # Try different timeout values
                timeout_tests = [1000, 5000, 10000]  # milliseconds
                
                for timeout in timeout_tests:
                    try:
                        print(f"  Testing with {timeout}ms timeout...")
                        frames = pipeline.wait_for_frames(timeout)
                        
                        if frames:
                            result['frames_received'] += 1
                            print(f"    ✓ Frame received with {timeout}ms timeout")
                            
                            # Try to get 10 frames
                            for i in range(9):
                                frames = pipeline.wait_for_frames(timeout)
                                if frames:
                                    result['frames_received'] += 1
                            
                            result['status'] = 'success'
                            result['working_timeout'] = timeout
                            break
                            
                    except Exception as e:
                        error_msg = f"Timeout {timeout}ms: {str(e)}"
                        result['errors'].append(error_msg)
                        print(f"    ❌ {error_msg}")
                
                print(f"  Total frames received: {result['frames_received']}")
                
            except Exception as e:
                error_msg = f"Pipeline start error: {str(e)}"
                result['errors'].append(error_msg)
                print(f"  ❌ {error_msg}")
                
            finally:
                try:
                    pipeline.stop()
                except:
                    pass
            
            results.append(result)
            time.sleep(2)  # Brief pause between tests
        
        self.results['tests']['minimal_stream'] = results
        return results
    
    def test_power_recovery(self):
        """Test camera recovery after power cycling"""
        print("\n" + "="*60)
        print("POWER RECOVERY TEST")
        print("="*60)
        
        results = {
            'recovery_attempts': [],
            'recommendations': []
        }
        
        print("Testing camera initialization and recovery...")
        
        for attempt in range(3):
            print(f"\nAttempt {attempt + 1}:")
            attempt_result = {
                'attempt': attempt + 1,
                'status': 'failed',
                'time_to_ready': None
            }
            
            try:
                # Reset USB if possible (requires sudo)
                if os.geteuid() == 0:
                    print("  Resetting USB hub...")
                    self.run_command("echo '1-1' > /sys/bus/usb/drivers/usb/unbind 2>/dev/null")
                    time.sleep(2)
                    self.run_command("echo '1-1' > /sys/bus/usb/drivers/usb/bind 2>/dev/null")
                    time.sleep(3)
                
                start_time = time.time()
                
                # Try to initialize
                ctx = rs.context()
                devices = ctx.query_devices()
                
                if len(devices) > 0:
                    init_time = time.time() - start_time
                    attempt_result['time_to_ready'] = init_time
                    print(f"  ✓ Device ready in {init_time:.2f} seconds")
                    
                    # Quick stream test
                    pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 15)
                    
                    pipeline.start(config)
                    frames = pipeline.wait_for_frames(10000)
                    pipeline.stop()
                    
                    if frames:
                        attempt_result['status'] = 'success'
                        print("  ✓ Stream test successful")
                    else:
                        print("  ⚠ Device found but stream failed")
                        
                else:
                    print("  ❌ No device found")
                    
            except Exception as e:
                print(f"  ❌ Error: {e}")
                attempt_result['error'] = str(e)
            
            results['recovery_attempts'].append(attempt_result)
            
            if attempt < 2:
                print("  Waiting 5 seconds before next attempt...")
                time.sleep(5)
        
        # Analyze results and provide recommendations
        successful_attempts = [a for a in results['recovery_attempts'] if a['status'] == 'success']
        
        if len(successful_attempts) == 0:
            results['recommendations'].extend([
                "CRITICAL: Camera fails to initialize reliably",
                "1. Check Y-splitter cable quality and connections",
                "2. Try a powered USB 3.0 hub instead of Y-splitter",
                "3. Ensure total power supply can provide at least 2A",
                "4. Test with shorter USB cable (under 1 meter)"
            ])
        elif len(successful_attempts) < 3:
            results['recommendations'].extend([
                "WARNING: Camera initialization is intermittent",
                "1. Power supply may be marginal - increase current capacity",
                "2. USB connection may be unstable - check all connections",
                "3. Consider adding capacitor across power lines for stability"
            ])
        else:
            results['recommendations'].append("Camera initialization is stable")
        
        self.results['tests']['power_recovery'] = results
        return results
    
    def check_system_resources(self):
        """Check Raspberry Pi system resources"""
        print("\n" + "="*60)
        print("SYSTEM RESOURCES CHECK")
        print("="*60)
        
        results = {}
        
        # CPU temperature
        temp_output = self.run_command("vcgencmd measure_temp")
        print(f"CPU Temperature: {temp_output}")
        results['cpu_temp'] = temp_output
        
        # Check for throttling
        throttle_output = self.run_command("vcgencmd get_throttled")
        print(f"Throttle Status: {throttle_output}")
        if "0x0" not in throttle_output:
            print("⚠ WARNING: System throttling detected!")
            print("  This indicates power or temperature issues")
        results['throttle_status'] = throttle_output
        
        # Memory usage
        mem_output = self.run_command("free -m | grep Mem")
        print(f"\nMemory Usage:\n{mem_output}")
        results['memory'] = mem_output
        
        # USB controller info
        print("\nUSB Controller:")
        usb_controller = self.run_command("lspci 2>/dev/null | grep USB || echo 'Using onboard USB'")
        print(usb_controller)
        
        self.results['tests']['system_resources'] = results
        return results
    
    def generate_report(self):
        """Generate diagnostic report with recommendations"""
        print("\n" + "="*60)
        print("DIAGNOSTIC SUMMARY")
        print("="*60)
        
        # Save detailed results
        with open('realsense_diagnostic_report.json', 'w') as f:
            json.dump(self.results, f, indent=2)
        
        print("\nKey Findings:")
        
        # Check USB power
        if 'usb_power' in self.results['tests']:
            usb_test = self.results['tests']['usb_power']
            if 'usb_speed' in usb_test:
                if "USB 2.0" in usb_test['usb_speed']:
                    print("❌ CRITICAL: Running on USB 2.0 - insufficient bandwidth")
                else:
                    print("✓ USB 3.0 connection confirmed")
        
        # Check connection
        if 'connection' in self.results['tests']:
            conn_test = self.results['tests']['connection']
            if conn_test['status'] == 'connected':
                print("✓ Camera detected by RealSense SDK")
            else:
                print("❌ Camera not properly detected")
        
        # Check streaming
        if 'minimal_stream' in self.results['tests']:
            stream_results = self.results['tests']['minimal_stream']
            working_configs = [r for r in stream_results if r['status'] == 'success']
            if working_configs:
                print(f"✓ {len(working_configs)} configurations work")
                print(f"  Recommended: {working_configs[0]['config']}")
            else:
                print("❌ No configurations work reliably")
        
        print("\n" + "-"*60)
        print("RECOMMENDATIONS FOR YOUR SETUP:")
        print("-"*60)
        
        print("\n1. IMMEDIATE ACTIONS:")
        print("   - Verify Y-splitter is connected properly (data to Pi, power to adapter)")
        print("   - Ensure power adapter provides at least 2A @ 5V")
        print("   - Try connecting directly to Pi USB 3.0 port without splitter")
        
        print("\n2. POWER SOLUTIONS:")
        print("   - Best: Use powered USB 3.0 hub (recommended: 3A+ per port)")
        print("   - Alternative: Official RealSense power cable")
        print("   - Check: Measure voltage at camera (should be 4.75V-5.25V)")
        
        print("\n3. SOFTWARE OPTIMIZATIONS:")
        print("   - Start with lower resolution: 480x270 @ 15fps")
        print("   - Increase pipeline timeout: wait_for_frames(10000)")
        print("   - Disable auto-exposure initially")
        print("   - Use single stream before attempting multiple")
        
        print("\n4. TROUBLESHOOTING STEPS:")
        print("   a. Test with external powered hub")
        print("   b. Monitor dmesg during connection: dmesg -w")
        print("   c. Try different USB ports")
        print("   d. Update camera firmware if needed")
        
        print("\nDetailed report saved to: realsense_diagnostic_report.json")

def main():
    print("Intel RealSense D435 Diagnostic Tool")
    print("For Raspberry Pi with Y-Splitter Power Setup")
    print("="*60)
    
    if os.geteuid() != 0:
        print("Note: Running as user. Some tests may be limited.")
        print("For full diagnostics, run with: sudo python3 diagnostics.py")
    
    diag = RealSenseDiagnostics()
    
    # Run all diagnostics
    diag.check_system_resources()
    diag.check_usb_power()
    diag.check_realsense_connection()
    diag.test_minimal_stream()
    diag.test_power_recovery()
    diag.generate_report()

if __name__ == "__main__":
    main()