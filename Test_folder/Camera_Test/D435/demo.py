#!/usr/bin/env python3
"""
Solutions for "Frame didn't arrive within 5000" error on RealSense D435
Includes USB reset methods and robust connection handling
"""

import pyrealsense2 as rs
import numpy as np
import time
import subprocess
import os
import sys

# ==============================================================================
# Solution 1: USB Reset Script
# ==============================================================================

def reset_usb_device():
    """Reset USB device using system commands"""
    print("\nAttempting USB reset...")
    
    try:
        # Method 1: Using usbreset command
        # First, find the device
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'Intel Corp.' in line and 'RealSense' in line:
                # Extract bus and device numbers
                parts = line.split()
                bus = parts[1]
                device = parts[3].rstrip(':')
                
                print(f"Found RealSense on Bus {bus} Device {device}")
                
                # Try usbreset if available
                usb_path = f"/dev/bus/usb/{bus}/{device}"
                if os.path.exists(usb_path):
                    try:
                        subprocess.run(['sudo', 'usbreset', usb_path], check=True)
                        print("USB reset completed using usbreset")
                        time.sleep(2)  # Wait for device to reinitialize
                        return True
                    except:
                        print("usbreset not available, trying alternative method...")
        
        # Method 2: Using sysfs to reset USB
        # Find the device in sysfs
        result = subprocess.run(['find', '/sys/bus/usb/devices/', '-name', 'idVendor'], 
                              capture_output=True, text=True)
        
        for vendor_path in result.stdout.strip().split('\n'):
            if vendor_path:
                with open(vendor_path, 'r') as f:
                    if f.read().strip() == '8086':  # Intel vendor ID
                        device_path = os.path.dirname(vendor_path)
                        product_path = os.path.join(device_path, 'idProduct')
                        
                        if os.path.exists(product_path):
                            with open(product_path, 'r') as f:
                                product = f.read().strip()
                                # RealSense D435 product IDs
                                if product in ['0b07', '0b3a']:
                                    # Found RealSense device
                                    print(f"Found RealSense in sysfs: {device_path}")
                                    
                                    # Reset by unbinding and rebinding
                                    driver_path = os.path.join(device_path, 'driver')
                                    if os.path.exists(driver_path):
                                        device_name = os.path.basename(device_path)
                                        unbind_path = os.path.join(driver_path, 'unbind')
                                        bind_path = os.path.join(driver_path, 'bind')
                                        
                                        try:
                                            # Unbind
                                            subprocess.run(['sudo', 'sh', '-c', 
                                                          f'echo {device_name} > {unbind_path}'], 
                                                         check=True)
                                            time.sleep(1)
                                            
                                            # Rebind
                                            subprocess.run(['sudo', 'sh', '-c', 
                                                          f'echo {device_name} > {bind_path}'], 
                                                         check=True)
                                            print("USB reset completed using sysfs")
                                            time.sleep(2)
                                            return True
                                        except Exception as e:
                                            print(f"Sysfs reset failed: {e}")
        
        # Method 3: Power cycle USB port (if hub supports it)
        print("Attempting USB power cycle...")
        try:
            # This requires uhubctl to be installed
            subprocess.run(['sudo', 'uhubctl', '-a', 'cycle', '-d', '2'], check=True)
            print("USB power cycle completed")
            time.sleep(3)
            return True
        except:
            print("uhubctl not available")
            
    except Exception as e:
        print(f"USB reset failed: {e}")
    
    return False


# ==============================================================================
# Solution 2: Robust Pipeline with Automatic Recovery
# ==============================================================================

class RobustRealSenseCamera:
    """Robust camera handler with automatic error recovery"""
    
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.device = None
        self.running = False
        self.frame_timeout = 5000  # ms
        self.retry_count = 3
        self.last_reset_time = 0
        self.reset_cooldown = 10  # seconds between resets
        
    def find_device(self):
        """Find and return RealSense device"""
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            return None
            
        return devices[0]
    
    def initialize_pipeline(self, width=640, height=480, fps=30):
        """Initialize pipeline with specified configuration"""
        try:
            # Find device
            self.device = self.find_device()
            if not self.device:
                print("No RealSense device found")
                return False
            
            # Create pipeline and config
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Get device serial for specific targeting
            serial = self.device.get_info(rs.camera_info.serial_number)
            self.config.enable_device(serial)
            
            # Configure streams with lower settings for USB 2.0
            # Try multiple configurations in order of preference
            configurations = [
                # (width, height, fps)
                (width, height, fps),
                (640, 480, 15),
                (480, 270, 30),
                (480, 270, 15),
                (424, 240, 30),
                (424, 240, 15)
            ]
            
            for w, h, f in configurations:
                try:
                    print(f"Trying configuration: {w}x{h} @ {f}fps")
                    
                    # Clear previous config
                    self.config = rs.config()
                    self.config.enable_device(serial)
                    
                    # Enable streams
                    self.config.enable_stream(rs.stream.depth, w, h, rs.format.z16, f)
                    self.config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
                    
                    # Try to start
                    profile = self.pipeline.start(self.config)
                    
                    # Verify we can get frames
                    for _ in range(5):
                        frames = self.pipeline.wait_for_frames(timeout_ms=self.frame_timeout)
                        if frames:
                            print(f"Successfully initialized at {w}x{h} @ {f}fps")
                            self.running = True
                            return True
                            
                except Exception as e:
                    print(f"Configuration {w}x{h} @ {f}fps failed: {str(e)}")
                    if self.pipeline:
                        try:
                            self.pipeline.stop()
                        except:
                            pass
                    continue
            
            print("All configurations failed")
            return False
            
        except Exception as e:
            print(f"Pipeline initialization error: {e}")
            return False
    
    def get_frames_safe(self):
        """Get frames with error handling and recovery"""
        if not self.running:
            return None, None
            
        try:
            # Use try_wait_for_frames with timeout
            frames = self.pipeline.try_wait_for_frames(timeout_ms=self.frame_timeout)
            
            if not frames:
                print("No frames received within timeout")
                return None, None
                
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return None, None
                
            return depth_frame, color_frame
            
        except Exception as e:
            print(f"Error getting frames: {e}")
            return None, None
    
    def recover_connection(self):
        """Attempt to recover from connection loss"""
        print("\nAttempting to recover connection...")
        
        # Check cooldown
        current_time = time.time()
        if current_time - self.last_reset_time < self.reset_cooldown:
            print("Too soon to reset, waiting...")
            time.sleep(self.reset_cooldown - (current_time - self.last_reset_time))
        
        self.last_reset_time = time.time()
        
        # Stop current pipeline
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        
        self.running = False
        self.pipeline = None
        
        # Try USB reset
        if reset_usb_device():
            time.sleep(3)  # Wait for device to fully initialize
        
        # Try to reinitialize
        for attempt in range(self.retry_count):
            print(f"Recovery attempt {attempt + 1}/{self.retry_count}")
            
            if self.initialize_pipeline():
                print("Recovery successful!")
                return True
                
            time.sleep(2)
        
        print("Recovery failed")
        return False
    
    def run_with_recovery(self, callback, duration=None):
        """Run camera with automatic recovery on errors"""
        if not self.initialize_pipeline():
            print("Failed to initialize camera")
            return
        
        start_time = time.time()
        frame_count = 0
        error_count = 0
        consecutive_errors = 0
        
        print("\nCamera running. Press Ctrl+C to stop.")
        
        try:
            while True:
                # Check duration
                if duration and (time.time() - start_time) > duration:
                    break
                
                # Get frames
                depth_frame, color_frame = self.get_frames_safe()
                
                if depth_frame and color_frame:
                    # Reset error counter on success
                    consecutive_errors = 0
                    
                    # Convert to numpy arrays
                    depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())
                    
                    # Call user callback
                    if callback:
                        callback(depth_image, color_image, frame_count)
                    
                    frame_count += 1
                    
                else:
                    # Frame error
                    consecutive_errors += 1
                    error_count += 1
                    
                    if consecutive_errors > 10:
                        print(f"\nToo many consecutive errors ({consecutive_errors})")
                        
                        if not self.recover_connection():
                            print("Unable to recover, exiting")
                            break
                        
                        consecutive_errors = 0
                
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            if self.pipeline:
                self.pipeline.stop()
            
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            print(f"\nStatistics:")
            print(f"  Total frames: {frame_count}")
            print(f"  Total errors: {error_count}")
            print(f"  Duration: {elapsed:.1f}s")
            print(f"  Average FPS: {fps:.2f}")


# ==============================================================================
# Solution 3: Simple Test with Lower Requirements
# ==============================================================================

def test_minimal_configuration():
    """Test with minimal configuration for USB 2.0"""
    print("\nTesting minimal configuration for USB 2.0...")
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Use very low resolution and framerate
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 6)
    
    try:
        pipeline.start(config)
        print("Started successfully at 424x240 @ 6fps")
        
        for i in range(30):
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if depth_frame:
                print(f"Frame {i}: OK")
            else:
                print(f"Frame {i}: FAILED")
            
            time.sleep(0.1)
        
        print("Minimal configuration test passed!")
        
    except Exception as e:
        print(f"Minimal configuration failed: {e}")
    finally:
        pipeline.stop()


# ==============================================================================
# Example Usage
# ==============================================================================

def example_callback(depth_image, color_image, frame_number):
    """Example callback for processing frames"""
    if frame_number % 30 == 0:  # Every 30 frames
        print(f"Frame {frame_number}: Depth shape={depth_image.shape}, "
              f"Color shape={color_image.shape}")
        
        # Save snapshot
        np.save(f'depth_frame_{frame_number}.npy', depth_image)
        np.save(f'color_frame_{frame_number}.npy', color_image)


# ==============================================================================
# Main Menu
# ==============================================================================

if __name__ == "__main__":
    print("RealSense D435 USB Reset and Recovery Solutions")
    print("=" * 50)
    
    while True:
        print("\nOptions:")
        print("1. Test USB Reset")
        print("2. Test Minimal Configuration (424x240 @ 6fps)")
        print("3. Run Robust Camera with Auto-Recovery")
        print("4. Install USB reset tools")
        print("0. Exit")
        
        choice = input("\nSelect option (0-4): ")
        
        if choice == '1':
            reset_usb_device()
            
        elif choice == '2':
            test_minimal_configuration()
            
        elif choice == '3':
            camera = RobustRealSenseCamera()
            camera.run_with_recovery(example_callback, duration=60)
            
        elif choice == '4':
            print("\nTo install USB reset tools:")
            print("  sudo apt-get update")
            print("  sudo apt-get install usbutils")
            print("  ")
            print("  # For usbreset:")
            print("  sudo apt-get install usb-reset")
            print("  ")
            print("  # For uhubctl (USB hub control):")
            print("  sudo apt-get install libusb-1.0-0-dev")
            print("  git clone https://github.com/mvp/uhubctl")
            print("  cd uhubctl")
            print("  make")
            print("  sudo make install")
            
        elif choice == '0':
            break
        else:
            print("Invalid option")