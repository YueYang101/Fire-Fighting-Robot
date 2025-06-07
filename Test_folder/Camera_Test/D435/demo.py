#!/usr/bin/env python3
"""
Intel RealSense D435 Depth Camera Demo - Terminal Version
Tests depth camera functionality on Raspberry Pi 4 (No GUI required)
"""

import numpy as np
import time
import sys
import os

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("Error: pyrealsense2 not installed!")
    print("Install with: pip3 install pyrealsense2")
    sys.exit(1)


class D435TerminalDemo:
    def __init__(self):
        """Initialize the RealSense D435 camera"""
        self.pipeline = None
        self.config = None
        self.depth_scale = None
        
    def initialize_camera(self):
        """Configure and start the camera pipeline"""
        try:
            # Create a pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Get device info
            context = rs.context()
            devices = context.query_devices()
            
            if len(devices) == 0:
                print("No RealSense devices detected!")
                print("Check USB connection (use USB 3.0 port)")
                return False
            
            # Print device info
            print("\n" + "="*60)
            print("DEVICE INFORMATION")
            print("="*60)
            device = None
            for dev in devices:
                print(f"Device: {dev.get_info(rs.camera_info.name)}")
                print(f"Serial: {dev.get_info(rs.camera_info.serial_number)}")
                print(f"Firmware: {dev.get_info(rs.camera_info.firmware_version)}")
                device = dev
                
                # Check USB type
                if dev.supports(rs.camera_info.usb_type_descriptor):
                    usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
                    print(f"USB Type: {usb_type}")
                    if "2." in usb_type:
                        print("WARNING: Camera connected to USB 2.0 - Performance will be limited!")
            print("="*60 + "\n")
            
            # Try different configurations based on USB connection
            configs_to_try = [
                # Optimized for Raspberry Pi - start with lower settings
                # USB 2.0 friendly configs
                (424, 240, 6),   # Lowest stable config
                (424, 240, 15),
                (480, 270, 6),
                (480, 270, 15),
                (640, 360, 6),
                # USB 3.0 configs (if available)
                (640, 480, 6),
                (640, 480, 15),
                (640, 480, 30),
                (848, 480, 6),
                (848, 480, 15),
            ]
            
            # Try each configuration
            for width, height, fps in configs_to_try:
                try:
                    print(f"Trying depth stream: {width}x{height} @ {fps}fps...", end='')
                    
                    # Reset config
                    self.config = rs.config()
                    
                    # Enable depth stream only
                    self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
                    
                    # Try to start
                    profile = self.pipeline.start(self.config)
                    
                    print(" SUCCESS!")
                    print(f"Running at {width}x{height} @ {fps}fps")
                    
                    # Get depth sensor and scale
                    depth_sensor = profile.get_device().first_depth_sensor()
                    self.depth_scale = depth_sensor.get_depth_scale()
                    print(f"Depth Scale: {self.depth_scale}")
                    
                    # For Raspberry Pi - set some optimizations
                    try:
                        # Disable auto-exposure for more consistent performance
                        depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
                        # Set a fixed exposure value
                        depth_sensor.set_option(rs.option.exposure, 8500)
                    except:
                        pass  # Some options might not be available
                    
                    # Store resolution for later use
                    self.width = width
                    self.height = height
                    
                    print("Camera initialized successfully!")
                    return True
                    
                except Exception as e:
                    print(f" Failed: {str(e)}")
                    if self.pipeline:
                        try:
                            self.pipeline.stop()
                        except:
                            pass
                    continue
            
            print("\nFailed to initialize with any configuration!")
            print("Possible issues:")
            print("1. USB bandwidth limitations - try a different USB port")
            print("2. Power issues - camera may need more power than Pi can provide")
            print("3. Try using a powered USB hub")
            return False
            
        except Exception as e:
            print(f"Failed to initialize camera: {e}")
            return False
    
    def get_depth_frame(self):
        """Get depth frame data"""
        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                return None
            
            # Convert to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            return depth_image
            
        except Exception as e:
            print(f"Error getting frame: {e}")
            return None
    
    def depth_to_ascii(self, depth_array, width=80, height=24):
        """Convert depth data to ASCII visualization"""
        # ASCII characters for different depth levels
        ascii_chars = " .:-=+*#%@"
        
        # Resize depth array to fit terminal
        h, w = depth_array.shape
        scale_w = w // width
        scale_h = h // height
        
        # Downsample
        small_depth = depth_array[::scale_h, ::scale_w]
        
        # Convert depth to meters
        depth_meters = small_depth * self.depth_scale
        
        # Normalize depth values (0-5 meters range)
        max_depth = 5.0
        normalized = np.clip(depth_meters / max_depth, 0, 1)
        
        # Convert to ASCII
        ascii_indices = (normalized * (len(ascii_chars) - 1)).astype(int)
        
        # Build ASCII image
        ascii_image = []
        for row in ascii_indices:
            ascii_row = ''.join(ascii_chars[idx] for idx in row[:width])
            ascii_image.append(ascii_row)
        
        return ascii_image[:height], depth_meters
    
    def print_depth_stats(self, depth_meters):
        """Print depth statistics"""
        valid_depths = depth_meters[depth_meters > 0]
        
        if len(valid_depths) > 0:
            stats = {
                'Min': np.min(valid_depths),
                'Max': np.max(valid_depths),
                'Mean': np.mean(valid_depths),
                'Center': depth_meters[depth_meters.shape[0]//2, depth_meters.shape[1]//2]
            }
        else:
            stats = {'Min': 0, 'Max': 0, 'Mean': 0, 'Center': 0}
        
        return stats
    
    def save_depth_data(self, depth_image):
        """Save depth data to file"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f'depth_data_{timestamp}.npy'
        np.save(filename, depth_image)
        
        # Also save as CSV for easy viewing
        csv_filename = f'depth_data_{timestamp}.csv'
        depth_meters = depth_image * self.depth_scale
        np.savetxt(csv_filename, depth_meters, delimiter=',', fmt='%.3f')
        
        return filename, csv_filename
    
    def run_terminal_demo(self):
        """Main demo loop for terminal"""
        if not self.initialize_camera():
            return
        
        print("\nStarting depth camera stream...")
        print("Commands: 'q' to quit, 's' to save depth data")
        print("Depth visualization (near to far): " + " .:-=+*#%@")
        print("-" * 80)
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                # Clear screen (works on most terminals)
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Get depth frame
                depth_image = self.get_depth_frame()
                
                if depth_image is None:
                    continue
                
                # Convert to ASCII and get stats
                ascii_image, depth_meters = self.depth_to_ascii(depth_image)
                stats = self.print_depth_stats(depth_meters)
                
                # Calculate FPS
                frame_count += 1
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                
                # Print header
                print(f"RealSense D435 - Terminal View | FPS: {fps:.1f}")
                print("="*80)
                
                # Print ASCII visualization
                for line in ascii_image:
                    print(line)
                
                # Print statistics
                print("="*80)
                print(f"Depth Stats (meters): Min: {stats['Min']:.2f} | "
                      f"Max: {stats['Max']:.2f} | "
                      f"Mean: {stats['Mean']:.2f} | "
                      f"Center: {stats['Center']:.2f}")
                print("Commands: 'q' + Enter to quit, 's' + Enter to save")
                
                # Non-blocking input check
                import select
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    key = sys.stdin.readline().strip()
                    
                    if key == 'q':
                        break
                    elif key == 's':
                        npy_file, csv_file = self.save_depth_data(depth_image)
                        print(f"\nSaved: {npy_file} and {csv_file}")
                        time.sleep(2)
                
                time.sleep(0.033)  # ~30 FPS
                
        except KeyboardInterrupt:
            print("\n\nStopping...")
        
        finally:
            self.cleanup()
    
    def run_matrix_demo(self):
        """Alternative demo showing depth as numeric matrix"""
        if not self.initialize_camera():
            return
        
        print("\nRunning matrix view demo...")
        print("Showing 10x10 center region depth values in meters")
        print("Press Ctrl+C to stop")
        print("-" * 80)
        
        try:
            while True:
                depth_image = self.get_depth_frame()
                
                if depth_image is None:
                    continue
                
                # Get center region
                h, w = depth_image.shape
                center_y, center_x = h//2, w//2
                
                # Extract 10x10 region
                region = depth_image[center_y-5:center_y+5, center_x-5:center_x+5]
                depth_meters = region * self.depth_scale
                
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                print("Depth Matrix (meters) - 10x10 center region:")
                print("="*80)
                
                # Print matrix with formatting
                for row in depth_meters:
                    row_str = " ".join(f"{val:5.2f}" for val in row)
                    print(row_str)
                
                print("="*80)
                print(f"Center depth: {depth_meters[5,5]:.2f} meters")
                
                time.sleep(0.5)  # Update every 0.5 seconds
                
        except KeyboardInterrupt:
            print("\nStopping...")
        
        finally:
            self.cleanup()
    
    def run_performance_test(self):
        """Minimal processing performance test"""
        if not self.initialize_camera():
            return
        
        print("\nRunning performance test...")
        print("This mode does minimal processing to test maximum FPS")
        print("Press Ctrl+C to stop\n")
        
        frame_count = 0
        start_time = time.time()
        last_print = start_time
        
        try:
            while True:
                # Just get frame and basic stats
                depth_image = self.get_depth_frame()
                
                if depth_image is None:
                    continue
                
                frame_count += 1
                current_time = time.time()
                
                # Print stats every second
                if current_time - last_print >= 1.0:
                    elapsed = current_time - start_time
                    fps = frame_count / elapsed
                    
                    # Get simple center depth
                    h, w = depth_image.shape
                    center_depth = depth_image[h//2, w//2] * self.depth_scale
                    
                    print(f"\rFPS: {fps:.1f} | Frames: {frame_count} | Center depth: {center_depth:.2f}m", end='', flush=True)
                    last_print = current_time
                
        except KeyboardInterrupt:
            print("\n\nPerformance test complete!")
            elapsed = time.time() - start_time
            avg_fps = frame_count / elapsed
            print(f"Average FPS: {avg_fps:.1f}")
            print(f"Total frames: {frame_count}")
            print(f"Duration: {elapsed:.1f} seconds")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.pipeline:
            self.pipeline.stop()
        print("Camera stopped.")


def check_usb_connection():
    """Check USB connection details"""
    print("\nChecking USB connections...")
    
    try:
        # Check lsusb for RealSense device
        import subprocess
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        
        for line in result.stdout.split('\n'):
            if '8086:0b3a' in line or 'Intel' in line and 'RealSense' in line:
                print(f"Found: {line}")
                
                # Get bus and device number
                parts = line.split()
                if len(parts) >= 4:
                    bus = parts[1]
                    device = parts[3].rstrip(':')
                    
                    # Check USB speed
                    speed_cmd = f"lsusb -t | grep -B5 'Intel' | grep -E 'Bus|{device}'"
                    speed_result = subprocess.run(speed_cmd, shell=True, capture_output=True, text=True)
                    if speed_result.stdout:
                        print(f"USB Tree Info:\n{speed_result.stdout}")
    except Exception as e:
        print(f"Could not check USB details: {e}")
    
    print("\nTo check USB 3.0 connection manually:")
    print("1. Run: lsusb -t")
    print("2. Look for '5000M' (USB 3.0) or '480M' (USB 2.0) next to the Intel device")
    print("3. For best performance, ensure it shows '5000M'\n")


def main():
    print("="*60)
    print("Intel RealSense D435 Depth Camera Demo - Terminal Version")
    print("="*60)
    
    if not REALSENSE_AVAILABLE:
        return
    
    # Check USB connection first
    check_usb_connection()
    
    # Create demo instance
    demo = D435TerminalDemo()
    
    # Ask user which demo to run
    print("\nSelect demo mode:")
    print("1. ASCII visualization (full frame)")
    print("2. Numeric matrix (center region)")
    print("3. Performance test (minimal processing)")
    
    choice = input("\nEnter choice (1, 2, or 3): ").strip()
    
    if choice == '1':
        # For ASCII mode, we need to set terminal to non-canonical mode
        import termios, tty
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            demo.run_terminal_demo()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    elif choice == '2':
        demo.run_matrix_demo()
    elif choice == '3':
        demo.run_performance_test()
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()