#!/usr/bin/env python3
"""
Intel RealSense D435 Depth Camera Demo
Tests depth camera functionality on Raspberry Pi 4
"""

import numpy as np
import cv2
import time
import sys

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("Error: pyrealsense2 not installed!")
    print("Install with: pip3 install pyrealsense2")
    sys.exit(1)


class D435Demo:
    def __init__(self):
        """Initialize the RealSense D435 camera"""
        self.pipeline = None
        self.config = None
        self.align = None
        
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
                return False
            
            # Print device info
            for dev in devices:
                print(f"Found device: {dev.get_info(rs.camera_info.name)}")
                print(f"Serial number: {dev.get_info(rs.camera_info.serial_number)}")
                print(f"Firmware version: {dev.get_info(rs.camera_info.firmware_version)}")
                print("-" * 50)
            
            # Configure streams
            # Depth stream - 640x480 @ 30 FPS
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            # Color stream - 640x480 @ 30 FPS
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # Start streaming
            profile = self.pipeline.start(self.config)
            
            # Get depth sensor
            depth_sensor = profile.get_device().first_depth_sensor()
            
            # Get depth scale
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"Depth Scale: {self.depth_scale}")
            
            # Create align object to align depth to color
            self.align = rs.align(rs.stream.color)
            
            print("Camera initialized successfully!")
            return True
            
        except Exception as e:
            print(f"Failed to initialize camera: {e}")
            return False
    
    def get_frames(self):
        """Get aligned color and depth frames"""
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            
            # Align depth frame to color frame
            aligned_frames = self.align.process(frames)
            
            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return None, None
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
            
        except Exception as e:
            print(f"Error getting frames: {e}")
            return None, None
    
    def process_depth_data(self, depth_image, x, y):
        """Get depth value at specific pixel coordinates"""
        if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
            depth_value = depth_image[y, x] * self.depth_scale  # Convert to meters
            return depth_value
        return 0
    
    def run_demo(self):
        """Main demo loop"""
        if not self.initialize_camera():
            return
        
        print("\nStarting camera feed...")
        print("Press 'q' to quit")
        print("Press 's' to save current frames")
        print("Click on the image to get depth at that point")
        
        # Mouse callback variables
        mouse_x, mouse_y = 320, 240
        
        def mouse_callback(event, x, y, flags, param):
            nonlocal mouse_x, mouse_y
            if event == cv2.EVENT_LBUTTONDOWN:
                mouse_x, mouse_y = x, y
        
        # Create windows
        cv2.namedWindow('Color', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Color', mouse_callback)
        
        # Frame counter for FPS calculation
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                # Get frames
                color_image, depth_image = self.get_frames()
                
                if color_image is None or depth_image is None:
                    continue
                
                # Apply colormap to depth image for visualization
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                
                # Get depth at mouse position
                depth_value = self.process_depth_data(depth_image, mouse_x, mouse_y)
                
                # Draw crosshair and depth info on color image
                display_image = color_image.copy()
                cv2.drawMarker(display_image, (mouse_x, mouse_y), 
                              (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                
                # Add text info
                cv2.putText(display_image, f"Depth: {depth_value:.2f}m", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_image, f"Position: ({mouse_x}, {mouse_y})", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Calculate and display FPS
                frame_count += 1
                if frame_count % 30 == 0:
                    end_time = time.time()
                    fps = 30 / (end_time - start_time)
                    print(f"FPS: {fps:.2f}")
                    start_time = time.time()
                
                cv2.putText(display_image, f"FPS: {fps:.1f}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Display images
                cv2.imshow('Color', display_image)
                cv2.imshow('Depth', depth_colormap)
                
                # Handle key press
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    # Save frames
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f'color_{timestamp}.png', color_image)
                    cv2.imwrite(f'depth_{timestamp}.png', depth_colormap)
                    np.save(f'depth_data_{timestamp}.npy', depth_image)
                    print(f"Saved frames with timestamp: {timestamp}")
                
        except KeyboardInterrupt:
            print("\nStopping...")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()
        print("Camera stopped and windows closed.")


def check_usb_bandwidth():
    """Check if camera is connected to USB 3.0"""
    print("\nChecking USB connection...")
    print("For best performance, ensure D435 is connected to a USB 3.0 port (blue port)")
    print("USB 2.0 will work but with limited resolution and framerate\n")


def main():
    print("="*60)
    print("Intel RealSense D435 Depth Camera Demo")
    print("="*60)
    
    if not REALSENSE_AVAILABLE:
        return
    
    check_usb_bandwidth()
    
    # Create and run demo
    demo = D435Demo()
    demo.run_demo()


if __name__ == "__main__":
    main()