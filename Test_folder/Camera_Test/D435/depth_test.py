#!/usr/bin/env python3
"""
Intel RealSense D435 Depth-Only Terminal Demo
Displays depth readings in the terminal with a simple ASCII visualization
"""

import pyrealsense2 as rs
import numpy as np
import time
import os
import sys

def clear_terminal():
    """Clear terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')

def depth_to_ascii(depth_value, min_depth=0.3, max_depth=3.0):
    """Convert depth value to ASCII character for visualization"""
    if depth_value == 0:
        return ' '  # No reading
    
    # Normalize depth to 0-1 range
    normalized = (depth_value - min_depth) / (max_depth - min_depth)
    normalized = max(0, min(1, normalized))  # Clamp to 0-1
    
    # ASCII characters from near to far
    ascii_chars = '█▓▒░·. '
    index = int(normalized * (len(ascii_chars) - 1))
    return ascii_chars[index]

def print_depth_grid(depth_frame, depth_scale, grid_size=20):
    """Print depth values in a grid pattern"""
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    
    # Calculate step sizes for grid
    x_step = width // grid_size
    y_step = height // grid_size
    
    # Get depth data as numpy array
    depth_image = np.asanyarray(depth_frame.get_data())
    
    print("\nDepth Grid (meters):")
    print("=" * (grid_size * 4 + 1))
    
    for y in range(0, height, y_step):
        row = []
        for x in range(0, width, x_step):
            # Get depth value in meters
            depth_m = depth_image[y, x] * depth_scale
            
            if depth_m > 0:
                # Format depth value
                row.append(f"{depth_m:3.1f}")
            else:
                row.append(" -- ")
        
        print("|" + "|".join(row) + "|")
    
    print("=" * (grid_size * 4 + 1))

def print_ascii_visualization(depth_frame, depth_scale, width=80, height=24):
    """Print ASCII art visualization of depth"""
    frame_width = depth_frame.get_width()
    frame_height = depth_frame.get_height()
    
    # Calculate sampling steps
    x_step = frame_width // width
    y_step = frame_height // height
    
    # Get depth data
    depth_image = np.asanyarray(depth_frame.get_data())
    
    print("\nDepth Visualization (█=near, ·=far, space=no reading):")
    print("┌" + "─" * width + "┐")
    
    for y in range(0, frame_height, y_step):
        if y // y_step >= height:
            break
        
        row = "│"
        for x in range(0, frame_width, x_step):
            if x // x_step >= width:
                break
            
            # Get depth in meters
            depth_m = depth_image[y, x] * depth_scale
            char = depth_to_ascii(depth_m)
            row += char
        
        row += "│"
        print(row)
    
    print("└" + "─" * width + "┘")

def get_center_region_stats(depth_frame, depth_scale, region_size=50):
    """Get statistics for center region of frame"""
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    
    # Define center region
    center_x = width // 2
    center_y = height // 2
    x1 = max(0, center_x - region_size // 2)
    x2 = min(width, center_x + region_size // 2)
    y1 = max(0, center_y - region_size // 2)
    y2 = min(height, center_y + region_size // 2)
    
    # Get depth data
    depth_image = np.asanyarray(depth_frame.get_data())
    center_region = depth_image[y1:y2, x1:x2] * depth_scale
    
    # Filter out zero values
    valid_depths = center_region[center_region > 0]
    
    if len(valid_depths) > 0:
        return {
            'center': depth_frame.get_distance(center_x, center_y),
            'mean': np.mean(valid_depths),
            'min': np.min(valid_depths),
            'max': np.max(valid_depths),
            'std': np.std(valid_depths),
            'valid_pixels': len(valid_depths),
            'total_pixels': center_region.size
        }
    else:
        return None

def main():
    """Main demo function"""
    print("Intel RealSense D435 Depth-Only Terminal Demo")
    print("=" * 50)
    
    # Create pipeline and config
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure depth stream
    # Use 640x480 for good balance of resolution and performance
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        # Start streaming
        print("Starting depth stream...")
        profile = pipeline.start(config)
        
        # Get depth scale
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth scale: {depth_scale} meters/unit")
        
        # Allow auto-exposure to settle
        print("Allowing sensor to stabilize...")
        for _ in range(30):
            pipeline.wait_for_frames()
        
        print("\nPress Ctrl+C to exit")
        print("\nDisplay modes:")
        print("1. Simple center point reading")
        print("2. Depth grid values")
        print("3. ASCII visualization")
        print("4. All modes")
        
        mode = input("\nSelect display mode (1-4): ").strip()
        if mode not in ['1', '2', '3', '4']:
            mode = '1'  # Default to simple mode
        
        frame_count = 0
        fps_start = time.time()
        
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            # Clear terminal for clean display
            clear_terminal()
            
            # Display header
            print(f"RealSense D435 Depth Demo - Frame {frame_count}")
            print("=" * 50)
            
            # Mode 1 or 4: Simple center reading
            if mode in ['1', '4']:
                width = depth_frame.get_width()
                height = depth_frame.get_height()
                center_distance = depth_frame.get_distance(width // 2, height // 2)
                
                print(f"\nCenter Point Distance: {center_distance:.3f} meters")
                
                # Get center region statistics
                stats = get_center_region_stats(depth_frame, depth_scale)
                if stats:
                    print(f"\nCenter Region Statistics (50x50 pixels):")
                    print(f"  Mean depth:  {stats['mean']:.3f} m")
                    print(f"  Min depth:   {stats['min']:.3f} m")
                    print(f"  Max depth:   {stats['max']:.3f} m")
                    print(f"  Std dev:     {stats['std']:.3f} m")
                    print(f"  Valid pixels: {stats['valid_pixels']}/{stats['total_pixels']} "
                          f"({stats['valid_pixels']/stats['total_pixels']*100:.1f}%)")
            
            # Mode 2 or 4: Depth grid
            if mode in ['2', '4']:
                print_depth_grid(depth_frame, depth_scale, grid_size=15)
            
            # Mode 3 or 4: ASCII visualization
            if mode in ['3', '4']:
                print_ascii_visualization(depth_frame, depth_scale, width=60, height=20)
            
            # Calculate and display FPS
            frame_count += 1
            if frame_count % 30 == 0:
                fps = 30 / (time.time() - fps_start)
                fps_start = time.time()
                print(f"\nFPS: {fps:.1f}")
            
            # Small delay to make display readable
            if mode in ['1', '2', '3']:
                time.sleep(0.1)  # 10 FPS for single modes
            else:
                time.sleep(0.2)  # 5 FPS for all modes
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\nERROR: {str(e)}")
        print("\nTroubleshooting:")
        print("1. Check if camera is connected: lsusb | grep Intel")
        print("2. Check permissions: groups (should include 'video')")
        print("3. Try running with sudo (not recommended for production)")
        print("4. Check if another process is using the camera")
    finally:
        pipeline.stop()
        print("Depth stream stopped")
        print(f"Total frames captured: {frame_count}")

if __name__ == "__main__":
    main()