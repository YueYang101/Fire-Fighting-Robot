#!/usr/bin/env python3
"""
Intel RealSense D435 Headless Test Scripts for Raspberry Pi (Ubuntu Server)
Test different features without GUI: Depth, RGB, and Infrared
"""

import pyrealsense2 as rs
import numpy as np
import time
import json
import os
from datetime import datetime

# ==============================================================================
# Test 1: Basic Connection and Device Info
# ==============================================================================

def test_connection():
    """Test basic connection and display device information"""
    print("\n" + "="*60)
    print("TEST 1: Device Connection and Information")
    print("="*60)
    
    try:
        # Create context
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("ERROR: No RealSense devices found!")
            return False
            
        print(f"Found {len(devices)} RealSense device(s)")
        
        for i, dev in enumerate(devices):
            print(f"\nDevice {i}:")
            print(f"  Name: {dev.get_info(rs.camera_info.name)}")
            print(f"  Serial Number: {dev.get_info(rs.camera_info.serial_number)}")
            print(f"  Firmware Version: {dev.get_info(rs.camera_info.firmware_version)}")
            print(f"  USB Type: {dev.get_info(rs.camera_info.usb_type_descriptor)}")
            print(f"  Product ID: {dev.get_info(rs.camera_info.product_id)}")
            
            # Check sensors
            print(f"\n  Available Sensors:")
            for sensor in dev.sensors:
                print(f"    - {sensor.get_info(rs.camera_info.name)}")
                
        return True
        
    except Exception as e:
        print(f"ERROR: {str(e)}")
        return False


# ==============================================================================
# Test 2: Stream Capabilities
# ==============================================================================

def test_stream_capabilities():
    """Test and list all supported stream configurations"""
    print("\n" + "="*60)
    print("TEST 2: Stream Capabilities")
    print("="*60)
    
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("ERROR: No devices found")
            return
            
        dev = devices[0]
        
        # Dictionary to store capabilities
        capabilities = {
            'depth': [],
            'color': [],
            'infrared': []
        }
        
        for sensor in dev.sensors:
            print(f"\nSensor: {sensor.get_info(rs.camera_info.name)}")
            profiles = sensor.get_stream_profiles()
            
            for profile in profiles:
                if hasattr(profile, 'stream_type'):
                    stream_type = str(profile.stream_type())
                    
                    if profile.stream_type() in [rs.stream.depth, rs.stream.color, rs.stream.infrared]:
                        vp = profile.as_video_stream_profile()
                        
                        mode = {
                            'width': vp.width(),
                            'height': vp.height(),
                            'fps': profile.fps(),
                            'format': str(profile.format())
                        }
                        
                        # Categorize by stream type
                        if profile.stream_type() == rs.stream.depth:
                            if mode not in capabilities['depth']:
                                capabilities['depth'].append(mode)
                        elif profile.stream_type() == rs.stream.color:
                            if mode not in capabilities['color']:
                                capabilities['color'].append(mode)
                        elif profile.stream_type() == rs.stream.infrared:
                            if mode not in capabilities['infrared']:
                                capabilities['infrared'].append(mode)
        
        # Display sorted capabilities
        for stream_type, modes in capabilities.items():
            print(f"\n{stream_type.upper()} Stream Capabilities:")
            # Sort by resolution and fps
            sorted_modes = sorted(modes, key=lambda x: (x['width'], x['height'], x['fps']))
            for mode in sorted_modes[:10]:  # Show first 10 to avoid clutter
                print(f"  {mode['width']}x{mode['height']} @ {mode['fps']}fps ({mode['format']})")
                
        # Save capabilities to file
        with open('camera_capabilities.json', 'w') as f:
            json.dump(capabilities, f, indent=2)
        print("\nCapabilities saved to 'camera_capabilities.json'")
        
    except Exception as e:
        print(f"ERROR: {str(e)}")


# ==============================================================================
# Test 3: RGB Stream Data Collection
# ==============================================================================

def test_rgb_data():
    """Test RGB stream and collect raw data statistics"""
    print("\n" + "="*60)
    print("TEST 3: RGB Stream Raw Data Analysis")
    print("="*60)
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure color stream (lower resolution for RPi)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        print("RGB stream started successfully")
        print("Collecting 100 frames for analysis...")
        
        frame_count = 0
        frame_times = []
        rgb_stats = []
        
        start_time = time.time()
        
        while frame_count < 100:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
                
            frame_time = time.time()
            frame_times.append(frame_time)
            
            # Convert to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Calculate statistics
            stats = {
                'frame': frame_count,
                'timestamp': color_frame.get_timestamp(),
                'shape': color_image.shape,
                'dtype': str(color_image.dtype),
                'mean_r': float(np.mean(color_image[:,:,2])),
                'mean_g': float(np.mean(color_image[:,:,1])),
                'mean_b': float(np.mean(color_image[:,:,0])),
                'std_r': float(np.std(color_image[:,:,2])),
                'std_g': float(np.std(color_image[:,:,1])),
                'std_b': float(np.std(color_image[:,:,0])),
                'min': int(np.min(color_image)),
                'max': int(np.max(color_image))
            }
            rgb_stats.append(stats)
            
            # Save first and last frame as raw numpy arrays
            if frame_count == 0:
                np.save('rgb_first_frame.npy', color_image)
                print(f"  First frame saved (shape: {color_image.shape}, dtype: {color_image.dtype})")
            elif frame_count == 99:
                np.save('rgb_last_frame.npy', color_image)
                print(f"  Last frame saved")
                
            frame_count += 1
            
            # Progress indicator
            if frame_count % 20 == 0:
                print(f"  Processed {frame_count} frames...")
        
        # Calculate FPS
        total_time = time.time() - start_time
        actual_fps = frame_count / total_time
        
        # Calculate frame time statistics
        frame_intervals = np.diff(frame_times)
        
        print(f"\nRGB Stream Statistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Total time: {total_time:.2f} seconds")
        print(f"  Actual FPS: {actual_fps:.2f}")
        print(f"  Frame interval mean: {np.mean(frame_intervals)*1000:.2f} ms")
        print(f"  Frame interval std: {np.std(frame_intervals)*1000:.2f} ms")
        
        # Color statistics summary
        mean_stats = {
            'avg_red': np.mean([s['mean_r'] for s in rgb_stats]),
            'avg_green': np.mean([s['mean_g'] for s in rgb_stats]),
            'avg_blue': np.mean([s['mean_b'] for s in rgb_stats])
        }
        
        print(f"\nColor Channel Averages:")
        print(f"  Red:   {mean_stats['avg_red']:.2f}")
        print(f"  Green: {mean_stats['avg_green']:.2f}")
        print(f"  Blue:  {mean_stats['avg_blue']:.2f}")
        
        # Save statistics
        with open('rgb_statistics.json', 'w') as f:
            json.dump({
                'summary': {
                    'total_frames': frame_count,
                    'total_time': total_time,
                    'actual_fps': actual_fps,
                    'mean_stats': mean_stats
                },
                'frame_stats': rgb_stats[:10]  # Save first 10 frames details
            }, f, indent=2)
        
        print("\nStatistics saved to 'rgb_statistics.json'")
        
    except Exception as e:
        print(f"ERROR: {str(e)}")
    finally:
        pipeline.stop()
        print("RGB stream stopped")


# ==============================================================================
# Test 4: Depth Stream Data Collection
# ==============================================================================

def test_depth_data():
    """Test depth stream and analyze depth data"""
    print("\n" + "="*60)
    print("TEST 4: Depth Stream Raw Data Analysis")
    print("="*60)
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure depth stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        print("Depth stream started successfully")
        print("Collecting 100 frames for analysis...")
        
        frame_count = 0
        depth_stats = []
        
        while frame_count < 100:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
                
            # Convert to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Get actual distance values in meters
            depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
            depth_in_meters = depth_image * depth_scale
            
            # Calculate statistics (excluding zero values)
            valid_depths = depth_in_meters[depth_in_meters > 0]
            
            if len(valid_depths) > 0:
                stats = {
                    'frame': frame_count,
                    'timestamp': depth_frame.get_timestamp(),
                    'shape': depth_image.shape,
                    'depth_scale': depth_scale,
                    'min_distance_m': float(np.min(valid_depths)),
                    'max_distance_m': float(np.max(valid_depths)),
                    'mean_distance_m': float(np.mean(valid_depths)),
                    'std_distance_m': float(np.std(valid_depths)),
                    'valid_pixels': len(valid_depths),
                    'total_pixels': depth_image.size,
                    'coverage_percent': (len(valid_depths) / depth_image.size) * 100
                }
                depth_stats.append(stats)
            
            # Save first frame
            if frame_count == 0:
                np.save('depth_first_frame_raw.npy', depth_image)
                np.save('depth_first_frame_meters.npy', depth_in_meters)
                print(f"  First depth frame saved (shape: {depth_image.shape})")
                print(f"  Depth scale: {depth_scale} meters/unit")
                
            frame_count += 1
            
            if frame_count % 20 == 0:
                print(f"  Processed {frame_count} frames...")
        
        # Summary statistics
        print(f"\nDepth Stream Statistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Average min distance: {np.mean([s['min_distance_m'] for s in depth_stats]):.3f} m")
        print(f"  Average max distance: {np.mean([s['max_distance_m'] for s in depth_stats]):.3f} m")
        print(f"  Average coverage: {np.mean([s['coverage_percent'] for s in depth_stats]):.1f}%")
        
        # Save statistics
        with open('depth_statistics.json', 'w') as f:
            json.dump({
                'summary': {
                    'total_frames': frame_count,
                    'depth_scale': depth_scale,
                    'avg_min_distance': np.mean([s['min_distance_m'] for s in depth_stats]),
                    'avg_max_distance': np.mean([s['max_distance_m'] for s in depth_stats])
                },
                'frame_stats': depth_stats[:10]
            }, f, indent=2)
            
        print("Statistics saved to 'depth_statistics.json'")
        
    except Exception as e:
        print(f"ERROR: {str(e)}")
    finally:
        pipeline.stop()
        print("Depth stream stopped")


# ==============================================================================
# Test 5: Infrared Stream Test
# ==============================================================================

def test_infrared_data():
    """Test infrared streams and collect data"""
    print("\n" + "="*60)
    print("TEST 5: Infrared Stream Raw Data Analysis")
    print("="*60)
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure infrared streams
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
    
    try:
        pipeline.start(config)
        print("Infrared streams started successfully")
        print("Collecting 50 frames for analysis...")
        
        frame_count = 0
        ir_stats = []
        
        while frame_count < 50:
            frames = pipeline.wait_for_frames()
            ir1_frame = frames.get_infrared_frame(1)
            ir2_frame = frames.get_infrared_frame(2)
            
            if not ir1_frame or not ir2_frame:
                continue
                
            # Convert to numpy arrays
            ir1_image = np.asanyarray(ir1_frame.get_data())
            ir2_image = np.asanyarray(ir2_frame.get_data())
            
            stats = {
                'frame': frame_count,
                'ir1_mean': float(np.mean(ir1_image)),
                'ir1_std': float(np.std(ir1_image)),
                'ir2_mean': float(np.mean(ir2_image)),
                'ir2_std': float(np.std(ir2_image)),
                'difference_mean': float(np.mean(np.abs(ir1_image.astype(float) - ir2_image.astype(float))))
            }
            ir_stats.append(stats)
            
            # Save first frame
            if frame_count == 0:
                np.save('ir_left_first_frame.npy', ir1_image)
                np.save('ir_right_first_frame.npy', ir2_image)
                print(f"  First IR frames saved (shape: {ir1_image.shape})")
                
            frame_count += 1
            
            if frame_count % 10 == 0:
                print(f"  Processed {frame_count} frames...")
        
        print(f"\nInfrared Stream Statistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Left IR average intensity: {np.mean([s['ir1_mean'] for s in ir_stats]):.2f}")
        print(f"  Right IR average intensity: {np.mean([s['ir2_mean'] for s in ir_stats]):.2f}")
        
        # Save statistics
        with open('ir_statistics.json', 'w') as f:
            json.dump({
                'summary': {
                    'total_frames': frame_count,
                    'left_avg': np.mean([s['ir1_mean'] for s in ir_stats]),
                    'right_avg': np.mean([s['ir2_mean'] for s in ir_stats])
                },
                'frame_stats': ir_stats[:10]
            }, f, indent=2)
            
        print("Statistics saved to 'ir_statistics.json'")
        
    except Exception as e:
        print(f"ERROR: {str(e)}")
    finally:
        pipeline.stop()
        print("Infrared streams stopped")


# ==============================================================================
# Test 6: Multi-Stream Performance Test
# ==============================================================================

def test_multi_stream_performance():
    """Test performance with multiple streams enabled"""
    print("\n" + "="*60)
    print("TEST 6: Multi-Stream Performance Test")
    print("="*60)
    
    test_configs = [
        {
            'name': 'Depth only (640x480)',
            'streams': [
                (rs.stream.depth, 640, 480, rs.format.z16, 30)
            ]
        },
        {
            'name': 'RGB only (640x480)',
            'streams': [
                (rs.stream.color, 640, 480, rs.format.bgr8, 30)
            ]
        },
        {
            'name': 'Depth + RGB (640x480)',
            'streams': [
                (rs.stream.depth, 640, 480, rs.format.z16, 30),
                (rs.stream.color, 640, 480, rs.format.bgr8, 30)
            ]
        },
        {
            'name': 'Depth + IR (640x480)',
            'streams': [
                (rs.stream.depth, 640, 480, rs.format.z16, 30),
                (rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            ]
        },
        {
            'name': 'Low res - All streams (480x270)',
            'streams': [
                (rs.stream.depth, 480, 270, rs.format.z16, 30),
                (rs.stream.color, 480, 270, rs.format.bgr8, 30),
                (rs.stream.infrared, 1, 480, 270, rs.format.y8, 30)
            ]
        }
    ]
    
    results = []
    
    for test_config in test_configs:
        print(f"\nTesting: {test_config['name']}")
        
        pipeline = rs.pipeline()
        config = rs.config()
        
        # Configure streams
        for stream_type, width, height, format, fps in test_config['streams']:
            config.enable_stream(stream_type, width, height, format, fps)
        
        try:
            pipeline.start(config)
            
            # Warm up
            for _ in range(10):
                pipeline.wait_for_frames()
            
            # Measure performance
            start_time = time.time()
            frame_count = 0
            errors = 0
            
            while frame_count < 100:
                try:
                    frames = pipeline.wait_for_frames()
                    frame_count += 1
                except:
                    errors += 1
                    
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time
            
            result = {
                'config': test_config['name'],
                'fps': fps,
                'errors': errors,
                'time': elapsed_time
            }
            results.append(result)
            
            print(f"  Result: {fps:.2f} FPS ({errors} errors)")
            
            pipeline.stop()
            
        except Exception as e:
            print(f"  ERROR: {str(e)}")
            results.append({
                'config': test_config['name'],
                'error': str(e)
            })
    
    # Save results
    with open('performance_results.json', 'w') as f:
        json.dump(results, f, indent=2)
    
    print("\nPerformance results saved to 'performance_results.json'")


# ==============================================================================
# Test 7: Continuous Data Logger
# ==============================================================================

def test_data_logger(duration=30):
    """Log continuous data from all streams for analysis"""
    print("\n" + "="*60)
    print(f"TEST 7: Continuous Data Logger ({duration} seconds)")
    print("="*60)
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure all streams at lower resolution for stability
    config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 480, 270, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 1, 480, 270, rs.format.y8, 30)
    
    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"realsense_data_{timestamp}"
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Logging data to directory: {output_dir}")
    
    try:
        pipeline.start(config)
        
        # Get depth scale
        depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        
        log_data = {
            'metadata': {
                'start_time': timestamp,
                'depth_scale': depth_scale,
                'duration_target': duration
            },
            'frames': []
        }
        
        start_time = time.time()
        frame_count = 0
        
        print("Logging started. Press Ctrl+C to stop early.")
        
        while (time.time() - start_time) < duration:
            try:
                frames = pipeline.wait_for_frames()
                
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                ir_frame = frames.get_infrared_frame(1)
                
                if depth_frame and color_frame and ir_frame:
                    current_time = time.time() - start_time
                    
                    # Get center point distance
                    width = depth_frame.get_width()
                    height = depth_frame.get_height()
                    center_distance = depth_frame.get_distance(width // 2, height // 2)
                    
                    # Log frame data
                    frame_data = {
                        'frame': frame_count,
                        'time': current_time,
                        'depth_timestamp': depth_frame.get_timestamp(),
                        'color_timestamp': color_frame.get_timestamp(),
                        'center_distance_m': center_distance
                    }
                    log_data['frames'].append(frame_data)
                    
                    # Save snapshot every 5 seconds
                    if frame_count % 150 == 0:  # Approximately every 5 seconds at 30fps
                        # Save arrays
                        depth_array = np.asanyarray(depth_frame.get_data())
                        color_array = np.asanyarray(color_frame.get_data())
                        ir_array = np.asanyarray(ir_frame.get_data())
                        
                        np.save(f"{output_dir}/depth_{frame_count}.npy", depth_array)
                        np.save(f"{output_dir}/color_{frame_count}.npy", color_array)
                        np.save(f"{output_dir}/ir_{frame_count}.npy", ir_array)
                        
                        print(f"  Saved snapshot at {current_time:.1f}s (frame {frame_count})")
                    
                    frame_count += 1
                    
            except KeyboardInterrupt:
                print("\nLogging stopped by user")
                break
            except Exception as e:
                print(f"Frame error: {e}")
                continue
        
        # Final statistics
        actual_duration = time.time() - start_time
        actual_fps = frame_count / actual_duration
        
        log_data['metadata']['actual_duration'] = actual_duration
        log_data['metadata']['total_frames'] = frame_count
        log_data['metadata']['actual_fps'] = actual_fps
        
        # Save log data
        with open(f"{output_dir}/log_data.json", 'w') as f:
            json.dump(log_data, f, indent=2)
        
        print(f"\nLogging completed:")
        print(f"  Duration: {actual_duration:.1f} seconds")
        print(f"  Frames: {frame_count}")
        print(f"  FPS: {actual_fps:.2f}")
        print(f"  Data saved to: {output_dir}/")
        
    except Exception as e:
        print(f"ERROR: {str(e)}")
    finally:
        pipeline.stop()
        print("Data logger stopped")


# ==============================================================================
# Main Menu
# ==============================================================================

if __name__ == "__main__":
    print("\nIntel RealSense D435 Headless Test Suite")
    print("For Raspberry Pi with Ubuntu Server 22.04")
    print("="*60)
    
    # Check if running as root (sometimes needed for USB access)
    if os.geteuid() == 0:
        print("WARNING: Running as root. Consider adding user to 'video' group instead.")
    
    while True:
        print("\nAvailable Tests:")
        print("1. Test Connection and Device Info")
        print("2. List Stream Capabilities")
        print("3. RGB Raw Data Analysis")
        print("4. Depth Raw Data Analysis")
        print("5. Infrared Raw Data Analysis")
        print("6. Multi-Stream Performance Test")
        print("7. Continuous Data Logger (30s)")
        print("8. Run All Tests")
        print("0. Exit")
        
        choice = input("\nSelect test (0-8): ")
        
        try:
            if choice == '1':
                test_connection()
            elif choice == '2':
                test_stream_capabilities()
            elif choice == '3':
                test_rgb_data()
            elif choice == '4':
                test_depth_data()
            elif choice == '5':
                test_infrared_data()
            elif choice == '6':
                test_multi_stream_performance()
            elif choice == '7':
                duration = input("Enter duration in seconds (default 30): ")
                duration = int(duration) if duration else 30
                test_data_logger(duration)
            elif choice == '8':
                # Run all tests
                print("\nRunning all tests...")
                if test_connection():
                    test_stream_capabilities()
                    test_rgb_data()
                    test_depth_data()
                    test_infrared_data()
                    test_multi_stream_performance()
                    print("\nAll tests completed!")
                else:
                    print("Connection test failed. Please check your device.")
            elif choice == '0':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please try again.")
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        except Exception as e:
            print(f"\nERROR: {str(e)}")
            print("Make sure:")
            print("  1. Camera is connected")
            print("  2. pyrealsense2 is installed: pip3 install pyrealsense2")
            print("  3. User has USB permissions: sudo usermod -a -G video $USER")
            print("  4. Then logout and login again")
            
    print("\nTest suite finished.")