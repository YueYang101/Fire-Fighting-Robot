#!/usr/bin/env python3
"""
ROS 2 node for MLX90640 thermal camera
Publishes thermal frames as both topic and service
"""

import time
import warnings
import numpy as np
import board
import busio
import adafruit_mlx90640

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from mlx90640_interfaces.msg import ThermalFrame
from mlx90640_interfaces.srv import GetThermalFrame

# Suppress I2C frequency warning
warnings.filterwarnings(
    "ignore", category=RuntimeWarning,
    message="I2C frequency is not settable in python, ignoring!"
)

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 0.5)  # Hz
        self.declare_parameter('frame_id', 'thermal_camera')
        self.declare_parameter('enable_publisher', True)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.enable_publisher = self.get_parameter('enable_publisher').value
        
        # Initialize thermal camera with SAFE refresh rate
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.mlx = adafruit_mlx90640.MLX90640(i2c)
            # Use 1Hz - this is the most stable
            self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ
            self.flat = np.zeros(24 * 32, dtype=np.float32)
            self.get_logger().info('MLX90640 thermal camera initialized at 1Hz (safe rate)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize thermal camera: {e}')
            raise
        
        # Frame counter
        self.frame_count = 0
        
        # Error handling
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # Create publisher
        if self.enable_publisher:
            self.publisher = self.create_publisher(ThermalFrame, 'thermal_frame', 10)
            self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_thermal_frame)
            self.get_logger().info(f'Publishing thermal frames at {self.publish_rate} Hz')
        
        # Create service
        self.get_frame_service = self.create_service(
            GetThermalFrame, 
            'get_thermal_frame', 
            self.handle_get_thermal_frame
        )
        
        self.get_logger().info('Thermal camera node ready')
    
    def read_thermal_frame(self):
        """Read a frame from the thermal camera with error handling"""
        try:
            self.mlx.getFrame(self.flat)
            self.frame_count += 1
            self.consecutive_errors = 0  # Reset error counter on success
            
            # Calculate statistics
            min_temp = float(np.min(self.flat))
            max_temp = float(np.max(self.flat))
            avg_temp = float(np.mean(self.flat))
            
            # Get center temperature (middle of 24x32 array)
            frame2d = self.flat.reshape((24, 32))
            center_temp = float(frame2d[12, 16])
            
            # Create message
            msg = ThermalFrame()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            msg.width = 32
            msg.height = 24
            msg.data = self.flat.tolist()
            
            msg.min_temp = min_temp
            msg.max_temp = max_temp
            msg.avg_temp = avg_temp
            msg.center_temp = center_temp
            msg.frame_count = self.frame_count
            
            return True, msg
            
        except (ValueError, OSError) as e:
            self.consecutive_errors += 1
            self.get_logger().warning(
                f'Error reading thermal frame ({self.consecutive_errors}/{self.max_consecutive_errors}): {e}'
            )
            
            if self.consecutive_errors >= self.max_consecutive_errors:
                self.get_logger().error('Too many consecutive errors, trying to reinitialize camera')
                self.reinitialize_camera()
            
            # Small delay before retry
            time.sleep(0.1)
            return False, None
        
        except Exception as e:
            self.get_logger().error(f'Unexpected error reading thermal frame: {e}')
            return False, None
    
    def reinitialize_camera(self):
        """Try to reinitialize the camera after errors"""
        try:
            self.get_logger().info('Attempting to reinitialize thermal camera...')
            time.sleep(1)  # Give it a moment
            
            i2c = busio.I2C(board.SCL, board.SDA)
            self.mlx = adafruit_mlx90640.MLX90640(i2c)
            self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ
            
            self.consecutive_errors = 0
            self.get_logger().info('Camera reinitialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to reinitialize camera: {e}')
    
    def publish_thermal_frame(self):
        """Timer callback to publish thermal frames"""
        success, msg = self.read_thermal_frame()
        if success and msg:
            self.publisher.publish(msg)
            self.get_logger().debug(
                f'Published frame {self.frame_count}: '
                f'min={msg.min_temp:.1f}°C, max={msg.max_temp:.1f}°C'
            )
    
    def handle_get_thermal_frame(self, request, response):
        """Service callback to get a single thermal frame"""
        success, msg = self.read_thermal_frame()
        
        response.success = success
        if success:
            response.message = f'Frame {self.frame_count} captured successfully'
            response.frame = msg
        else:
            response.message = 'Failed to capture thermal frame'
            response.frame = ThermalFrame()  # Empty frame
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ThermalCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()