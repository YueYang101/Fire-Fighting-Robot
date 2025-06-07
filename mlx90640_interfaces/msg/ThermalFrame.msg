# Message definition for thermal camera frame
# mlx90640_interfaces/msg/ThermalFrame.msg

# Header for timestamp and frame_id
std_msgs/Header header

# Image dimensions
uint32 width        # 32 pixels
uint32 height       # 24 pixels

# Temperature data (in Celsius)
float32[] data      # 768 values (24x32)

# Statistics
float32 min_temp
float32 max_temp
float32 avg_temp
float32 center_temp

# Frame counter
uint32 frame_count