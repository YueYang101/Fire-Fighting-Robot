# Fire Fighting Robot Operations Guide

## SSH Configuration

### Set up SSH key for passwordless login

```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t rsa -b 4096

# Copy SSH key to Raspberry Pi
ssh-copy-id ubuntu-robot-pi4@ubuntu-robot.local
# Or use IP addresses:
ssh-copy-id ubuntu-robot-pi4@192.168.124.39
ssh-copy-id ubuntu-robot-pi4@192.168.0.121
```

### SSH Connection Options

```bash
# Connect via hostname
ssh ubuntu-robot-pi4@ubuntu-robot.local

# Connect via IP (Option 1)
ssh ubuntu-robot-pi4@192.168.124.39

# Connect via IP (Option 2)
ssh ubuntu-robot-pi4@192.168.0.121
```

**Default Password**: `000000`

## WiFi Configuration

### Scan and List Available Networks

```bash
sudo nmcli device wifi rescan && nmcli device wifi list
```

### Connect to WiFi Networks

```bash
# Connect to robot_connection network
sudo nmcli device wifi connect "robot_connection" password "00000000"

# Connect to H3C_831A87 network
sudo nmcli device wifi connect "H3C_831A87" password "987654321@"
```

## ROS2 Launch Commands

### Option 1: Launch Everything at Once

```bash
ros2 launch fire_fighting_robot_bringup robot_bringup.launch.py
```

### Option 2: Launch Components Individually

#### 1. Launch Rosbridge WebSocket

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### 2. Launch Motor Driver Node

```bash
ros2 run pca9685_motor_driver_py motor_driver_node \
  --ros-args --params-file /home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/pca9685_motor_driver_py/config/motor_map.yaml
```

#### 3. Launch YDLIDAR

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=/home/ubuntu-robot-pi4/ros2_ws/src/ydlidar_ros2_driver/params/G2.yaml
```

#### 4. Launch Thermal Camera

```bash
~/ros2_ws/install/mlx90640_driver/bin/thermal_camera_node
```

## Useful Commands

### Navigate to Repository on Raspberry Pi

```bash
cd ~/ros2_ws/src/Fire-Fighting-Robot
```

### Shutdown Raspberry Pi

```bash
sudo shutdown -h now
```

## Dashboard Setup (Mac)

### Navigate to Project Directory

```bash
cd ~/Library/CloudStorage/OneDrive-Personal/UCL/Year\ 4\ UCL/MECH0073/GitHub_Repositories/Fire-fighting-Robot
```

### Activate Virtual Environment and Run Dashboard

```bash
# Activate virtual environment
source venv/bin/activate

# Navigate to dashboard directory
cd robot-dashboard

# Set Flask port
export FLASK_PORT=5001

# Run the dashboard
python run.py
```

### Scan Dashboard Files

```bash
python3 dashboard_scanner.py > dashboard_scan_output.txt
```

## Quick Reference

### Common IP Addresses
- **Hostname**: `ubuntu-robot.local`
- **IP Option 1**: `192.168.124.39`
- **IP Option 2**: `192.168.0.121`

### Default Credentials
- **Username**: `ubuntu-robot-pi4`
- **Password**: `000000`

### WiFi Networks
- **Network 1**: `robot_connection` (Password: `00000000`)
- **Network 2**: `H3C_831A87` (Password: `987654321@`)

### Important Paths
- **ROS2 Workspace**: `/home/ubuntu-robot-pi4/ros2_ws/`
- **Fire Fighting Robot Package**: `/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/`
- **Motor Config**: `/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/pca9685_motor_driver_py/config/motor_map.yaml`
- **YDLIDAR Config**: `/home/ubuntu-robot-pi4/ros2_ws/src/ydlidar_ros2_driver/params/G2.yaml`

## Troubleshooting Tips

1. **SSH Connection Issues**
   - Ensure Raspberry Pi is powered on and connected to the network
   - Try different IP addresses if hostname doesn't work
   - Check if SSH service is running: `sudo systemctl status ssh`

2. **WiFi Connection Issues**
   - Verify WiFi adapter is enabled: `nmcli radio wifi`
   - Enable if disabled: `nmcli radio wifi on`
   - Check current connection: `nmcli connection show`

3. **ROS2 Launch Issues**
   - Source ROS2 setup: `source /opt/ros/humble/setup.bash`
   - Source workspace: `source ~/ros2_ws/install/setup.bash`
   - Check if all dependencies are installed

4. **Dashboard Connection Issues**
   - Ensure Flask port is not already in use
   - Check firewall settings
   - Verify rosbridge WebSocket is running