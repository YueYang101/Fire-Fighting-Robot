# 📖 Common Use Codes & Commands

[← Back to Home](../README.md)

---

## 📋 Table of Contents

- [🔐 SSH & Network Connection](#-ssh--network-connection)
- [🚀 Quick Launch Commands](#-quick-launch-commands)
- [📡 Individual Node Commands](#-individual-node-commands)
- [🌐 Dashboard Operations](#-dashboard-operations)
- [🛠️ System Utilities](#️-system-utilities)
- [🔍 Troubleshooting Commands](#-troubleshooting-commands)
- [📝 Important Paths & Configs](#-important-paths--configs)

---

## 🔐 SSH & Network Connection

### SSH Access
```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t rsa -b 4096

# Copy SSH key to Raspberry Pi
ssh-copy-id ubuntu-robot-pi4@ubuntu-robot.local

# Connect via hostname
ssh ubuntu-robot-pi4@ubuntu-robot.local

# Connect via IP (Option 1)
ssh ubuntu-robot-pi4@192.168.124.39

# Connect via IP (Option 2)  
ssh ubuntu-robot-pi4@192.168.0.121
```

**Default Credentials:**
- Username: `ubuntu-robot-pi4`
- Password: `000000`

### WiFi Configuration
```bash
# Scan for available networks
sudo nmcli device wifi rescan && nmcli device wifi list

# Connect to robot_connection network
sudo nmcli device wifi connect "robot_connection" password "00000000"

# Connect to H3C_831A87 network
sudo nmcli device wifi connect "H3C_831A87" password "987654321@"

# Check current connection
nmcli connection show
```

---

## 🚀 Quick Launch Commands

### Complete System Launch
```bash
# Launch all nodes at once
ros2 launch fire_fighting_robot_bringup robot_bringup.launch.py
```

### WebSocket Bridge (for Dashboard)
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## 📡 Individual Node Commands

### Motor Driver Node
```bash
ros2 run pca9685_motor_driver_py motor_driver_node \
--ros-args --params-file /home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/pca9685_motor_driver_py/config/motor_map.yaml
```

### YDLIDAR Node
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/ubuntu-robot-pi4/ros2_ws/src/ydlidar_ros2_driver/params/G2.yaml
```

### Thermal Camera Node
```bash
~/ros2_ws/install/mlx90640_driver/bin/thermal_camera_node
```

---

## 🌐 Dashboard Operations

### On Development Machine
```bash
# Navigate to project
cd ~/Library/CloudStorage/OneDrive-Personal/UCL/Year\ 4\ UCL/MECH0073/GitHub_Repositories/Fire-fighting-Robot

# Activate virtual environment
source venv/bin/activate

# Navigate to dashboard directory
cd robot-dashboard

# Set Flask port
export FLASK_PORT=5001

# Run the dashboard
python run.py
```

### Dashboard Scanner
```bash
python3 dashboard_scanner.py > dashboard_scan_output.txt
```

---

## 🛠️ System Utilities

### ROS2 Environment Setup
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source ~/ros2_ws/install/setup.bash
```

### System Control
```bash
# Shutdown robot
sudo shutdown -h now

# Reboot robot
sudo reboot

# Check system status
systemctl status
```

### ROS2 Monitoring
```bash
# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /scan
ros2 topic echo /cmd_vel
ros2 topic echo /thermal_image

# Check node info
ros2 node info /motor_driver_node

# View TF tree
ros2 run tf2_tools view_frames
```

---

## 🔍 Troubleshooting Commands

### SSH Connection Issues
```bash
# Check if SSH service is running
sudo systemctl status ssh

# Restart SSH service
sudo systemctl restart ssh

# Check network connectivity
ping ubuntu-robot.local
```

### WiFi Issues
```bash
# Check WiFi status
nmcli radio wifi

# Enable WiFi if disabled
nmcli radio wifi on

# Restart network manager
sudo systemctl restart NetworkManager
```

### ROS2 Issues
```bash
# Check for missing dependencies
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -y

# Clean and rebuild workspace
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Process Management
```bash
# Find ROS2 processes
ps aux | grep ros2

# Kill specific process
kill -9 <PID>

# Kill all ROS2 processes
pkill -f ros2
```

---

## 📝 Important Paths & Configs

### Connection Info
| Parameter | Value |
|-----------|-------|
| **Hostname** | `ubuntu-robot.local` |
| **IP Option 1** | `192.168.124.39` |
| **IP Option 2** | `192.168.0.121` |
| **Username** | `ubuntu-robot-pi4` |
| **Password** | `000000` |

### Network Credentials
| Network | Password |
|---------|----------|
| **robot_connection** | `00000000` |
| **H3C_831A87** | `987654321@` |

### File Paths
```bash
# ROS2 Workspace
/home/ubuntu-robot-pi4/ros2_ws/

# Fire Fighting Robot Package
/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/

# Motor Configuration
/home/ubuntu-robot-pi4/ros2_ws/src/Fire-Fighting-Robot/pca9685_motor_driver_py/config/motor_map.yaml

# YDLIDAR Configuration
/home/ubuntu-robot-pi4/ros2_ws/src/ydlidar_ros2_driver/params/G2.yaml
```

---

## 💡 Quick Tips

1. **Always source ROS2** before running commands:
   ```bash
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   ```

2. **Check robot connectivity** before SSH:
   ```bash
   ping ubuntu-robot.local
   ```

3. **Use screen/tmux** for persistent sessions:
   ```bash
   screen -S robot_session
   # Run commands
   # Detach: Ctrl+A, D
   # Reattach: screen -r robot_session
   ```

4. **Monitor system resources**:
   ```bash
   htop  # CPU and memory usage
   df -h  # Disk usage
   ```

---

[← Back to Home](../README.md)