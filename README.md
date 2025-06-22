# 🔥 Fire-Fighting Robot

<div align="center">
  
  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
  [![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
  [![Python](https://img.shields.io/badge/Python-3.8+-green)](https://www.python.org/)
  [![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
  
  <h3>Autonomous Fire Detection and Suppression Robot</h3>
  
  <img src="docs/videos/robot_photo.jpg" alt="Fire Fighting Robot" width="600">
  
</div>

---

## 🎯 Project Overview

An autonomous robotic system designed to detect and respond to fire emergencies. The robot combines thermal imaging, LIDAR navigation, and real-time monitoring capabilities to provide an effective fire detection and suppression solution.

### ✨ Key Features

- 🔥 **Thermal Fire Detection** - MLX90640 thermal camera for heat source identification
- 🗺️ **Autonomous Navigation** - YDLIDAR G2 for SLAM and obstacle avoidance  
- 🌐 **Remote Control Dashboard** - Web-based interface for real-time monitoring
- 🤖 **ROS2 Architecture** - Modular design built on ROS2 Humble
- 📡 **Wireless Operation** - WiFi connectivity for remote access

---

## 🎬 Demo Videos

<div align="center">

### Dashboard Control & Fire Suppression
<table>
<tr>
<td width="50%">
  <video src="docs/videos/aim_dashboard.mp4" width="100%" autoplay loop muted>
    Your browser does not support the video tag.
  </video>
  <p><i>Dashboard-controlled aiming system</i></p>
</td>
<td width="50%">
  <video src="docs/videos/aim_hot_water.mp4" width="100%" autoplay loop muted>
    Your browser does not support the video tag.
  </video>
  <p><i>Hot water fire suppression test</i></p>
</td>
</tr>
</table>

### System Calibration & Navigation
<table>
<tr>
<td width="50%">
  <video src="docs/videos/body_calibration.mp4" width="100%" autoplay loop muted>
    Your browser does not support the video tag.
  </video>
  <p><i>Robot body calibration process</i></p>
</td>
<td width="50%">
  <video src="docs/videos/obstacles_avoidance.mp4" width="100%" autoplay loop muted>
    Your browser does not support the video tag.
  </video>
  <p><i>Autonomous obstacle avoidance</i></p>
</td>
</tr>
</table>

### Fire Detection Views
<table>
<tr>
<td width="50%">
  <video src="docs/videos/above_view_fire.mp4" width="100%" autoplay loop muted>
    Your browser does not support the video tag.
  </video>
  <p><i>Top-down fire detection view</i></p>
</td>
<td width="50%">
  <video src="docs/videos/front_view_fire.mp4" width="100%" autoplay loop muted>
    Your browser does not support the video tag.
  </video>
  <p><i>Front-facing fire detection</i></p>
</td>
</tr>
</table>

</div>

---

## 🚀 Quick Start

### 1. SSH Connection
```bash
ssh ubuntu-robot-pi4@ubuntu-robot.local
# Password: 000000
```

### 2. Launch Robot System
```bash
ros2 launch fire_fighting_robot_bringup robot_bringup.launch.py
```

### 3. Access Dashboard
Open browser and navigate to: `http://robot-ip:5001`

---

## 📋 Documentation

<div align="center">

### 📖 **[Common Use Codes & Commands](docs/COMMON_CODES.md)**

Complete reference for all frequently used commands, including:
- SSH & Network Setup
- ROS2 Launch Commands  
- System Control
- Troubleshooting
- Dashboard Operations

</div>

---

## 🔧 Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| **Controller** | Raspberry Pi 4 (4GB) | Main processing unit |
| **LIDAR** | YDLIDAR G2 | 360° environmental scanning |
| **Thermal Camera** | MLX90640 | Fire detection (32x24 IR array) |
| **Motor Driver** | PCA9685 | 16-channel PWM control |
| **Motors** | 4x DC Motors | Differential drive system |

---

## 💻 Software Stack

- **OS**: Ubuntu 22.04 Server
- **Framework**: ROS2 Humble
- **Languages**: Python 3.8+, C++
- **Dashboard**: Flask + WebSocket
- **Key Packages**:
  - `fire_fighting_robot_bringup` - System launcher
  - `pca9685_motor_driver_py` - Motor control
  - `mlx90640_driver` - Thermal imaging
  - `robot_dashboard` - Web interface

---

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">
  
  **Made with ❤️ for UCL MECH0073**
  
  [Report Issue](https://github.com/YueYang101/Fire-Fighting-Robot/issues) · [Contact](https://github.com/YueYang101)
  
</div>