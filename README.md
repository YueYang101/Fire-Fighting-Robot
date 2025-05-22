# Fire-Fighting Robot — Reproduction & Data-Availability Guide

This repository accompanies the paper **“Design of an Automated Fire-Fighting Robot”** (University College London, 2025).  
It contains *every* digital artefact needed to rebuild the hardware, replay the simulations and verify the results discussed in the manuscript.

---

## 1 | Repository layout (top level)

| Path | Purpose |
|------|---------|
| `Cad Files/` | Fusion 360 assemblies (`*.f3d`), neutral STEP & STL exports for machining / 3-D printing |
| `pca9685_interfaces/` | Custom ROS 2 Humble message & service definitions for the PCA9685 PWM expander |
| `pca9685_motor_driver_py/` | Python driver node that publishes `/cmd_motor` and `/motor_state` topics |
| `Image Processing/` | YOLOv8-based fire-detector notebooks, training data, pretrained weights (`*.pt`) |
| `Test_folder/` | Gazebo world, RViz config, ROS bag files, CSV motor-current logs for the figures in the paper |
| `Flask_app.py`, `Pi_motor_control.py` | Stand-alone demos for channel mapping & manual drive |
| `docs/` (generated on first build) | Build manual, Bill of Materials, lab test protocols, slide deck |

> **Tip**  Clone with `--recursive` to pull CAD and documentation sub-modules automatically.

---

## 2 | Reproducing the results

```bash
# 1.  Install ROS 2 Humble on Ubuntu 22.04
sudo apt install ros-humble-desktop-full

# 2.  Build all packages
colcon build --symlink-install
source install/setup.bash

# 3.  Bring-up demo (mapping, fire-detection, navigation, trigger control)
ros2 launch bringup bringup.launch.py
