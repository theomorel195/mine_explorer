# mine_explorer

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
The **Mine Explorer** repository is the central integration hub for an autonomous landmine exploration robot. It combines a **Husky mobile base** with a **UR5e robotic arm** and a sophisticated sensor suite designed for hazardous environment mapping and neutralization.

This project is divided into modular packages to separate the physical robot definition from the simulation and control logic.


## Repository Structure

| Package | Status | Description |
| :--- | :--- | :--- |
| `mine_explorer_description` | ✅ Ready | Robot assembly (Husky + UR5e + Sensors). |
| `mine_explorer_gazebo` | ✅ Ready | Simulation & Controller management. |
| `mine_explorer_navigation` | 🛠️ In Progress | Nav2 configuration & Waypoint following. |
| `mine_explorer_localization` | 🛠️ In Progress | SLAM configuration. |
| `mine_explorer_perception` | 🛠️ In Progress | Mine detection algorithms & Pointcloud filtering. |
| `mine_explorer_bringup` | 🛠️ In Progress | One-click launch files for the whole system. |


## System Components

### 1. Mobility & Manipulation
* **Base**: Clearpath Husky (4WD Differential Drive).
* **Arm**: Universal Robots UR5e (6-DOF manipulator).

### 2. Perception Suite
* **Long Range**: Velodyne VLP-16 LiDAR for 3D mapping and SLAM.
* **Short Range**: 6x Ultrasonic Sonars for 360° obstacle avoidance.
* **Vision**: Dual RealSense cameras (D455 for mine detection, D435 for tool precision).


## Quick Commands

To visualize the full robot model in RViz :

```bash
ros2 launch mine_explorer_description display.launch.py
```

To start the full Gazebo simulation with controllers and data from sensors :

```bash
ros2 launch mine_explorer_gazebo gazebo.launch.py
```


