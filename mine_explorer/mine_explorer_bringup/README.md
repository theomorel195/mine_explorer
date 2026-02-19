# mine_explorer_bringup

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This package serves as the main entry point for the **Mine Explorer** robotic system. It centralizes the launch configurations to initialize the simulation, localization, and mapping components simultaneously.

## Description

The `mine_explorer_bringup` package provides high-level launch files that orchestrate multiple subsystems:
* **Simulation:** Launches Gazebo with the robot model
* **Localization:** Starts the EKF (Extended Kalman Filter) for fused odometry.
* **Mapping:** Initializes the SLAM and LiDAR processing nodes.

---

## Usage

### Launching the Full System (2D)

To launch the 2D simulation, EKF, SLAM all at once, run:
```bash
ros2 launch mine_explorer_bringup bringup_mine_slam2d_ekf.launch.py
```

### Launch Arguments

You can customize the launch using the following arguments:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `world` | `empty.world` | Name of the world file loadfrom `mine_explorer_resources`. |

### Example with custom world:

```bash
ros2 launch mine_explorer_bringup bringup_mine_slam2d_ekf.launch.py world:=world_1.world
```
