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

## Tests

This package includes unit tests for the entire simulation as well as code style checks using Flake8.

### Running ROS 2 Unit Tests

To run the unit tests, make sure you are at the root of your ROS 2 workspace :

```bash
colcon test --packages-select mine_explorer_bringup --event-handlers console_cohesion+
```

After the tests finish, you can view detailed results with : 

```bash
colcon test-result --verbose
```
