# arm_mine_explorer

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This repository is a meta-package containing all the components required to control and simulate the **Universal Robots UR5e** for the Mine Explorer project. Its primary goal is to provide a modular framework for landmine detection and neutralization.

The project is structured to separate physical description, simulation parameters, hardware communication, and high-level motion planning.



## Repository Structure

| Package | Status | Description |
| :--- | :--- | :--- |
| `arm_mine_explorer_description` | ✅ Done | URDF/Xacro models, visual meshes, and kinematic chain. |
| `arm_mine_explorer_gazebo` | ✅ Done | Gazebo simulation setup, worlds, and ros2_control integration. |
| `arm_mine_explorer_controllers` | 🛠️ Planned | Specific controller configurations (Position/Velocity/Effort/Hybrid/...). |
| `arm_mine_explorer_hardware_interface` | 🛠️ Planned | Custom hardware abstraction layer for real-world operation. |
| `arm_mine_explorer_moveit2_config` | 🛠️ Planned | MoveIt 2 configuration for path planning and obstacle avoidance. |


## Quick Start

To visualize the arm model:

```bash
ros2 launch arm_mine_explorer_description display.launch.py
```

To run the simulationin Gazebo:
```bash
ros2 launch arm_mine_explorer_gazebo gazebo.launch.py
```