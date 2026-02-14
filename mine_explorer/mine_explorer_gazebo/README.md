# mine_explorer_gazebo

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This package is the **Main Simulation Entry Point** for the Mine Explorer project. It integrates the complete robot (Husky base + UR5e arm + full sensor suite) into the Gazebo physics engine. 

It provides the necessary configurations to spawn the robot, activate its controllers, and visualize real-time sensor data (LiDAR, Cameras, Sonars) within a simulated environment.


## Package Structure
```text
mine_explorer_gazebo/
├── config/             # Global controller configurations (Base + Arm)
├── launch/             # Launch files for full system simulation and RViz
├── worlds/             # Gazebo world files (.world, .model) for mine scenarios
├── CMakeLists.txt
└── package.xml
```

## Usage

### Launching the Full Simulation

To launch Gazebo, spawn the robot, and open RViz for sensor visualization, run:

```bash
ros2 launch mine_explorer_gazebo gazebo.launch.py
``` 

### Key Features

* **Integrated Control:** Manages both the `diff_drive_controller` for the Husky and the `joint_trajectory_controller` for the UR5e simultamiously.
* **Sensor Visualization:** Pre-configured RViz setup to display:
    * **3D Point Cloud** from the Velodyne VLP-16 LiDAR.
    * **RGB-D Streams** from the dual RealSense cameras.
    * **Range Cones** for the 6 ultrasonic sonars.
* **Modular Environments:** Support for custom worlds designed for landmine exploration and neutralization tasks (coming soon).

## Launch Arguments

You can customize the global robot assembly using the following arguments :

| Argument | Default | Description |
| :--- | :--- | :--- |
| `name_arm` | `ArmMineExplorerSystem` | Name of the hardware system for the arm. |
| `prefix_arm` | `"arm_"` | Prefix for all arm joint and link names (essential for avoiding name conflicts). |
| `name_base` | `MobileBaseMineExplorerSystem` | Name of the hardware system for the mobile robot. |
| `prefix_base` | `""` | Prefix for all base link names (essential for avoiding name conflicts). |
| `name_lidar` | `lidar` | Name of the hardware system for the lidar. |
| `use_sim` | `true` | Toggle for Gazebo simulation compatibility and specific hardware plugins. |
| `joint_limit_params` | `joint_limits.yaml` | Path to the YAML file defining min/max angles and velocities. |
| `kinematics_params` | `default_kinematics.yaml` | Path to the specific kinematic calibration of the UR5e. |
| `safety_limits` | `true` | Enables the safety controller to prevent joint over-extension. |
| `safety_pos_margin` | `0.15` | The safety margin (in radians) added to the lower and upper joint limits. |
| `safety_k_position` | `20` | The k-position factor used by the safety controller for limit enforcement. |
| `initial_positions_file` | `initial_positions.yaml` | Path to the YAML file defining the arm's default pose at start. |
| `world` | `empty_world.model` | Path to world you want to use forthe simulation. |

