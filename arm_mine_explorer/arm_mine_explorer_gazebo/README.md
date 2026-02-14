# arm_mine_explorer_gazebo

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This package provides the simulation environment for the Mine Explorer arm. It integrates the UR5e robot description into **Gazebo**, configuring the physics, world environments, and the `ros2_control` loop necessary for simulated movements.

It acts as the bridge between the static URDF description and the dynamic simulation, allowing for testing of arm controllers and motion planning without physical hardware.



## Package Structure
```text
arm_mine_explorer_gazebo/
├── config/             # Hardware interface and gazebo_ros2_control parameters
├── launch/             # Launch files to spawn the robot and start controllers
├── worlds/             # Custom Gazebo environments (e.g., mine-like scenarios)
├── CMakeLists.txt
└── package.xml
```

## Usage

### Launching the simulation

To spawn the arm in an empty Gazebo world with the default controllers (Joint State Broadcaster and Arm Controller), run:

```bash
ros2 launch arm_mine_explorer_gazebo gazebo.launch.py
```

### Key Features
- **Automated Spawning:** Automatically processes Xacro and spawns the robot entity into the simulation.
- **Controller Sequencing:** Uses event handlers to ensure the `joint_state_broadcaster` is active before spawning the `arm_controller`, preventing initization errors.
- **World Selection:** Support for custom simulation environments tailored for mine detection tasks.


## Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `name` | `ArmMineExplorerSystem` | Name of the hardware system. |
| `prefix` | `""` | Prefix for all joint and link names (essential for avoiding name conflicts). |
| `use_sim` | `true` | Toggle for Gazebo simulation compatibility and specific hardware plugins. |
| `joint_limit_params` | `joint_limits.yaml` | Path to the YAML file defining min/max angles and velocities. |
| `kinematics_params` | `default_kinematics.yaml` | Path to the specific kinematic calibration of the UR5e. |
| `safety_limits` | `true` | Enables the safety controller to prevent joint over-extension. |
| `safety_pos_margin` | `0.15` | The safety margin (in radians) added to the lower and upper joint limits. |
| `safety_k_position` | `20` | The k-position factor used by the safety controller for limit enforcement. |
| `initial_positions_file` | `initial_positions.yaml` | Path to the YAML file defining the arm's default pose at start. |
| `world` | `empty_world.model` | Path to world you want to use forthe simulation. |