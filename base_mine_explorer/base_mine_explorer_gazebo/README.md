# base_mine_explorer_gazebo

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This package provides the simulation environment for the Mine Explorer mobile base. It integrates the Husky robot description into **Gazebo**, configuring the physics, world environments, and the `ros2_control` loop necessary for simulated movements.

It acts as the bridge between the static URDF description and the dynamic simulation, allowing for testing of navigation without physical hardware.


## Package Structure
```text
base_mine_explorer_gazebo/
├── config/             # Hardware interface and gazebo_ros2_control parameters
├── launch/             # Launch files to spawn the robot and start controllers
├── worlds/             # Custom Gazebo environments (e.g., mine-like scenarios)
├── CMakeLists.txt
└── package.xml
```

## Usage

### Launching the simulation

To spawn the mobile robot in an empty Gazebo world with the default controllers (Joint State Broadcaster and Base_Controller), run:

```bash
ros2 launch base_mine_explorer_gazebo gazebo.launch.py
```

### Key Features
- **Automated Spawning:** Automatically processes Xacro and spawns the robot entity into the simulation.
- **Controller Sequencing:** Uses event handlers to ensure the `joint_state_broadcaster` is active before spawning the `base_controller`, preventing initization errors.
- **World Selection:** Support for custom simulation environments tailored for mine detection tasks.


## Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `name` | `MobileBaseMineExplorerSystem` | Name of the hardware system. |
| `prefix` | `""` | Prefix for all joint and link names (essential for avoiding name conflicts). |
| `use_sim` | `true` | Toggle for Gazebo simulation compatibility and specific hardware plugins. |
| `world` | `empty_world.model` | Path to world you want to use forthe simulation. |