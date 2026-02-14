# mine_explorer_description

## Overview
This package is the **Top-Level Robot Description** for the Mine Explorer robot. It acts as the "master" assembly, integrating the mobile base, the robotic arm, and a full suite of perception sensors into a single unified URDF/Xacro model.

The robot is a customized **Clearpath Husky** platform equipped with a **Universal Robots UR5e** arm, specifically designed for autonomous landmine exploration and neutralization tasks.


## Hardware Integration

The global model integrates the following components:

* **Mobile Base:** Clearpath Husky (4-wheel differential drive).
* **Robotic Arm:** Universal Robots UR5e (6-DOF).
* **LiDAR:** Velodyne VLP-16 (configured for 2D/3D mapping).
* **Depth Cameras:** * 1x Intel RealSense D455 (Front-facing for navigation).
    * 1x Intel RealSense D435 (Arm-mounted for precision manipulation).
* **Ultrasonic Sensors:** 6x MaxBotix Sonars for close-range obstacle detection (360° coverage).
* **Navigation Sensors:** * Internal IMU for orientation tracking.
    * 4x Wheel Encoders (integrated into the Husky base) for odometry.

## Package Structure
```text
mine_explorer_description/
├── launch/             # Master launch files for visualization
├── rviz/               # RViz2 configuration files
├── urdf/
│   └── robot.urdf.xacro # Master Xacro file (Assembles Base + Arm + Sensors)
├── CMakeLists.txt
└── package.xml
```

## Usage

### Visualization in RViz

To see the fully assembled robot with all its sensors and the arm attached :
```bash
ros2 launch mine_explorer_description display.launch.py
```

This launch file will :
1. Parse the master `robot.urdf.xacro` file.
2. Publish the global robot state.
3. Open RViz with robot model.
4. Open the Joint State Publisher GUIto move both the Husky wheels and the UR5e joints.


## Xacro Architecture

The master URDF is built using a modular approach by including macros from sub-packages : 

```xml
<xacro:include filename="$(find base_mine_explorer_description)/urdf/husky_macro.urdf.xacro" />
<xacro:include filename="$(find arm_mine_explorer_description)/urdf/arm/arm.xacro"/>
<xacro:include filename="$(find sensors_mine_explorer_description)/urdf/lidar/velodyne_vlp16.xacro"/>
```

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


## Dependencies

This package requires the following sub-packages from the Mine Explorer workspace : 
* base_mine_explorer_description
* arm_mine_explorer_description
* sensors_mine_explorer_description