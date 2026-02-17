# arm_mine_explorer_description

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This package contains the **URDF/Xacro description** for the **Universal Robots UR5e** integrated into the Mine Explorer project. This robotic arm is a core component of the system, designed to handle landmine detection and neutralization operations in hazardous environments.

Currently, the package provides the base kinematic chain of the UR5e. Support for specific end-effectors (mine detection tools or grippers) is planned for future integration.


## Package Structure
```text
arm_mine_explorer_description/
├── config/             # Kinematics, joint limits, and physical parameters
├── launch/             # Launch files for visualization and state publishing
├── rviz/               # Pre-configured RViz settings (.rviz files)
├── urdf/               # Xacro files defining the arm and its macro
│   └── arm/            # Specific UR5e xacro components and definitions
├── CMakeLists.txt      # Build configuration
└── package.xml         # Package metadata and dependencies
```

## Usage

### Visualization in RViz

To verify the arm's kinematic chain, visual meshes, and joint transformations, you can use the provided display launch file. This is the best way to ensure the URDF is correctly parsed and all frames (TFs) are properly aligned.

**To launch the visualization:**
```bash
ros2 launch arm_mine_explorer_description display.launch.py
```

### Integration in a Global Robot URDF

This package is built to be modular. You can integrate the UR5e arm onto any mobile platform (such as a Husky) by including and calling the macro in your main robot description file.

**Example Xacro Integration**

In your top-level `.urdf.xacro` file, you can instantiate the arm as follows:

```xml
<xacro:include filename="$(find arm_mine_explorer_description)/urdf/arm/arm.xacro"/>

<xacro:arg name="name_arm" default="ArmMineExplorerSystem"/>
<xacro:arg name="prefix_arm" default="arm_"/>
<xacro:arg name="use_sim" default="true"/>

<xacro:arm_mine_explorer
    name="$(arg name_arm)"
    prefix="$(arg prefix_arm)"
    parent="top_plate_link"
    use_sim="$(arg use_sim)"
    joint_limits_parameters_file="$(find arm_mine_explorer_description)/config/joint_limits.yaml"
    kinematics_parameters_file="$(find arm_mine_explorer_description)/config/default_kinematics.yaml"
    physical_parameters_file="$(find arm_mine_explorer_description)/config/physical_parameters.yaml"
    visual_parameters_file="$(find arm_mine_explorer_description)/config/visual_parameters.yaml"
    safety_limits="true"
    safety_pos_margin="0.15"
    safety_k_position="20">
    <origin xyz="-0.25 0 0" rpy="0 0 3.1415" />
</xacro:arm_mine_explorer>
```

## Key Features & Arguments

This package is designed for high flexibility, allowing you to tune the arm's behavior and safety constraints directly from the launch command or by passing parameters to the Xacro macro.

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

## Testing

This package ensures the robot description is valid and follows ROS 2 coding standards. The test suite includes:

* **URDF Validation**: Checks if the Xacro files compile correctly into a valid URDF and that the kinematic tree is consistent.
* **Static Analysis**: Verifies that the Python launch files and configuration files follow PEP8 and ROS 2 style guidelines (using `flake8`, `pep257`, and `xmllint`).

### Run the tests
Execute the following command in your workspace:
```bash
colcon test --packages-select arm_mine_explorer_description --event-handlers console_cohesion+
```

### Key checked performed

1. **Linter checks:** Ensuring no unused imports or syntax errors in launch files.
2. **URDF Parsing:** Validating the model with `check_urdf` or by ensuring the `robot_state_publisher`can parse the description without warnings.
