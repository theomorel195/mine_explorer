# base_mine_explorer_description

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This package contains the **URDF/Xacro description** for the **Husky Mobile Robot** integrated into the Mine Explorer project. This mobile robot is a core component of the system, designed to handle landmine detection and neutralization operations in hazardous environments.


## Package Structure
```text
base_mine_explorer_description/
├── launch/             # Launch files for visualization and state publishing
├── rviz/               # Pre-configured RViz settings (.rviz files)
├── urdf/               # Xacro files defining the robot and its macro
│   └── mobile_base/    # Specific Husky xacro components and definitions
├── CMakeLists.txt      # Build configuration
└── package.xml         # Package metadata and dependencies
```

## Usage

### Visualization in RViz

To verify the robot's visual meshes, you can use the provided display launch file. This is the best way to ensure the URDF is correctly parsed and all frames (TFs) are properly aligned.

**To launch the visualization:**
```bash
ros2 launch base_mine_explorer_description display.launch.py
```

### Integration in a Global Robot URDF

This package is built to be modular. You can integrate onto the Husky any robotic arm (such as a UR5e) by including and calling the macro in your main robot description file.

**Example Xacro Integration**

In your top-level `.urdf.xacro` file, you can instantiate the robot as follows:

```xml
<xacro:include filename="$(find base_mine_explorer_description)/urdf/mobile_base/mobile_base.xacro"/>

<xacro:arg name="name_base" default="MobileBaseMineExplorerSystem">
<xacro:arg name="prefix_base" default=""/>
<xacro:arg name="use_sim" default="true"/>

<xacro:base_mine_explorer
    name="$(arg name_base)"
    prefix="$(arg prefix_base)"
    use_sim="$(arg use_sim)"/>
</xacro:base_mine_explorer>
```

## Key Features & Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `name` | `MobileBaseMineExplorerSystem` | Name of the hardware system. |
| `prefix` | `""` | Prefix for all joint and link names (essential for avoiding name conflicts). |
| `use_sim` | `true` | Toggle for Gazebo simulation compatibility and specific hardware plugins. |