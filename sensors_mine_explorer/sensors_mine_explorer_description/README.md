# sensors_mine_explorer_description

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)

## Overview
This package is a modular sensor library for the Mine Explorer project. It contains the **URDF/Xacro descriptions**, **3D meshes**, and **Gazebo plugins** for all the perception hardware used in the mine exploration missions.

The package is designed to be standalone, allowing each sensor to be tested individually in a simulation environment before integration into the main robot model.


## Package Structure
```text
sensors_mine_explorer_description/
├── launch/             # Individual sensor testing launch files
├── meshes/             # 3D models for visualization (.stl, .dae)
│   ├── camera/         # Intel RealSense D435/D455 meshes
│   └── lidar/          # Velodyne VLP-16/HDL-32E meshes
├── rviz/               # Pre-configured sensor visualization settings
├── urdf/               # Xacro definitions and Gazebo plugins
│   ├── camera/         # RGB-D camera macros
│   ├── imu/            # Inertial Measurement Unit definition
│   ├── lidar/          # Laser scanner macros (2D and 3D)
│   └── sonar/          # Ultrasonic range sensor array
|── worlds/             # Testing environments with obstacles
├── CMakeLists.txt
└── package.xml
```

## Supported Hardware

| Sensor Type | Models | Features |
| :--- | :--- | :--- |
| **LiDAR** | Velodyne VLP-16, HDL-32E | 3D Point Cloud, 360° FOV. |
| **Depth Camera** | RealSense D435, D455 | RGB-D Streams, PointCloud2 output. |
| **Acoustic** | MatBotix Ultrasonic | Range data for close obstacle avoidance. |
| **Inertial** | Generic IMU | Orientation and acceleration data. |


## Usage

### Individual Sensor Testing

You can launch a standalone Gazebo simulation for each sensor to verify its field of view (FOV) and data output in RViz.

For LiDARs : 

```bash
ros2 launch sensors_mine_explorer_description lidar_vlp16_display.launch.py
# OR
ros2 launch sensors_mine_explorer_description lidar_hdl32e_display.launch.py
```

For Depth Cameras :

```bash
ros2 launch sensors_mine_explorer_description camera_d435_display.launch.py
# OR
ros2 launch sensors_mine_explorer_description camera_d455_display.launch.py
```

## Integration Guide

To add a sensor to your robot, include the specific Xacro macro in your main URDF file :

```xml
<xacro:include filename="$(find sensors_mine_explorer_description)/urdf/camera/d455.urdf.xacro" />

<xacro:sensor_d455 parent="base_link" prefix="front_">
  <origin xyz="0.4 0 0.2" rpy="0 0 0" />
</xacro:sensor_d455>
```

## Simulation Details 

Each sensor includes a Gazebo plugin (`̀libgazebo_ros_ray_sensor.so`, `̀libgazebo_ros_camera.so`, etc.) to simulate realistic data streams. The parameters (noise, update rate, resolution) can be adjusted in the respective `.urdf.xacro` files. 

## Testing 

This package uses `launch_testing` to verify the simulation's health. The tests check for:
* **Static Analysis**: PEP8 compliance, docstring presence, and XML validity.
* **Integration**: Successful launch of Gazebo and controllers, and active data publishing on `/joint_states`.

### Run all tests
To run the tests, use the following command in your workspace:
```bash
colcon test --packages-select sensors_mine_explorer_description --event-handlers console_cohesion+
```

### View test results

After running the tests, you can see a detailed summary with :
```bash
colcon test-result --all --verbose
```