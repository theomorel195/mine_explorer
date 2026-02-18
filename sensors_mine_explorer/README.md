# sensors_mine_explorer

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## Overview
This folder contains all the packages related to the perception system of the Mine Explorer robot. It acts as a modular sensing suite that handles everything from physical sensor definitions (URDF) to future data filtering.

The goal is to provide a clean and unified interface for the robot's "senses" (LiDAR, Cameras, Sonars, and IMU).


## Repository Structure

| Package | Status | Description |
| :--- | :--- | :--- |
| `sensors_mine_explorer_description` | ✅ Ready | URDF/Xacro models, 3D meshes, and Gazebo simulation plugins for all sensors. |
| `sensors_mine_explorer_processing` | 📅 Planned | Future package for point cloud filtering (PCL), laser scan merging, and data synchronization. |


## Sensors Included

The suite currently supports and provides models for:
* **3D LiDARs**: Velodyne VLP-16 and HDL-32E.
* **Vision**: Intel RealSense D435 and D455 (RGB-D).
* **Ultrasonic**: MaxBotix Sonar array (6 units for 360° coverage).
* **Inertial**: Standard IMU for orientation and gravity vector sensing.

## Future Data Processing (Planned)
To ensure high-quality data for the Navigation and SLAM stacks, upcoming packages in this folder will focus on:
1.  **Point Cloud Filtering**: Removing noise and outliers from LiDAR data in dusty mine environments.
2.  **Scan Merging**: Combining multiple sonar ranges into a single costmap-ready laser scan.
3.  **Visual Odometry**: Leveraging RealSense depth data to improve positioning accuracy.



## Usage
To test individual sensors and visualize their raw data output, please refer to the documentation inside the `sensors_mine_explorer_description` package.

```bash
# Example: Testing the VLP-16 LiDAR
ros2 launch sensors_mine_explorer_description lidar_vlp16_display.launch.py
```

## Quality Assurance & Testing

Each package in this repository includes its own suite of tests (Linters and Integration tests). 
You can run the full test suite for the entire project to ensure everything is correctly installed and configured:

```bash
# Run tests for all packages in the project
colcon test --packages-select sensors_mine_explorer_description --event-handlers console_cohesion+

# Check the overall summary
colcon test-result --all --verbose
```