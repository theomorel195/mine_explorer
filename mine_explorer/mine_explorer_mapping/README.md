# mine_explorer_mapping

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

This package provides the mapping capabilities for the **Mine Explorer** robot. It currently processes 3D LiDAR data into 2D laser scans to perform robust SLAM in underground environments, with a roadmap toward full 3D SLAM implementation.

---

## Package Structure

* **`config/`**: Contains the 2D SLAM configuration parameters (e.g., `slam_toolbox.yaml`).
* **`launch/`**: Includes the main mapping launch files that orchestrate:
    * **Pointcloud to LaserScan**: Conversion of 3D LiDAR pointclouds into 2D `/scan` data.
    * **SLAM Execution**: Starting the SLAM node (Asynchronous mode) with tuned parameters.

---

## Getting Started

### Prerequisites
You will need the `slam_toolbox` and `pointcloud_to_laserscan` packages:
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox 
sudo apt install ros-humble-pointcloud-to-laserscan
sudo apt install ros-humble-nav2-map-server
```

### Launching the Mapping Pipeline 

To start the scan conversion and the 2D SLAM simultamiously :
```bash
ros2 launch mine_explorer_mapping slam_2d.launch.py
```

## Features

1. **3D to 2D Data Conversion**

Since the robot operated in a 3D environment, this package extracts a 2D slice from the 3D LiDAR data.
* **Optimized FOV:** The scan is configured for a **270° FOV** to ensure the side-wall visibility, which is critical for yaw stability in narrow environment. (Soon to be 360° scan)
* **Density:** An angular increment of **0.5° (0.0087 rad)** is used to balance processing speed and geometric precision.

2. **2D SLAM Configuration**

* **Solver:** Uses ̀`CeresSolver` for high-precision scan matching. 
* **Stability:** Custom variance penalties are applied to prevent "jumps" in the map when the robot performs tight turns.
* **Loop Closure:** Enabled to correct long-term drift.

## Current Calibration

* **Scan Resolution:** Set to 0.05m for a perfect balance between map detail and real-time performance.
* **TF Synchronization:** Configured with a `transform_timeout`of **0.5s** to handle the computational load of high-density Lidar data in Gazebo.

## Road Map

* **Map Management:** Integration of nodes for automated map saving and loading.
* **3D SLAM:** Implementation of a full 3D SLAM solution to capture the volumetric complexity environment.
* **Multi-Level Mapping:** Support for vertical shafts.

## Testing 

This package uses `launch_testing` to verify the simulation's health. The tests check for:
* **Static Analysis**: PEP8 compliance, docstring presence, and XML validity.
* **Integration**: Successful launch of Gazebo simulation and SLAM, and active data publishing on `/sensors/lidar/scan_2d` & `/map`.

### Run all tests
To run the tests, use the following command in your workspace:
```bash
colcon test --packages-select mine_explorer_mapping --event-handlers console_cohesion+
```

### View test results

After running the tests, you can see a detailed summary with :
```bash
colcon test-result --all --verbose
```