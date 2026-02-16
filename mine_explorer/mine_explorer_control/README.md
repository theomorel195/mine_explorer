# mine_explorer_control

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

This package handles the localization and state estimation for the **Mine Explorer** robot. It leverages an Extended Kalman Filter (EKF) to fuse odometry and IMU data, providing a robust pose estimate for navigation and mapping in underground environments.

---

## Package Structure

* **`config/`**: Contains `ekf.yaml`, the main configuration file for the EKF node (parameters for IMU, odometry, and process noise covariance).
* **`launch/`**: Contains `localization.launch.py` to start the robot state estimation.
* **`scripts/`**: Includes utility scripts like `compute_variance.py` for sensor noise analysis.
* **`CMakeLists.txt` & `package.xml`**: Standard ROS 2 build and dependency definitions.

---

## Getting Started

### Prerequisites
Ensure you have the `robot_localization` package installed:
```bash
sudo apt update
sudo apt install ros-<distro>-robot-localization
```

### Launching Localization

To start the EKF Node and begin sensor fusin : 

```bash
ros2 launch mine_explorer_control localization.launch.py
```

## Features

1. **Extented Kalman Filter (EKF)**

The package uses the `robot_localization` stack to fuse :
* **Wheel Odometry:** From the `diff_drive_controller` (`base_controller`).
* **IMU Data:** 6-axis data (angular velocity and linear acceleration) from `/sensors/imu/data`.

2. **Sensor Calibration Tools**

Inside the ̀`scripts/` folder, the `compute_variance.py` tool allow you to:
* Collect raw data from sensor topics.
* Calculate the statistical variance for the ̀`covariance_diagonal`parameters required in the YAML configuration.
* Automate the tuning of the R Matrix (Measurement Noise).

## Technical Callibration Note

The odometry used in this package has been calibrated against simulation Ground Truth (GT) to ensure maximum precision:
* **Wheel Separation Multiplier:** Set to **1.23** in the controller configuration to ensure perfect yaw matching during rotations.
* **TF Stability:** `transform_timeout`and `publish_rate` are optimized to prevent "TF_OLD_DATA" warnings in high-load simulation scenarios.
