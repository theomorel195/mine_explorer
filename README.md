# Mine Explorer Project 🛠️💣

![ROS 2](https://img.shields.io/badge/ROS2-Humble-green)

## 🌟 Overview
**Mine Explorer** is an advanced autonomous robotic solution designed for landmine detection and neutralization in hazardous environments. The project utilizes a mobile manipulator configuration—combining the ruggedness of a **Clearpath Husky** base with the precision of a **Universal Robots UR5e** arm.

The system is equipped with a multi-modal perception stack (LiDAR, RGB-D Cameras, Sonar Array) to navigate complex terrains and interact safely with detected objects.



---

## 🏗️ System Architecture
The project is organized into modular "Meta-Packages" to ensure scalability and ease of maintenance:

### 1. [arm_mine_explorer](./arm_mine_explorer)
Everything related to the **UR5e Robotic Arm**.
* Description (URDF/Xacro), Simulation (Gazebo), and future Hardware Interfaces & MoveIt 2 configs.

### 2. [base_mine_explorer](./base_mine_explorer)
Focuses on the **Husky Mobile Base**.
* Kinematics, wheel controllers, and Gazebo physical parameters.

### 3. [sensors_mine_explorer](./sensors_mine_explorer)
A standalone library for **Perception Hardware**.
* High-fidelity models for LiDAR (Velodyne), Cameras (RealSense), and Sonars with their respective Gazebo plugins.

### 4. [mine_explorer](./mine_explorer)
The **Master Integration** layer.
* Global robot assembly, top-level Gazebo worlds, and soon: SLAM, Navigation, and EKF Localization.


---

## 🚀 Getting Started

### Prerequisites
* **ROS 2** (Humble recommended)
* **Gazebo** (Classic)
* **ros2_control** & **ros2_controllers** packages

### Installation
```bash
# Create and navigate to your workspace
mkdir -p ~/mine_explorer_ws/src
cd ~/mine_explorer_ws/src

# Clone the project
git clone -b humble <your-repository-url> .

# Install dependencies
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Commands

| Task | Command |
| :--- | :--- |
| Visualize Full Roobt | `ros2 launch mine_explorer_description display.launch.py` |
| Launch Simulation | `ros2 launch mine_explorer_gazebo gazebo.launch.py` |
| Test Sensors | `ros2 launch sensros_mine_explorer_description <sensor>_display.launch.py` |

---

## Road Map

The development of **Mine Explorer** is divided into five strategic phases, moving from structural definition to autonomous mission execution.

### 🟢 Phase 1: Robot Description & Simulation
- [x] **Modular URDF/Xacro**: Integrated assembly of Husky base, UR5e arm, and multi-sensor suite.
- [x] **Sensor Library**: Standalone Gazebo plugins and descriptions for LiDAR, RGB-D cameras, and Sonars.
- [x] **Physics Integration**: Gazebo simulation setup with surface friction and gravity parameters.
- [x] **Basic Control**: Implementation of `joint_state_publisher` and `diff_drive_controller` for manual teleoperation.

### 🟡 Phase 2: Localization & Mapping
- [ ] **Sensor Fusion**: Implementation of `robot_localization` (EKF) to fuse IMU and Wheel Encoders.
- [ ] **SLAM Integration**: Deployment of `slam_toolbox` or `Cartographer` for real-time environment mapping.
- [ ] **Perception Filtering**: Pointcloud cleaning to remove "self-hits" (robot body) from laser scans.

### 🟠 Phase 3: Autonomous Navigation
- [ ] **Nav2 Stack**: Configuration of Costmaps (Global/Local) and Path Planners (Smac/RPP).
- [ ] **Obstacle Avoidance**: Dynamic avoidance using 360° sonar coverage for close-range safety.
- [ ] **Exploration Algorithms**: Implementation of frontier exploration to map unknown mine zones automatically.

### 🔵 Phase 4: Manipulation & Detection
- [ ] **MoveIt 2 Integration**: Motion planning for the UR5e arm with collision awareness.
- [ ] **Vision-Based Detection**: Landmine identification using RealSense D435/D455 data.
- [ ] **Coordinated Tasking**: Automated arm deployment upon detection of a target.

---

## 🤝 Contributing

This is an ongoing project. For any architectural suggestions or bug reports, please open an issue or submit a pull request.

**Author:** Théo Morel
**Project Link:** xxx