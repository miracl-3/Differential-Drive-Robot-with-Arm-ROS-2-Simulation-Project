# Differential Drive with a 2DOF Arm Robot – A ROS 2 Simulation Mini Project

This repository contains a **ROS 2 package** for simulating a mobile differential drive robot equipped with a simple robotic arm.  
The project integrates **URDF/Xacro robot description**, **Gazebo (ros_gz_sim)** for simulation, and **RViz2** for visualization.

---

## 📦 Features
- Differential drive mobile base  
- 2-DOF robotic arm (forearm + hand)  
- URDF/Xacro-based robot description  
- Gazebo world (`sample_world.sdf`)  
- RViz2 configuration for visualization  
- ROS 2 ↔ Gazebo bridge via `ros_gz_bridge`  

---

## 🛠️ Requirements

- **Ubuntu Noble 24.02** or Docker but I won't mention it here.
- **ROS 2 Jazzy**
- **GZ Harmonic (LTS)**
- **Rviz2**  

ROS 2 installation instructions: [ROS 2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)

---

## 📂 Repository Structure
    .
    ├── config/                   # Config files (RViz2, ROS-Gazebo bridge)
    ├── launch/                   # Launch files (XML and Python)
    ├── urdf/                     # Robot description (URDF/Xacro models)
    ├── worlds/                   # Gazebo world files
    ├── .gitignore                # Ignore build/log/install when pushing to GitHub
    ├── CMakeLists.txt            # CMake build script for the ROS 2 package
    ├── package.xml               # ROS 2 package manifest  
    └── README.md                 # Project documentation

