# Differential Drive Arm Robot – ROS 2 Simulation Project

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

- **ROS 2 Jazzy** (tested on Jazzy)  
- **colcon** build system  
- **Gazebo (Fortress or Garden)** with `ros_gz_sim`  
- Python 3.8+  

ROS 2 installation instructions: [ROS 2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)

---

## 📂 Repository Structure
    .
    ├── config                   # Compiled files (alternatively `dist`)
    ├── launch                    # Documentation files (alternatively `doc`)
    ├── urdf                     # Source files (alternatively `lib` or `app`)
    ├── worlds                    # Automated tests (alternatively `spec` or `tests`)
    ├── .gitignore                   # Tools and utilities
    ├── CMakeList.txt
    ├── README.md
    └── package.xml
