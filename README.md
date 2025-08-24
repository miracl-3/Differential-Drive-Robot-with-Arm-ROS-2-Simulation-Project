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
---

## 🚀 How to Build & Run

1. **Clone this repository** into your ROS 2 workspace (e.g. `~/ros2_ws/src`):

    ```bash
    $ mkdir ros2_ws
    $ cd ros2_ws
    $ mkdir src
    $ git clone https://github.com/miracl-3/Differential-Drive-Robot-with-Arm-ROS-2-Simulation-Project.git
    
    ```

2. **Build the workspace**:

    ```bash
    $ cd ../
    $ colcon build 
    ```

3. **Source the workspace** (every new terminal):
    ```bash
    $ source ros2_ws/install/setup.bash
    ```
4. **Installing TF Tools**:
    ```bash
    $ sudo apt install ros-jazzy-tf2-tools
    ```
5. **Installing Gazebo**:
    ```bash
    $ sudo apt install ros-jazzy-ros-gz
    # Launching Gazebo
    $ gz sim
    ```
6. **Launch the simulation** (choose XML or Python launch file):

    ```bash
    # XML launch file
    ros2 launch simulation_final_project robot_simulation.launch.xml
    
    # or Python launch file
    ros2 launch simulation_final_project robot_simulation.launch.py
    ```

   This will start:
   - `robot_state_publisher` (publishes TF & robot_description)  
   - Gazebo with the provided world (`sample_world.sdf`)  
   - RViz2 with the preconfigured setup  
   - ROS–Gazebo bridge (`ros_gz_bridge`)
7. **Testing functionality and plugins**:
   ```bash
    # Control differential drive robot movement via teleop_twist_keyboard package
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    # Send the data to the Arm's Robot angle via an example command below:
    $ ros2 topic pub -1 /joint0/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"
    $ ros2 topic pub -1 /joint1/cmd_pos std_msgs/msg/Float64 "{data: 1.5}"
    ```  
---

