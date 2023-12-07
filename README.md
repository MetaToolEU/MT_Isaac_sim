## Overview

🤖 Simulate and control two Universal Robots (UR) arms concurrently using NVIDIA Isaac Sim. This project provides a digital twin environment for realistic robotic simulation and testing control algorithms.

![Isaac_sim_real_control](https://github.com/MetaToolEU/MT_Isaac_sim/assets/28174056/ea133980-3dd8-4deb-8aa1-991018188275)


## Features

- **Dual UR Arm Simulation**: Simulate UR3e arms simultaneously.
- **Real-Time Digital Twin Control**: Implement real-time control using UR RTDE interface.
- **Collision Avoidance**: Integrate collision detection for safe operations.
- **Path Planning**: Develop optimized trajectories using Isaac Sim extensions.

### Prerequisites
- [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)
- Python 3.7 or later
- [UR RTDE](https://sdurobotics.gitlab.io/ur_rtde/)


## Getting Started
1. Clone the repository:

   ```bash
   git clone https://github.com/MetaToolEU/MT_Isaac_sim.git
   ```
   
2. Add an alias to Isaac Sim’s python in your bashrc file:
   ```bash  
   echo "alias omni_python='~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh'" >> ~/.bashrc
   ```
   
3. Update your bash environment to include the new alias:
   ```bash 
   source ~/.bashrc
   ```
   
4.   Run the Python script with Isaac Sim:
   Replace "yyy.yyy.yyy.yyy" with the actual IP addresses of your first and second robots.
   ```bash 
   omni_python isaac_rtde_dual_arm.py --robot-ip "yyy.yyy.yyy.yyy" --robot-ip2 "yyy.yyy.yyy.yyy"
   ```
