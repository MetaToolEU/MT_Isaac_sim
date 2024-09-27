## Overview

🤖 Simulate and control two Universal Robots (UR) arms concurrently using NVIDIA Isaac Sim. This project provides a digital twin environment for realistic robotic simulation and testing control algorithms.

![Isaac_sim_real_control](https://github.com/MetaToolEU/MT_Isaac_sim/assets/28174056/ea133980-3dd8-4deb-8aa1-991018188275)
<p align="center"><b>Model Predictive Control (MPC) to reach target</b></p>
<p align="center">
  <img src="https://github.com/MetaToolEU/MT_Isaac_sim//assets/28174056/d44a65a1-64d7-4133-a371-0c3a1c28e209" alt="gripper_mpc">
</p>


https://github.com/MetaToolEU/MT_Isaac_sim/assets/28174056/f53435bb-87a1-4ff9-9b28-50b225b56b98


## Features

- **Dual UR Arm Simulation**: Simulate UR3e arms simultaneously.
- **Real-Time Digital Twin Control**: Implement real-time control using UR RTDE interface.
- **Collision Avoidance**: Integrate collision detection for safe operations.
- **Path Planning**: Develop optimized trajectories using Isaac Sim extensions.


## System Requirements

To run the Docker-based simulation tutorial, ensure your system meets the following requirements:

- **Operating System**: Ubuntu 20.04 LTS or later (preferred); Docker is also available for Windows and macOS.
- **Docker**: Docker Engine and Docker Compose installed (version 1.27+ recommended).
- **CPU**: Multi-core processor (e.g., Intel Core i7 or equivalent).
- **GPU**: NVIDIA GPU with CUDA support (e.g., RTX 2060 or higher) and driver version **525.60.11** or later.
- **RAM**: Minimum 8 GB (16 GB recommended).
- **Storage**: At least 50 GB of free disk space for Docker images and data.
- **Network**: Stable internet connection for downloading Docker images and dependencies.

**Important**: Ensure Docker is properly configured to utilize the GPU if required and that your system has adequate resources for smooth operation. Verify GPU availability with the following command:

```bash
docker run --rm --gpus all nvidia/cuda:11.2.2-base-ubuntu20.04 nvidia-smi
```
### Additional Requirements
- [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html)
- Python 3.7 or later
- [UR RTDE](https://sdurobotics.gitlab.io/ur_rtde/)
- Curobo v0.6.2  (The project is developed and tested on Curobo version 0.6.2)
## Setup using Docker (RECOMENDED)
1. **Setup NVIDIA GPU Cloud (NGC) Environment:**

   Ensure you have an NGC API key. Follow the instructions [here](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key) to generate one.

   Log in to NGC using Docker:

   ```bash
   docker login nvcr.io
   Username: $oauthtoken
   Password: [Your NGC API Key]
2. **Install and configure your omniverse Nucleus user:**
   
   Ensure that you have [Omniverse Launcher](https://www.nvidia.com/es-es/omniverse/download/) installed
   
   After installing Omniverse Launcher, configure your local [Nucleus Server](https://docs.omniverse.nvidia.com/nucleus/latest/workstation/installation.html)
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
   
4.  To launch dual robot digital twin run the Python script with Isaac Sim:
   Replace "yyy.yyy.yyy.yyy" with the actual IP addresses of your first and second robots.
   ```bash 
   omni_python isaac_rtde_dual_arm.py --robot-ip "yyy.yyy.yyy.yyy" --robot-ip2 "yyy.yyy.yyy.yyy"
   ```
<p align="center"><b>Curobo motion generation reacher demo</b></p>
<p align="center">
  <img src="https://github.com/MetaToolEU/MT_Isaac_sim/assets/28174056/f562b6ce-31e6-4a04-9e00-170197926f91" alt="Curobo">
</p>

5. To launch the Curobo example run :
   ```bash
   omni_python motion_gen_reacher.py
   ```
