now update this ReadME.md

# gazebo-ros-docker in Linux
 A simulation module in Gazebo environment using Docker ROS

## Overview

This project provides a containerized ROS-Gazebo simulation environment, allowing you to run robot simulations without installing ROS or Gazebo directly on your host machine. It supports both Linux and Mac systems, with Linux providing superior visualization capabilities.

## System Requirements

- Docker and Docker Compose installed
- Linux

### Recommended Linux Distributions

- **Ubuntu 20.04 LTS** (Focal Fossa) - Best choice for ROS Noetic
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Debian 11 (Bullseye)
- Linux Mint 20.3 (Una)

## Project Structure
gazebo-ros-docker-linux/
├── docker-compose.yml          # Docker configuration
├── models/                     # Custom Gazebo models
├── worlds/                     # Gazebo world files
├── catkin_ws/                  # ROS workspace
│   └── src/                    # ROS packages
│       ├── my_robot_description/   # Robot URDF and models
│       └── my_robot_gazebo/        # Simulation configuration
└── scripts/                    # Helper scripts
    ├── setup_xquartz.sh        # XQuartz configuration
    ├── run_simulation.sh       # Run the core simulation
    ├── teleop.sh               # Control the robot
    ├── teleop_with_feedback.sh # Control with position feedback
    ├── monitor_position.sh     # Real-time position monitoring
    ├── record_data.sh          # Record simulation data
    ├── create_viz.sh           # Create web visualization
    └── full_test.sh            # Full environment setup


## Setup Instructions
Initial Setup

1. Clone this repository:
```bash
git clone https://github.com/hanifr/gazebo-ros-docker-linux.git
cd gazebo-ros-docker-linux
```
2. Configure X11 for Docker::
```bash
./scripts/setup_x11.sh
```

3. Start the Docker containers
```bash
docker-compose up -d
```
4. Run the full environment setup:
```bash
./scripts/full_test.sh
```

## Running the Simulation
After the initial setup, you can run the simulation:
```bash
./scripts/run_simulation.sh
```

## Controlling the Robot
You have multiple options to control and monitor the robot:
Basic Control
```bash
./scripts/teleop.sh
```
Use keyboard keys: W (forward), A (left), S (backward), D (right), Space (stop), Q (quit)


```

This provides a simple 2D visualization you can control with WASD keys

Visualization
On Linux
Gazebo visualization works directly:
```bash
./scripts/start_gazebo_client.sh
```

RVIZ visualization is also available:
```bash
./scripts/run_rviz.sh
```

Custom Robot Models
To use your own robot models:

Place URDF files in catkin_ws/src/my_robot_description/urdf/
Update the robot name in run_simulation.sh if needed


## Stopping the Simulation

```bash
docker-compose down
```

## Advanced Usage
For more advanced needs, check out the scripts/full_test.sh file which demonstrates the complete setup process and can be modified for your specific requirements.

Troubleshooting
Linux Issues

Ensure X11 permissions are set: xhost +local:docker
Check if containers can access X11: ls -la /tmp/.X11-unix/

## Install docker on Linux

## Installation Steps for Linux (Ubuntu 20.04 LTS)

1. Install Docker and Docker Compose:
   ```bash
   sudo apt update
   sudo apt install -y apt-transport-https ca-certificates curl software-properties-common
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
   sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
   sudo apt update
   sudo apt install -y docker-ce docker-ce-cli containerd.io
   
   sudo usermod -aG docker $USER
   
   sudo curl -L "https://github.com/docker/compose/releases/download/v2.5.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
   sudo chmod +x /usr/local/bin/docker-compose
   ```
2. Run the simulation:
   ```bash
    ./scripts/run_simulation.sh
    ./scripts/start_gazebo_client.sh  # In a separate terminal
    ./scripts/teleop.sh               # In another separate terminal
   ```