now update this ReadME.md

# gazebo-ros-docker
 A simulation module in Gazebo environment using Docker ROS

gazebo-ros-docker/
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
git clone https://github.com/hanifr/gazebo-ros-docker.git
cd gazebo-ros-docker
```
2. Configure XQuartz for Docker compatibility:
```bash
# Install XQuartz if needed
brew install --cask xquartz

# Configure XQuartz
defaults write org.xquartz.X11 app_to_run /usr/bin/true
defaults write org.xquartz.X11 no_auth 1
defaults write org.xquartz.X11 nolisten_tcp 0

# Start XQuartz and allow connections
open -a XQuartz
xhost + localhost
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

## Advanced Control with Position Feedback
```bash
./scripts/teleop_with_feedback.sh
```
Same as above, but press F to see current position data

Monitor Position in Real-time
```bash
./scripts/monitor_position.sh
```

Displays continuously updating position data
Record Position Data
```bash
./scripts/record_data.sh
```

Records position data to a CSV file for later analysis
Visualization Options
Due to OpenGL challenges with Docker on Mac, several visualization options are available:

1. Web-based Visualization
```bash
# Create the HTML visualization
./scripts/create_viz.sh

# Open in Chrome
open -a "Google Chrome" robot_viz.html
```

This provides a simple 2D visualization you can control with WASD keys

2. Run Gazebo GUI (may not work on all Mac systems)
```bash
# Try to run Gazebo GUI (may fail on Mac)
./scripts/start_gazebo_client.sh
```

## Troubleshooting
XQuartz Issues

Ensure XQuartz is running: ps aux | grep XQuartz
Check permission configuration: xhost + localhost
Restart XQuartz if visualization fails

Docker Issues

Check containers are running: docker ps
View container logs: docker-compose logs
Restart containers: docker-compose down && docker-compose up -d

Gazebo Visualization Fails
This is a common issue on Mac due to OpenGL/GLX limitations with Docker:

Use the HTML visualization as an alternative
Try the teleop with feedback for headless operation
Use the position monitoring script to track robot state

Custom Robot Models
To use your own robot models:

Place URDF files in catkin_ws/src/my_robot_description/urdf/
Update the robot name in run_simulation.sh if needed
Restart the simulation

## Stopping the Simulation

```bash
docker-compose down
```

## Advanced Usage
For more advanced needs, check out the scripts/full_test.sh file which demonstrates the complete setup process and can be modified for your specific requirements.