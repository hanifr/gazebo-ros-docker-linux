#!/bin/bash

echo "🚀 Starting the simulation with differential drive robot..."

# Ensure containers are running
docker-compose up -d

# Create necessary packages
./scripts/full_test.sh

# Spawn the new robot model with differential drive
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    rosrun gazebo_ros spawn_model -urdf \
    -file /catkin_ws/src/my_robot_description/urdf/robot.urdf \
    -model differential_drive_robot -z 0.1"

echo "✅ Robot spawned successfully!"
echo "To control the robot, run: ./scripts/teleop.sh"