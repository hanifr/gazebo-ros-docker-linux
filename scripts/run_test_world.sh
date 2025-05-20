#!/bin/bash

echo "🚀 Starting Gazebo with test world..."
# Stop any running containers
docker-compose down

# Start Gazebo with test world
docker-compose up -d gazebo
# Modify the command to use the test world
docker-compose exec -d gazebo bash -c "pkill gzserver && gzserver /worlds/test_world.world"

# Start ROS container
docker-compose up -d ros

echo "⏳ Waiting for Gazebo to initialize..."
sleep 5

echo "🤖 Launching robot in test world..."
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    roslaunch my_robot_gazebo test_world.launch"

echo "🖥️ Starting Gazebo client..."
docker-compose exec gazebo gzclient