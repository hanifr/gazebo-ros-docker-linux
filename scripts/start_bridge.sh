#!/bin/bash
echo "Starting ROS-Gazebo bridge..."
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    rosrun gazebo_ros gznode"