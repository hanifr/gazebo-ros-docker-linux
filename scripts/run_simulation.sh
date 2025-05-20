#!/bin/bash

echo "🚀 Running ROS-Gazebo Simulation"
echo "==============================="

# Start containers if not running
if [ "$(docker ps -q -f name=gazebo_sim)" == "" ] || [ "$(docker ps -q -f name=ros_control)" == "" ]; then
    echo "🔄 Starting Docker containers..."
    docker-compose up -d
    echo "⏳ Waiting for containers to start..."
    sleep 3
fi

# Check if ROS-Gazebo bridge is running
if ! docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && rosnode list | grep -q /gazebo"; then
    echo "🌉 Starting ROS-Gazebo bridge..."
    docker-compose exec -d ros bash -c "source /opt/ros/noetic/setup.bash && \
        export GAZEBO_MASTER_URI=http://gazebo:11345 && \
        rosrun gazebo_ros gzserver"
    echo "⏳ Waiting for bridge to initialize..."
    sleep 3
fi

# Spawn robot model
echo "🤖 Spawning robot model..."
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    rosrun gazebo_ros spawn_model -urdf \
    -file /catkin_ws/src/my_robot_description/urdf/robot.urdf \
    -model differential_drive_robot -z 0.1 && \
    echo '✅ Robot spawned successfully!'"

echo "📋 ROS topics available:"
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && rostopic list | grep -v /rosout"

echo ""
echo "✅ Simulation is running!"
echo "To see the simulation GUI, run: ./scripts/run_gazebo_gui.sh"
echo "To control the robot, use: ./scripts/teleop.sh"
echo "To stop the simulation, run: docker-compose down"
