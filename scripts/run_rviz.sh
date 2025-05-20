
#!/bin/bash

echo "🖥️ Starting RVIZ visualization..."

# Install RVIZ if not already installed
docker-compose exec ros bash -c "apt-get update && apt-get install -y \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher"

# Create a basic RVIZ config
docker-compose exec ros bash -c "mkdir -p /catkin_ws/src/my_robot_description/config && \
if [ ! -f /catkin_ws/src/my_robot_description/config/robot.rviz ]; then
    # Create a basic RVIZ config file here
    # ...content omitted for brevity...
fi"

# Start joint state publishers
docker-compose exec -d ros bash -c "source /opt/ros/noetic/setup.bash && \
    rosrun robot_state_publisher robot_state_publisher"
docker-compose exec -d ros bash -c "source /opt/ros/noetic/setup.bash && \
    rosrun joint_state_publisher joint_state_publisher"

# Launch RVIZ
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    rosrun rviz rviz -d /catkin_ws/src/my_robot_description/config/robot.rviz"


