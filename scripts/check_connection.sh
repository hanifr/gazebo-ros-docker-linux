# Update scripts/check_connection.sh
#!/bin/bash
echo "Checking ROS topics..."
docker-compose exec -T ros bash -c "source /opt/ros/noetic/setup.bash && rostopic list"

echo -e "\nChecking if ROS-Gazebo bridge is working..."
docker-compose exec -T ros bash -c "source /opt/ros/noetic/setup.bash && \
    rostopic list | grep gazebo || echo 'No gazebo topics found in ROS, bridge may not be running'"

echo -e "\nChecking Gazebo topics (will ignore Boost error)..."
docker-compose exec -T gazebo bash -c "gz topic -l || true"