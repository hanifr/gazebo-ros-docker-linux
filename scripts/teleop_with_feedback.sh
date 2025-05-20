
#!/bin/bash

echo "🎮 Robot Teleop Control with Position Feedback 🎮"
echo "==========================================="
echo "Use keyboard to control the robot:"
echo "  w - forward"
echo "  s - backward"
echo "  a - turn left"
echo "  d - turn right"
echo "  space - stop"
echo "  f - show position feedback"
echo "  q - quit"
echo ""
echo "Press any key to start..."
read -n 1

# Function to get robot position
get_position() {
    docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
        rostopic echo -n 1 /odom | grep -A 6 position"
}

# Function to get robot orientation
get_orientation() {
    docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
        rostopic echo -n 1 /odom | grep -A 4 orientation"
}

while true; do
    echo -n "Command > "
    read -n 1 cmd
    echo ""
    
    case $cmd in
        w)
            echo "Moving forward"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        s)
            echo "Moving backward"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        a)
            echo "Turning left"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'"
            ;;
        d)
            echo "Turning right"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'"
            ;;
        " ")
            echo "Stopping robot"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        f)
            echo "📍 Current Position:"
            get_position
            echo "🧭 Current Orientation:"
            get_orientation
            ;;
        q)
            echo "Quitting"
            exit 0
            ;;
        *)
            echo "Unknown command: '$cmd'"
            ;;
    esac
done
