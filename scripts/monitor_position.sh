
#!/bin/bash

echo "📊 Robot Position Monitor 📊"
echo "=========================="
echo "Press Ctrl+C to exit"
echo ""

while true; do
    clear
    echo "📍 Robot Position (Updated every 1 second):"
    echo "--------------------------------------"
    docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
        rostopic echo -n 1 /odom/pose/pose | grep -A 10 position"
    
    echo ""
    echo "🚀 Velocity:"
    docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
        rostopic echo -n 1 /cmd_vel | head -n 10"
    
    echo ""
    echo "Press Ctrl+C to exit"
    sleep 1
done
