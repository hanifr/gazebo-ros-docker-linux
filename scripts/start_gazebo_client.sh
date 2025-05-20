#!/bin/bash

echo "🖥️ Starting Gazebo client..."

# Check if container is running
if ! docker ps | grep -q gazebo_sim; then
    echo "❌ Gazebo container is not running. Please start it with:"
    echo "   docker-compose up -d"
    exit 1
fi

# Start the Gazebo client
echo "Launching gzclient from the Gazebo container..."
docker-compose exec gazebo bash -c "export GAZEBO_MASTER_URI=http://gazebo:11345 && gzclient"

# Check exit status
if [ $? -ne 0 ]; then
    echo "❌ Failed to start Gazebo client. Possible issues:"
    echo "   - X11 forwarding not properly configured"
    echo "   - Graphics drivers incompatibility"
    echo "   - Container connectivity problems"
    echo ""
    echo "Try the following:"
    echo "   - Ensure X11 is properly configured: xhost +local:docker"
    echo "   - Check Gazebo server is running: docker-compose logs gazebo"
else
    echo "✅ Gazebo client exited normally"
fi