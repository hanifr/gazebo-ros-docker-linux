#!/bin/bash

echo "🖥️ Starting Gazebo GUI with enhanced stability..."

# Check if XQuartz is running
if ! ps aux | grep -v grep | grep -q XQuartz; then
    echo "❌ XQuartz not running! Please start XQuartz before running this script."
    exit 1
fi

# Check if xhost is configured correctly
if ! xhost | grep -q "access control disabled"; then
    echo "⚠️ XQuartz access control may not be configured correctly."
    echo "   Please run: xhost + localhost"
    echo "   Then run this script again."
    exit 1
fi

# Ensure simulation is running
if ! docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && rostopic list | grep -q /gazebo"; then
    echo "⚠️ ROS-Gazebo bridge not running. Starting simulation first..."
    ./scripts/run_simulation.sh
    sleep 3
fi

# Try to run gzclient with more detailed output and error handling
echo "🚀 Launching Gazebo client with verbose output..."
docker-compose exec gazebo bash -c "export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    export DISPLAY=host.docker.internal:0 && \
    export QT_X11_NO_MITSHM=1 && \
    export LIBGL_ALWAYS_SOFTWARE=1 && \
    echo 'Starting gzclient...' && \
    gzclient --verbose"

echo "If Gazebo client crashed, check the output above for error messages."
