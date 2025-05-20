#!/bin/bash

echo "🔧 Fixing CMakeLists.txt issue..."

# Remove the problematic file
docker-compose exec ros bash -c "rm -f /catkin_ws/src/CMakeLists.txt"

# Create proper symlink
docker-compose exec ros bash -c "cd /catkin_ws/src && \
    ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake CMakeLists.txt"

echo "🔍 Verifying the fix..."
docker-compose exec ros bash -c "ls -la /catkin_ws/src/CMakeLists.txt"

echo "🏗️ Attempting to build workspace..."
docker-compose exec ros bash -c "cd /catkin_ws && \
    mkdir -p build devel && \
    cd build && \
    cmake ../src -DCMAKE_INSTALL_PREFIX=/catkin_ws/devel && \
    make && \
    make install"

echo "✅ Fix completed"