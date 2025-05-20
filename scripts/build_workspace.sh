#!/bin/bash

echo "🔨 Building ROS workspace..."
docker-compose exec ros bash -c "cd /catkin_ws && catkin_make && source devel/setup.bash"
echo "✅ Workspace built successfully!"