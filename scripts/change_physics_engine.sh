#!/bin/bash

# Check if physics engine argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <physics_engine>"
    echo "Available engines: ode, bullet, simbody, dart"
    exit 1
fi

ENGINE=$1

# Restart Gazebo with the specified physics engine
echo "🔄 Restarting Gazebo with $ENGINE physics engine..."
docker-compose exec -d gazebo bash -c "pkill gzserver && gzserver --verbose -e $ENGINE /worlds/test_world.world"

echo "✅ Physics engine changed to $ENGINE"
echo "📝 Use ./scripts/start_gazebo_client.sh to view the simulation"