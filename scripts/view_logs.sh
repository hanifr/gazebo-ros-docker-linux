#!/bin/bash

# Check container argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <container>"
    echo "Available containers: gazebo, ros"
    exit 1
fi

CONTAINER=$1

echo "📋 Viewing logs for $CONTAINER container..."
docker logs --follow ros_gazebo_simulation_$CONTAINER_1