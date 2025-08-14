#!/bin/bash

echo "🧹 Cleaning up robot processes..."

# Kill any processes using port 10090
echo "Stopping processes on port 10090..."
sudo lsof -ti:10090 | xargs -r sudo kill -9

# Kill Python processes (be more specific to avoid killing other scripts)
echo "Stopping Python robot processes..."
sudo pkill -f "hide_and_seek"
sudo pkill -f "rosbridge"

# Kill ROS2 processes
echo "Stopping ROS2 processes..."
sudo pkill -f "ros2"

# Wait a moment for processes to fully terminate
sleep 2

# Check if port is free
if sudo lsof -i :10090 > /dev/null 2>&1; then
    echo "⚠️  Port 10090 is still in use. Force killing..."
    sudo lsof -ti:10090 | xargs -r sudo kill -9
    sleep 1
fi

# Verify cleanup
if ! sudo lsof -i :10090 > /dev/null 2>&1; then
    echo "✅ Port 10090 is now free"
else
    echo "❌ Port 10090 is still in use"
    sudo lsof -i :10090
fi

echo "🧹 Cleanup complete!"
