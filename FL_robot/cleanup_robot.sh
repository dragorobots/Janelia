#!/bin/bash

echo "🧹 Cleaning up robot processes..."

# Use netstat instead of lsof (more commonly available)
echo "Stopping processes on port 10090..."

# Find processes using port 10090 with netstat
PIDS=$(netstat -tlnp 2>/dev/null | grep :10090 | awk '{print $7}' | cut -d'/' -f1 | grep -v -)

if [ ! -z "$PIDS" ]; then
    echo "Found processes using port 10090: $PIDS"
    for pid in $PIDS; do
        if [ ! -z "$pid" ] && [ "$pid" != "-" ]; then
            echo "Killing process $pid"
            kill -9 $pid 2>/dev/null
        fi
    done
else
    echo "No processes found using port 10090"
fi

# Kill Python processes (be more specific to avoid killing other scripts)
echo "Stopping Python robot processes..."
pkill -f "hide_and_seek" 2>/dev/null
pkill -f "rosbridge" 2>/dev/null

# Kill ROS2 processes
echo "Stopping ROS2 processes..."
pkill -f "ros2" 2>/dev/null

# Wait a moment for processes to fully terminate
sleep 3

# Check if port is free using netstat
if netstat -tlnp 2>/dev/null | grep :10090 > /dev/null; then
    echo "⚠️  Port 10090 is still in use. Force killing..."
    PIDS=$(netstat -tlnp 2>/dev/null | grep :10090 | awk '{print $7}' | cut -d'/' -f1 | grep -v -)
    for pid in $PIDS; do
        if [ ! -z "$pid" ] && [ "$pid" != "-" ]; then
            kill -9 $pid 2>/dev/null
        fi
    done
    sleep 1
fi

# Verify cleanup
if ! netstat -tlnp 2>/dev/null | grep :10090 > /dev/null; then
    echo "✅ Port 10090 is now free"
else
    echo "❌ Port 10090 is still in use"
    netstat -tlnp | grep :10090
fi

echo "🧹 Cleanup complete!"
