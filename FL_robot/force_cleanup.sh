#!/bin/bash

echo "🔥 FORCE CLEANUP - Aggressive port cleanup..."

# Method 1: Kill all Python processes
echo "Killing all Python processes..."
pkill -9 python3 2>/dev/null
pkill -9 python 2>/dev/null

# Method 2: Kill specific processes
echo "Killing specific robot processes..."
pkill -9 -f "hide_and_seek" 2>/dev/null
pkill -9 -f "rosbridge" 2>/dev/null
pkill -9 -f "ros2" 2>/dev/null

# Method 3: Use fuser if available
if command -v fuser >/dev/null 2>&1; then
    echo "Using fuser to kill processes on port 10090..."
    fuser -k 10090/tcp 2>/dev/null
fi

# Method 4: Manual netstat approach
echo "Manual netstat cleanup..."
PIDS=$(netstat -tlnp 2>/dev/null | grep :10090 | awk '{print $7}' | cut -d'/' -f1 | grep -v -)
if [ ! -z "$PIDS" ]; then
    echo "Found PIDs: $PIDS"
    for pid in $PIDS; do
        if [ ! -z "$pid" ] && [ "$pid" != "-" ]; then
            echo "Force killing PID $pid"
            kill -9 $pid 2>/dev/null
        fi
    done
fi

# Wait longer for cleanup
echo "Waiting for cleanup..."
sleep 5

# Check if port is free
if netstat -tlnp 2>/dev/null | grep :10090 > /dev/null; then
    echo "❌ Port 10090 is STILL in use!"
    echo "Current processes using port 10090:"
    netstat -tlnp | grep :10090
    echo ""
    echo "Try these manual commands:"
    echo "1. Reboot the robot: sudo reboot"
    echo "2. Or use a different port: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=10091"
else
    echo "✅ Port 10090 is now free!"
fi

echo "🔥 Force cleanup complete!"
