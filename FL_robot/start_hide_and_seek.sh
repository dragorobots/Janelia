#!/bin/bash

echo "Starting Hide and Seek Robot System..."

# Start the bridge node in the background
echo "Starting bridge node..."
python3 hide_and_seek_bridge.py &
BRIDGE_PID=$!

# Wait a moment for bridge to initialize
sleep 2

# Start the main hide and seek node
echo "Starting main hide and seek node..."
python3 hide_and_seek.py &
MAIN_PID=$!

echo "Both nodes started. Bridge PID: $BRIDGE_PID, Main PID: $MAIN_PID"
echo "Press Ctrl+C to stop both nodes"

# Wait for user interrupt
trap "echo 'Stopping nodes...'; kill $BRIDGE_PID $MAIN_PID; exit" INT

# Keep script running
wait
