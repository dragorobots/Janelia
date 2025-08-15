#!/bin/bash
# Setup Camera Server on Robot
# Run this script on the robot to start the camera server

echo "🎥 Setting up Robot Camera Server"
echo "=================================="

# Check if we're on the robot (you can modify this check)
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "⚠️  Warning: This doesn't appear to be a ROS2 robot environment"
    echo "   Make sure you're running this on the robot!"
fi

# Install required packages if not already installed
echo "📦 Installing required packages..."
pip3 install flask opencv-python numpy

# Check if camera is available
echo "📷 Checking camera availability..."
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    print('✅ Camera 0 is available')
    ret, frame = cap.read()
    if ret:
        print(f'✅ Camera resolution: {frame.shape[1]}x{frame.shape[0]}')
    else:
        print('❌ Camera 0 is not working properly')
    cap.release()
else:
    print('❌ Camera 0 is not available')
    # Try other camera indices
    for i in range(1, 4):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f'✅ Camera {i} is available')
            cap.release()
            break
        cap.release()
"

echo ""
echo "🚀 Starting camera server..."
echo "   The server will be available at:"
echo "   - http://localhost:8080/"
echo "   - http://YOUR_ROBOT_IP:8080/stream.mjpg"
echo ""
echo "   Press Ctrl+C to stop the server"
echo ""

# Start the camera server
python3 camera_server.py --port 8080 --quality 80
