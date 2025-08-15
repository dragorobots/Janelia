#!/bin/bash
# First Boot Setup for Robot (Linux/Raspberry Pi)
# Run this script after cloning the repository to install all required packages

set -e  # Exit on any error

echo "🤖 FL_robot - First Boot Setup for Robot"
echo "========================================"
echo "This script will install all required packages for the robot system."
echo ""

# Check if we're on a Linux system
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "❌ This script is designed for Linux systems (including Raspberry Pi)."
    echo "   For other systems, please install packages manually."
    exit 1
fi

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.8+ first."
    echo "   Ubuntu/Debian: sudo apt install python3 python3-pip"
    echo "   Raspberry Pi: sudo apt install python3 python3-pip"
    exit 1
fi

echo "✅ Python 3 found: $(python3 --version)"

# Check if pip is installed
if ! command -v pip3 &> /dev/null; then
    echo "❌ pip3 is not installed. Please install pip3 first."
    echo "   Ubuntu/Debian: sudo apt install python3-pip"
    echo "   Raspberry Pi: sudo apt install python3-pip"
    exit 1
fi

echo "✅ pip3 found: $(pip3 --version)"

# Check if ROS2 is installed
if [ ! -f "/opt/ros/humble/setup.bash" ] && [ ! -f "/opt/ros/foxy/setup.bash" ]; then
    echo "❌ ROS2 is not installed. Please install ROS2 first."
    echo ""
    echo "   For Ubuntu 22.04 (Humble):"
    echo "   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    echo ""
    echo "   For Raspberry Pi:"
    echo "   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    echo ""
    echo "   After installing ROS2, run this script again."
    exit 1
fi

# Detect ROS2 version
if [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO="humble"
    echo "✅ ROS2 Humble found"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    ROS_DISTRO="foxy"
    echo "✅ ROS2 Foxy found"
else
    echo "❌ Unknown ROS2 distribution"
    exit 1
fi

# Source ROS2
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Update package lists
echo ""
echo "📦 Updating package lists..."
sudo apt-get update

# Install system dependencies
echo ""
echo "📦 Installing system dependencies..."

# Detect package manager and install dependencies
if command -v apt-get &> /dev/null; then
    # Ubuntu/Debian/Raspberry Pi
    echo "   Installing core system packages..."
    sudo apt-get install -y \
        python3-tk \
        python3-dev \
        libgl1-mesa-glx \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender-dev \
        libgomp1

    # Try to install optional packages (may not be available on all systems)
    echo "   Installing optional system packages..."
    sudo apt-get install -y libgthread-2.0-0 || echo "⚠️  libgthread-2.0-0 not available, continuing..."

    echo "   Installing ROS2 message packages..."
    sudo apt-get install -y \
        ros-$ROS_DISTRO-geometry-msgs \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-std-msgs

    echo "   Installing ROS2 bridge packages..."
    sudo apt-get install -y ros-$ROS_DISTRO-rosbridge-suite || echo "⚠️  ros-$ROS_DISTRO-rosbridge-suite not available, will install via pip..."

    # Try to install Python ROS bridge packages, fall back to pip if not available
    echo "   Installing Python ROS bridge packages..."
    sudo apt-get install -y python3-rosbridge-suite || echo "⚠️  python3-rosbridge-suite not available, will install via pip..."
    sudo apt-get install -y python3-rosbridge-library || echo "⚠️  python3-rosbridge-library not available, will install via pip..."

elif command -v yum &> /dev/null; then
    # CentOS/RHEL
    sudo yum install -y \
        python3-tkinter \
        python3-devel \
        mesa-libGL \
        glib2 \
        libSM \
        libXext \
        libXrender \
        libgomp \
        gtk2
else
    echo "⚠️  Unknown package manager. Please install dependencies manually."
fi

# Upgrade pip to latest version
echo ""
echo "📦 Upgrading pip to latest version..."
python3 -m pip install --upgrade pip

# Install Python packages
echo ""
echo "📦 Installing Python packages..."

# Core scientific computing
echo "   Installing numpy and scipy..."
pip3 install "numpy>=1.21.0" "scipy>=1.7.0"

# Camera and image processing
echo "   Installing camera and image processing packages..."
pip3 install "opencv-python>=4.5.0"

# Web server for camera streaming
echo "   Installing web server packages..."
pip3 install "flask>=2.0.0" "requests>=2.25.0"

# ROS2 Python packages (if not already installed)
echo "   Installing ROS2 Python packages..."
pip3 install "roslibpy>=1.8.0"

# Install ROS bridge packages via pip if system packages weren't available
echo "   Installing ROS bridge packages via pip..."
pip3 install "rosbridge-suite>=0.11.13" || echo "⚠️  rosbridge-suite installation failed, continuing..."
pip3 install "rosbridge-library>=1.3.0" || echo "⚠️  rosbridge-library installation failed, continuing..."

# Verify installations
echo ""
echo "🔍 Verifying installations..."

# Test core packages
echo "   Testing numpy..."
python3 -c "import numpy; print(f'✅ numpy {numpy.__version__}')"

echo "   Testing scipy..."
python3 -c "import scipy; print(f'✅ scipy {scipy.__version__}')"

echo "   Testing opencv..."
python3 -c "import cv2; print(f'✅ opencv-python {cv2.__version__}')"

echo "   Testing flask..."
python3 -c "import flask; print(f'✅ flask {flask.__version__}')"

echo "   Testing roslibpy..."
python3 -c "import roslibpy; print('✅ roslibpy')"

# Test ROS2
echo "   Testing ROS2..."
source "/opt/ros/$ROS_DISTRO/setup.bash"
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 command line tools"
else
    echo "❌ ROS2 command line tools not found"
    exit 1
fi

# Test camera access
echo ""
echo "📷 Testing camera access..."
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

# Test ROS2 nodes
echo ""
echo "🧪 Testing ROS2 nodes..."

# Test hide_and_seek_bridge.py
echo "   Testing hide_and_seek_bridge.py..."
if python3 -c "import rclpy; from rclpy.node import Node; print('✅ ROS2 imports work')" 2>/dev/null; then
    echo "✅ ROS2 Python bindings work"
else
    echo "❌ ROS2 Python bindings failed"
    exit 1
fi

# Test hide_and_seek.py
echo "   Testing hide_and_seek.py..."
if python3 -c "import rclpy; import cv2; import numpy; print('✅ All robot dependencies work')" 2>/dev/null; then
    echo "✅ Robot dependencies work"
else
    echo "❌ Robot dependencies failed"
    exit 1
fi

# Test camera server
echo "   Testing camera server..."
if python3 -c "from camera_server import CameraServer; print('✅ Camera server imports work')" 2>/dev/null; then
    echo "✅ Camera server is ready"
else
    echo "❌ Camera server test failed"
    exit 1
fi

# Make scripts executable
echo ""
echo "🔧 Making scripts executable..."
chmod +x *.sh
chmod +x *.py

echo ""
echo "🎉 Robot setup completed successfully!"
echo ""
echo "📋 What's installed:"
echo "   • ROS2 ($ROS_DISTRO) with required packages"
echo "   • Core scientific computing: numpy, scipy"
echo "   • Camera tools: opencv-python"
echo "   • Web tools: flask, requests"
echo "   • ROS2 communication: roslibpy"
echo "   • System dependencies for GUI and camera"
echo ""
echo "🚀 Next steps:"
echo "   1. Start the robot system: ./start_hide_and_seek.sh"
echo "   2. Start camera server: ./setup_camera_server.sh"
echo "   3. Connect from your PC and start experiments!"
echo ""
echo "📚 For more information, see:"
echo "   • README.md - Main documentation"
echo "   • CAMERA_SETUP_GUIDE.md - Camera setup guide"
echo "   • ENHANCED_COLOR_MEASURER_GUIDE.md - Color measurement guide"
echo ""
echo "🔧 Useful commands:"
echo "   • Start robot: ./start_hide_and_seek.sh"
echo "   • Start camera: ./setup_camera_server.sh"
echo "   • Cleanup: ./cleanup_robot.sh"
echo "   • Force cleanup: ./force_cleanup.sh"
