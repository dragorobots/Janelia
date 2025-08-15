#!/bin/bash
# First Boot Setup for Host PC (Linux/macOS)
# Run this script after cloning the repository to install all required packages

set -e  # Exit on any error

echo "🚀 FL_robot - First Boot Setup for Host PC"
echo "=========================================="
echo "This script will install all required packages for the k-ToM GUI and robot control."
echo ""

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.8+ first."
    echo "   Ubuntu/Debian: sudo apt install python3 python3-pip"
    echo "   macOS: brew install python3"
    exit 1
fi

echo "✅ Python 3 found: $(python3 --version)"

# Check if pip is installed
if ! command -v pip3 &> /dev/null; then
    echo "❌ pip3 is not installed. Please install pip3 first."
    echo "   Ubuntu/Debian: sudo apt install python3-pip"
    echo "   macOS: brew install python3"
    exit 1
fi

echo "✅ pip3 found: $(pip3 --version)"

# Upgrade pip to latest version
echo ""
echo "📦 Upgrading pip to latest version..."
python3 -m pip install --upgrade pip

# Install system dependencies (Linux only)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo ""
    echo "📦 Installing system dependencies..."
    
    # Detect package manager
    if command -v apt-get &> /dev/null; then
        # Ubuntu/Debian
        sudo apt-get update
        sudo apt-get install -y \
            python3-tk \
            python3-dev \
            libgl1-mesa-glx \
            libglib2.0-0 \
            libsm6 \
            libxext6 \
            libxrender-dev \
            libgomp1 \
            libgthread-2.0-0
    elif command -v yum &> /dev/null; then
        # CentOS/RHEL/Fedora
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
    elif command -v dnf &> /dev/null; then
        # Fedora (newer versions)
        sudo dnf install -y \
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
        echo "⚠️  Unknown package manager. Please install tkinter manually."
    fi
fi

# Install Python packages
echo ""
echo "📦 Installing Python packages..."

# Core scientific computing
echo "   Installing numpy and scipy..."
pip3 install "numpy>=1.25.2" "scipy>=1.16.0"

# ROS2 communication
echo "   Installing ROS2 communication packages..."
pip3 install "roslibpy>=1.8.0"
echo "   Note: rosbridge-suite may need to be installed via system package manager"
echo "   Ubuntu/Debian: sudo apt install ros-humble-rosbridge-suite"

# GUI and image processing
echo "   Installing GUI and image processing packages..."
pip3 install "pillow>=10.0.0"

# Color measurement and camera tools
echo "   Installing camera and color measurement packages..."
pip3 install "opencv-python>=4.8.0"

# Camera server and web tools
echo "   Installing web server packages..."
pip3 install "flask>=2.0.0" "requests>=2.25.0"

# Verify installations
echo ""
echo "🔍 Verifying installations..."

# Test core packages
echo "   Testing numpy..."
python3 -c "import numpy; print(f'✅ numpy {numpy.__version__}')"

echo "   Testing scipy..."
python3 -c "import scipy; print(f'✅ scipy {scipy.__version__}')"

echo "   Testing tkinter..."
python3 -c "import tkinter; print('✅ tkinter')"

echo "   Testing opencv..."
python3 -c "import cv2; print(f'✅ opencv-python {cv2.__version__}')"

echo "   Testing flask..."
python3 -c "import flask; print(f'✅ flask {flask.__version__}')"

echo "   Testing roslibpy..."
python3 -c "import roslibpy; print('✅ roslibpy')"

echo "   Testing requests..."
python3 -c "import requests; print(f'✅ requests {requests.__version__}')"

# Test the main GUI
echo ""
echo "🧪 Testing k-ToM GUI..."
if python3 -c "from ktom_experimenter import KToMExperimenterGUI; print('✅ k-ToM GUI imports successfully')" 2>/dev/null; then
    echo "✅ k-ToM GUI is ready to use!"
else
    echo "❌ k-ToM GUI test failed. Please check the error above."
    exit 1
fi

# Test color measurer
echo ""
echo "🧪 Testing color measurer..."
if python3 -c "from color_measurer import ColorMeasurer; print('✅ Color measurer imports successfully')" 2>/dev/null; then
    echo "✅ Color measurer is ready to use!"
else
    echo "❌ Color measurer test failed. Please check the error above."
    exit 1
fi

echo ""
echo "🎉 Setup completed successfully!"
echo ""
echo "📋 What's installed:"
echo "   • Core scientific computing: numpy, scipy"
echo "   • ROS2 communication: rosbridge-suite, roslibpy"
echo "   • GUI framework: tkinter (built-in), pillow"
echo "   • Camera tools: opencv-python"
echo "   • Web tools: flask, requests"
echo ""
echo "🚀 Next steps:"
echo "   1. Start the k-ToM GUI: python3 ktom_experimenter.py"
echo "   2. Use the color measurer: python3 color_measurer.py"
echo "   3. Connect to your robot and start experiments!"
echo ""
echo "📚 For more information, see:"
echo "   • README.md - Main documentation"
echo "   • NEW_KTOM_FEATURES.md - New GUI features"
echo "   • ENHANCED_COLOR_MEASURER_GUIDE.md - Color measurement guide"
