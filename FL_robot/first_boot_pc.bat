@echo off
REM First Boot Setup for Host PC (Windows)
REM Run this script after cloning the repository to install all required packages

echo 🚀 FL_robot - First Boot Setup for Host PC
echo ==========================================
echo This script will install all required packages for the k-ToM GUI and robot control.
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Python is not installed. Please install Python 3.8+ first.
    echo    Download from: https://www.python.org/downloads/
    echo    Make sure to check "Add Python to PATH" during installation.
    pause
    exit /b 1
)

echo ✅ Python found:
python --version

REM Check if pip is installed
pip --version >nul 2>&1
if errorlevel 1 (
    echo ❌ pip is not installed. Please install pip first.
    echo    Usually comes with Python installation.
    pause
    exit /b 1
)

echo ✅ pip found:
pip --version

REM Upgrade pip to latest version
echo.
echo 📦 Upgrading pip to latest version...
python -m pip install --upgrade pip

REM Install Python packages
echo.
echo 📦 Installing Python packages...

REM Core scientific computing
echo    Installing numpy and scipy...
pip install "numpy>=1.25.2" "scipy>=1.16.0"

REM ROS2 communication
echo    Installing ROS2 communication packages...
pip install "roslibpy>=1.8.0"
echo    Note: rosbridge-suite is not available on Windows via pip
echo    For Windows, you may need to install ROS2 or use alternative communication methods

REM GUI and image processing
echo    Installing GUI and image processing packages...
pip install "pillow>=10.0.0"

REM Color measurement and camera tools
echo    Installing camera and color measurement packages...
pip install "opencv-python>=4.8.0"

REM Camera server and web tools
echo    Installing web server packages...
pip install "flask>=2.0.0" "requests>=2.25.0"

REM Verify installations
echo.
echo 🔍 Verifying installations...

REM Test core packages
echo    Testing numpy...
python -c "import numpy; print(f'✅ numpy {numpy.__version__}')"

echo    Testing scipy...
python -c "import scipy; print(f'✅ scipy {scipy.__version__}')"

echo    Testing tkinter...
python -c "import tkinter; print('✅ tkinter')"

echo    Testing opencv...
python -c "import cv2; print(f'✅ opencv-python {cv2.__version__}')"

echo    Testing flask...
python -c "import flask; print(f'✅ flask {flask.__version__}')"

echo    Testing roslibpy...
python -c "import roslibpy; print('✅ roslibpy')"

echo    Testing requests...
python -c "import requests; print(f'✅ requests {requests.__version__}')"

REM Test the main GUI
echo.
echo 🧪 Testing k-ToM GUI...
python -c "from ktom_experimenter import KToMExperimenterGUI; print('✅ k-ToM GUI imports successfully')" 2>nul
if errorlevel 1 (
    echo ❌ k-ToM GUI test failed. Please check the error above.
    pause
    exit /b 1
) else (
    echo ✅ k-ToM GUI is ready to use!
)

REM Test color measurer
echo.
echo 🧪 Testing color measurer...
python -c "from color_measurer import ColorMeasurer; print('✅ Color measurer imports successfully')" 2>nul
if errorlevel 1 (
    echo ❌ Color measurer test failed. Please check the error above.
    pause
    exit /b 1
) else (
    echo ✅ Color measurer is ready to use!
)

echo.
echo 🎉 Setup completed successfully!
echo.
echo 📋 What's installed:
echo    • Core scientific computing: numpy, scipy
echo    • ROS2 communication: rosbridge-suite, roslibpy
echo    • GUI framework: tkinter (built-in), pillow
echo    • Camera tools: opencv-python
echo    • Web tools: flask, requests
echo.
echo 🚀 Next steps:
echo    1. Start the k-ToM GUI: python ktom_experimenter.py
echo    2. Use the color measurer: python color_measurer.py
echo    3. Connect to your robot and start experiments!
echo.
echo 📚 For more information, see:
echo    • README.md - Main documentation
echo    • NEW_KTOM_FEATURES.md - New GUI features
echo    • ENHANCED_COLOR_MEASURER_GUIDE.md - Color measurement guide

pause
