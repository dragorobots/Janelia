# 🚀 FL_robot - First Boot Setup Guide

## 📋 **Overview**

This guide provides step-by-step instructions for setting up the FL_robot system on both the **host PC** and **robot** after cloning the repository from git.

## 🎯 **Quick Start**

### **For Host PC (Linux/macOS):**
```bash
chmod +x first_boot_pc.sh
./first_boot_pc.sh
```

### **For Host PC (Windows):**
```cmd
.\first_boot_pc.bat
```

### **For Robot (Linux/Raspberry Pi):**
```bash
chmod +x first_boot_robot.sh
./first_boot_robot.sh
```

---

## 🖥️ **Host PC Requirements**

### **Minimum System Requirements:**
- **OS**: Windows 10+, macOS 10.14+, Ubuntu 18.04+
- **Python**: 3.8 or higher
- **RAM**: 4GB minimum, 8GB recommended
- **Storage**: 2GB free space

### **Required Packages:**

#### **Core Scientific Computing:**
- `numpy>=1.25.2` - Numerical computing
- `scipy>=1.16.0` - Scientific computing

#### **ROS2 Communication:**
- `roslibpy>=1.8.0` - Python ROS client library
- `rosbridge-suite` - WebSocket bridge for ROS2 (installed via system package manager)

#### **GUI Framework:**
- `tkinter` - Built into Python (GUI framework)
- `pillow>=10.0.0` - Image processing for GUI

#### **Camera Tools:**
- `opencv-python>=4.8.0` - Computer vision and camera access

#### **Web Tools:**
- `flask>=2.0.0` - Web server for camera streaming
- `requests>=2.25.0` - HTTP requests for camera server

### **System Dependencies (Linux):**
- `python3-tk` - Tkinter GUI framework
- `python3-dev` - Python development headers
- `libgl1-mesa-glx` - OpenGL libraries
- `libglib2.0-0` - GLib libraries
- `libsm6`, `libxext6`, `libxrender-dev` - X11 libraries
- `libgomp1`, `libgthread-2.0-0` - Threading libraries

---

## 🤖 **Robot Requirements**

### **Minimum System Requirements:**
- **OS**: Ubuntu 20.04+ or Raspberry Pi OS
- **Python**: 3.8 or higher
- **ROS2**: Humble or Foxy
- **RAM**: 2GB minimum, 4GB recommended
- **Storage**: 5GB free space
- **Camera**: USB camera or Pi Camera

### **Required Packages:**

#### **ROS2 System:**
- `ros-humble-geometry-msgs` - Geometry message types
- `ros-humble-sensor-msgs` - Sensor message types
- `ros-humble-std-msgs` - Standard message types
- `ros-humble-rosbridge-suite` - WebSocket bridge
- `python3-rosbridge-suite` - Python ROS bridge
- `python3-rosbridge-library` - ROS bridge library

#### **Core Scientific Computing:**
- `numpy>=1.25.2` - Numerical computing
- `scipy>=1.16.0` - Scientific computing

#### **Camera and Image Processing:**
- `opencv-python>=4.8.0` - Computer vision and camera access

#### **Web Server:**
- `flask>=2.0.0` - Web server for camera streaming
- `requests>=2.25.0` - HTTP requests

#### **ROS2 Communication:**
- `roslibpy>=1.8.0` - Python ROS client library

### **System Dependencies:**
- `python3-tk` - Tkinter GUI framework
- `python3-dev` - Python development headers
- `libgl1-mesa-glx` - OpenGL libraries
- `libglib2.0-0` - GLib libraries
- `libsm6`, `libxext6`, `libxrender-dev` - X11 libraries
- `libgomp1`, `libgthread-2.0-0` - Threading libraries

---

## 📦 **Package Installation Summary**

### **Host PC Packages (7 total):**
```bash
# Core scientific computing
pip install "numpy>=1.25.2" "scipy>=1.16.0"

# ROS2 communication
pip install "roslibpy>=1.8.0"
# Note: rosbridge-suite is installed via system package manager on Linux

# GUI and image processing
pip install "pillow>=10.0.0"

# Camera and web tools
pip install "opencv-python>=4.8.0" "flask>=2.0.0" "requests>=2.25.0"
```

### **Robot Packages (6 total):**
```bash
# Core scientific computing
pip3 install "numpy>=1.25.2" "scipy>=1.16.0"

# Camera and web tools
pip3 install "opencv-python>=4.8.0" "flask>=2.0.0" "requests>=2.25.0"

# ROS2 communication
pip3 install "roslibpy>=1.8.0"

# ROS2 system packages (via apt)
sudo apt install ros-humble-geometry-msgs ros-humble-sensor-msgs \
                  ros-humble-std-msgs ros-humble-rosbridge-suite \
                  python3-rosbridge-suite python3-rosbridge-library
```

---

## 🔧 **Setup Process**

### **Step 1: Clone Repository**
```bash
git clone <repository-url>
cd FL_robot
```

### **Step 2: Run First Boot Scripts**

#### **On Host PC:**
```bash
# Linux/macOS
chmod +x first_boot_pc.sh
./first_boot_pc.sh

# Windows
.\first_boot_pc.bat
```

#### **On Robot:**
```bash
chmod +x first_boot_robot.sh
./first_boot_robot.sh
```

### **Step 3: Verify Installation**

#### **Host PC Verification:**
- ✅ Python 3.8+ installed
- ✅ All Python packages installed
- ✅ k-ToM GUI imports successfully
- ✅ Color measurer imports successfully

#### **Robot Verification:**
- ✅ Python 3.8+ installed
- ✅ ROS2 (Humble/Foxy) installed
- ✅ All Python packages installed
- ✅ Camera access works
- ✅ ROS2 nodes import successfully

---

## 🚀 **Post-Setup Usage**

### **Starting the System:**

#### **On Robot:**
```bash
# Start the main robot system
./start_hide_and_seek.sh

# Start camera server (in another terminal)
./setup_camera_server.sh
```

#### **On Host PC:**
```bash
# Start the k-ToM GUI
python3 ktom_experimenter.py

# Start color measurer (optional)
python3 color_measurer.py
```

### **Testing the System:**
1. **Connect to robot** from k-ToM GUI
2. **Check robot status** using the new status check button
3. **Start a trial** and get k-ToM recommendation
4. **Auto-send target** to robot
5. **Monitor progress** using the trial progress indicator

---

## 🐛 **Troubleshooting**

### **Common Issues:**

#### **"Python not found"**
- Install Python 3.8+ from python.org or package manager
- Ensure Python is added to PATH (Windows)

#### **"pip not found"**
- Install pip: `sudo apt install python3-pip` (Linux)
- Upgrade pip: `python -m pip install --upgrade pip`

#### **"ROS2 not found" (Robot)**
- Install ROS2 Humble: https://docs.ros.org/en/humble/Installation/
- Source ROS2: `source /opt/ros/humble/setup.bash`

#### **"Camera not accessible" (Robot)**
- Check camera permissions: `sudo usermod -a -G video $USER`
- Test camera: `python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"`

#### **"Import errors"**
- Verify all packages installed: `pip list`
- Re-run first boot script
- Check Python version: `python3 --version`

#### **"GUI not working" (Linux)**
- Install tkinter: `sudo apt install python3-tk`
- Install X11 libraries: `sudo apt install libgl1-mesa-glx libglib2.0-0`

---

## 📚 **Additional Resources**

### **Documentation:**
- `README.md` - Main project documentation
- `NEW_KTOM_FEATURES.md` - New GUI features guide
- `ENHANCED_COLOR_MEASURER_GUIDE.md` - Color measurement guide
- `CAMERA_SETUP_GUIDE.md` - Camera setup guide

### **Useful Commands:**
```bash
# Check installed packages
pip list | grep -E "(numpy|scipy|opencv|flask|roslibpy)"

# Test imports
python3 -c "import numpy, scipy, cv2, flask, roslibpy; print('All packages work!')"

# Check ROS2 (robot)
ros2 --version
ros2 node list

# Check camera (robot)
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(f'Camera: {cap.isOpened()}')"
```

---

## 🎉 **Success Indicators**

### **Host PC Ready When:**
- ✅ All Python packages install without errors
- ✅ k-ToM GUI starts without import errors
- ✅ Color measurer starts without import errors
- ✅ Can connect to robot (when robot is running)

### **Robot Ready When:**
- ✅ ROS2 commands work (`ros2 --version`)
- ✅ Camera is accessible (`python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"`)
- ✅ Robot nodes import without errors
- ✅ Camera server starts without errors
- ✅ Can receive commands from host PC

---

## 📝 **Summary**

The FL_robot system requires **7 Python packages** on the host PC and **6 Python packages + ROS2 system packages** on the robot. The first boot scripts automate the entire setup process, including:

1. **Dependency checking** - Ensures Python, pip, and ROS2 are installed
2. **Package installation** - Installs all required Python packages
3. **System dependencies** - Installs OS-level dependencies (Linux)
4. **Verification testing** - Tests all components work correctly
5. **Permission setup** - Makes scripts executable

After running the first boot scripts, both systems are ready for hide-and-seek experiments! 🎉🤖
