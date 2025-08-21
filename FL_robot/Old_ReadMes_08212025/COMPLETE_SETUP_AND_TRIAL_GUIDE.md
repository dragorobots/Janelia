# 🤖 FL_robot Complete Setup and Trial Guide

## 📋 **Table of Contents**
1. [System Overview](#system-overview)
2. [First-Time Setup](#first-time-setup)
3. [Daily Startup Procedure](#daily-startup-procedure)
4. [Running a Complete Trial](#running-a-complete-trial)
5. [Troubleshooting](#troubleshooting)
6. [Emergency Procedures](#emergency-procedures)

---

## 🎯 **System Overview**

The FL_robot system is a complete hide-and-seek experiment platform with:
- **Robot**: Raspberry Pi with camera, LiDAR, motors, running ROS2
- **PC GUI**: k-ToM experimenter interface for control and data collection
- **Communication**: WebSocket bridge between PC and robot
- **Features**: Line following, rat detection, k-ToM modeling, automated trials

### **File Structure**
```
FL_robot/
├── hide_and_seek.py              # Main robot behavior controller
├── hide_and_seek_bridge.py       # PC-robot communication bridge
├── ktom_experimenter.py          # PC GUI application
├── camera_server.py              # Robot camera streaming server
├── color_measurer.py             # Color measurement tool
├── start_hide_and_seek.sh        # Robot startup script
├── setup_camera_server.sh        # Camera server setup
├── first_boot_pc.sh/.bat         # PC first-time setup
├── first_boot_robot.sh           # Robot first-time setup
└── requirements.txt              # Python dependencies
```

---

## 🚀 **First-Time Setup**

### **Prerequisites**
- Git repository cloned on both PC and robot
- PC: Windows 10+, macOS 10.14+, or Ubuntu 18.04+
- Robot: Ubuntu 20.04+ or Raspberry Pi OS
- Python 3.8+ on both systems
- ROS2 Humble on robot

### **Step 1: PC Setup**

#### **Windows:**
```cmd
# Open Command Prompt as Administrator
cd C:\path\to\FL_robot
.\first_boot_pc.bat
```

#### **Linux/macOS:**
```bash
cd /path/to/FL_robot
chmod +x first_boot_pc.sh
./first_boot_pc.sh
```

**What this does:**
- Installs Python packages: numpy, scipy, opencv-python, flask, roslibpy, pillow
- Installs system dependencies (Linux only)
- Verifies all components work

**Success indicators:**
- ✅ All packages install without errors
- ✅ k-ToM GUI starts without import errors
- ✅ Color measurer starts without import errors

### **Step 2: Robot Setup**

```bash
# SSH into robot (replace with your robot's IP)
ssh root@10.0.0.234

# Navigate to project directory
cd ~/yahboomcar_ws/src/Janelia/FL_robot

# Run first boot script
chmod +x first_boot_robot.sh
./first_boot_robot.sh
```

**What this does:**
- Installs Python packages: numpy, scipy, opencv-python, flask, roslibpy
- Installs ROS2 system packages
- Sets up camera permissions
- Verifies ROS2 and camera access

**Success indicators:**
- ✅ ROS2 commands work (`ros2 --version`)
- ✅ Camera is accessible
- ✅ Robot nodes import without errors

### **Step 3: Verify Setup**

#### **On PC:**
```bash
# Test imports
python -c "import numpy, scipy, cv2, flask, roslibpy; print('✅ All packages work!')"

# Test GUI
python ktom_experimenter.py
# Should open GUI without errors
```

#### **On Robot:**
```bash
# Test ROS2
ros2 --version

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(f'Camera: {cap.isOpened()}')"

# Test robot nodes
python3 -c "import hide_and_seek; print('✅ Robot nodes work!')"
```

---

## 🔄 **Daily Startup Procedure**

### **Step 1: Start Robot System**

#### **On Robot (SSH or direct access):**
```bash
# Navigate to project directory
cd ~/yahboomcar_ws/src/Janelia/FL_robot

# Start the main robot system
./start_hide_and_seek.sh
```

**Expected output:**
```
Starting Hide and Seek Robot System...
Starting bridge node...
[INFO] [timestamp] [hide_and_seek_bridge]: Initializing Hide and Seek Bridge...
[INFO] [timestamp] [hide_and_seek_bridge]: Bridge initialized and ready for PC communication
Starting main hide and seek node...
[INFO] [timestamp] [hide_and_seek_node]: Initializing Hide and Seek Node...
[INFO] [timestamp] [hide_and_seek_node]: Node initialized. Starting in START state.
Both nodes started. Bridge PID: XXXX, Main PID: XXXX
Press Ctrl+C to stop both nodes
```

#### **Start Camera Server (Optional - for color measurement):**
```bash
# In a new terminal on robot
./setup_camera_server.sh
```

**Expected output:**
```
🎥 Setting up Robot Camera Server
==================================
📦 Installing required packages...
📷 Checking camera availability...
✅ Camera 0 is available
✅ Camera resolution: 640x480
🚀 Starting camera server...
   The server will be available at:
   - http://localhost:8080/
   - http://YOUR_ROBOT_IP:8080/stream.mjpg
```

### **Step 2: Start PC GUI**

#### **On PC:**
```bash
# Navigate to project directory
cd /path/to/FL_robot

# Start the k-ToM GUI
python ktom_experimenter.py
```

**Expected output:**
- GUI window opens with tabs: "Setup", "Robot Control", "Data"
- No error messages in console

### **Step 3: Connect PC to Robot**

#### **In the GUI:**
1. Go to **"Robot Control"** tab
2. Verify robot IP address (default: `10.0.0.234`)
3. Click **"Connect to Robot"**
4. Wait for connection confirmation

**Success indicators:**
- ✅ Status shows "Successfully connected to robot"
- ✅ Robot status updates appear in the log
- ✅ No connection error messages

#### **Optional: Check Robot Status**
1. Click **"🔍 Check Robot Status"** button
2. Verify both hide_and_seek.sh and bridge are running
3. If not running, follow the instructions in the popup

---

## 🧪 **Running a Complete Trial**

### **Step 1: Configure Experiment**

#### **In the "Setup" tab:**
1. **Robot k-Level**: Set robot's Theory of Mind level (0-3)
   - k=0: Random or patterned strategy
   - k=1: Models rat as k=0
   - k=2: Models rat as k=1
   - k=3: Models rat as k=2

2. **Number of Hiding Spots**: Set to 4 (A, B, C, D)

3. **For k=0 Strategy**: Choose "Patterned" or "Percentage"

4. **Click "Start Trial"** to get k-ToM recommendation

**Expected output:**
- GUI shows k-ToM recommendation (e.g., "B")
- Belief values update
- Trial is ready to begin

### **Step 2: Send Target to Robot**

#### **Option A: Auto-Send (Recommended)**
1. Click **"🤖 Auto-Send k-ToM Target"** button
2. Robot receives the recommended hiding spot automatically
3. Trial progress resets to step 0

#### **Option B: Manual Selection**
1. In "Robot Control" tab, select target spot (A, B, C, or D)
2. Click "Send Target Spot"
3. Robot receives the selected spot

### **Step 3: Start Robot Trial**

#### **On Robot:**
1. **Position robot** at the start line (facing the line to follow)
2. **Press 's'** in the robot terminal to start trial manually
   - OR wait for PC to send start command

#### **On PC:**
1. In "Robot Control" tab, click **"Start Trial"**
2. Robot begins the trial sequence

### **Step 4: Monitor Trial Progress**

#### **Trial Progress Indicator shows 8 steps:**
1. **⭕ Leave entrance** - Robot starts line following
2. **⭕ Reach intersection & start new line** - Robot reaches intersection
3. **⭕ Follow the line** - Robot follows line to hiding spot
4. **⭕ Wait at hiding spot (detect rat)** - Robot waits for rat
5. **⭕ Wait 10s, turn 180°** - Robot turns after rat detection
6. **⭕ Follow line back (same color)** - Robot returns via line following
7. **⭕ Reach intersection & return to start** - Robot reaches intersection
8. **⭕ Wait for new command, turn 180°, reset** - Trial complete

#### **Visual Indicators:**
- **⭕ Gray**: Not started
- **🔄 Orange**: Currently active
- **✅ Green**: Completed

### **Step 5: Rat Detection**

#### **Automatic Detection (LiDAR):**
- Robot automatically detects rat using LiDAR sensor
- Detection occurs when rat approaches within 15cm
- Robot transitions to next phase automatically

#### **Manual Detection:**
1. In "Robot Control" tab, click **"Manual Found"** button
2. Robot immediately transitions to next phase
3. Use this if automatic detection fails

### **Step 6: Data Collection**

#### **Record Search Sequence:**
1. **Observe the rat's search behavior**
2. **Record the complete search sequence** in order
3. **Format**: Comma-separated list (e.g., "A,C,B,D" or "1,3,2,4")
4. **Enter in GUI**: Use the search sequence input field

#### **Example Search Sequences:**
- `A,B,C,D` - Rat searched in order
- `B,A,D,C` - Rat searched randomly
- `A,C,B` - Rat found robot before searching all spots

### **Step 7: Complete Trial**

#### **When trial ends:**
1. **Robot returns to start position**
2. **Trial progress shows all steps complete**
3. **Data is automatically saved** to CSV file
4. **Robot is ready for next trial**

#### **Data Saved:**
- Trial number
- Robot k-level
- Robot hiding spot
- Rat's first search
- Whether rat found robot
- Time to find (if found)
- Complete search sequence
- k-ToM beliefs and predictions

### **Step 8: Repeat for Multiple Trials**

#### **For next trial:**
1. **Click "Start Trial"** in Setup tab
2. **Get new k-ToM recommendation**
3. **Auto-send target** to robot
4. **Repeat steps 3-7**

#### **Recommended Trial Count:**
- **Minimum**: 20 trials per condition
- **Optimal**: 50+ trials per condition
- **Multiple conditions**: Test different k-levels

---

## 🔧 **Troubleshooting**

### **Connection Issues**

#### **"Connection Failed"**
```bash
# Check robot IP
ping 10.0.0.234

# Check if robot is running
ssh root@10.0.0.234
ps aux | grep hide_and_seek
```

#### **"Robot Status Check Failed"**
```bash
# On robot, restart the system
cd ~/yahboomcar_ws/src/Janelia/FL_robot
pkill -f hide_and_seek
./start_hide_and_seek.sh
```

### **Line Following Issues**

#### **Robot not following line**
1. **Check lighting** - Ensure good, consistent lighting
2. **Check line color** - Use color measurer to verify HSV values
3. **Check camera** - Ensure camera is working and focused
4. **Adjust parameters** - Modify HSV ranges in code if needed

#### **Robot getting stuck**
1. **Check line continuity** - Ensure line is unbroken
2. **Check surface** - Ensure smooth, non-reflective surface
3. **Check robot position** - Ensure robot starts on the line

### **Camera Issues**

#### **"Camera not accessible"**
```bash
# On robot, check camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"

# If false, try other camera indices
python3 -c "import cv2; cap = cv2.VideoCapture(1); print(cap.isOpened())"
```

#### **"Camera server not working"**
```bash
# Restart camera server
pkill -f camera_server
./setup_camera_server.sh
```

### **ROS2 Issues**

#### **"ROS2 not found"**
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Check ROS2 installation
ros2 --version
```

#### **"Node not found"**
```bash
# Check running nodes
ros2 node list

# Restart robot system
pkill -f hide_and_seek
./start_hide_and_seek.sh
```

### **GUI Issues**

#### **"GUI not opening"**
```bash
# Check Python and tkinter
python -c "import tkinter; print('tkinter works')"

# On Linux, install tkinter
sudo apt install python3-tk
```

#### **"Import errors"**
```bash
# Reinstall packages
pip install -r requirements.txt

# Check Python version
python --version  # Should be 3.8+
```

---

## 🚨 **Emergency Procedures**

### **Emergency Stop**

#### **Immediate Stop:**
1. **Press "🚨 ABORT MISSION"** button in GUI
2. **OR press Escape key** in GUI
3. **OR press Ctrl+C** in robot terminal

#### **If robot is unresponsive:**
```bash
# SSH into robot
ssh root@10.0.0.234

# Kill all robot processes
pkill -f hide_and_seek
pkill -f hide_and_seek_bridge
pkill -f camera_server

# Or force kill
killall -9 python3
```

### **Robot Stuck or Behaving Erratically**

#### **Immediate Actions:**
1. **Stop robot movement** using emergency stop
2. **Check robot position** - ensure it's not stuck
3. **Check line visibility** - ensure line is visible to camera
4. **Restart robot system** if needed

#### **Restart Procedure:**
```bash
# On robot
cd ~/yahboomcar_ws/src/Janelia/FL_robot
pkill -f hide_and_seek
./start_hide_and_seek.sh
```

### **Data Loss Prevention**

#### **Before stopping:**
1. **Save current data** in GUI
2. **Note current trial number**
3. **Record any partial data** manually

#### **After restart:**
1. **Check CSV files** for saved data
2. **Resume from last complete trial**
3. **Update trial numbering** if needed

---

## 📊 **Data Analysis**

### **CSV File Format**
```csv
trial_num,robot_k_level,robot_hiding_spot,rat_first_search,was_found,time_to_find,search_sequence,belief_rat_is_k0,belief_rat_is_k1,belief_rat_is_k2,pred_rat_searches_spot0,pred_rat_searches_spot1,pred_rat_searches_spot2,pred_rat_searches_spot3
1,2,1,B,True,45.2,"B,A,C,D",0.3,0.5,0.2,0.25,0.35,0.25,0.15
```

### **Key Metrics**
- **Success Rate**: Percentage of trials where rat found robot
- **Search Efficiency**: Average number of searches before finding
- **Search Patterns**: Analysis of rat's search strategies
- **k-ToM Accuracy**: How well robot's model predicted rat behavior

### **Exporting Data**
1. **In GUI**: Go to "Data" tab
2. **Click "Export Data"** to save CSV
3. **Files saved**: `hide_and_seek_trial_outcomes_YYYYMMDD/`
4. **Format**: Timestamped folders with CSV files

---

## 🎯 **Best Practices**

### **Before Each Session**
1. **Check robot battery** - Ensure sufficient charge
2. **Clean camera lens** - Remove dust and fingerprints
3. **Check line condition** - Ensure lines are clean and visible
4. **Test robot movement** - Verify motors work correctly
5. **Calibrate lighting** - Ensure consistent illumination

### **During Trials**
1. **Monitor robot behavior** - Watch for unusual movements
2. **Record data accurately** - Double-check search sequences
3. **Maintain consistent conditions** - Keep lighting and environment stable
4. **Take breaks** - Allow robot to cool down between trials

### **After Each Session**
1. **Save all data** - Export CSV files
2. **Clean robot** - Remove dust and debris
3. **Charge battery** - Ensure ready for next session
4. **Review data** - Check for anomalies or missing data

---

## 📞 **Support**

### **Common Issues and Solutions**
- **See troubleshooting section above**
- **Check log files** for error messages
- **Verify network connectivity** between PC and robot

### **Getting Help**
1. **Check this guide** for common solutions
2. **Review README.md** for technical details
3. **Check FIRST_BOOT_GUIDE.md** for setup issues
4. **Review test files** for system validation

### **System Requirements Summary**
- **PC**: Python 3.8+, 4GB RAM, 2GB storage
- **Robot**: Python 3.8+, ROS2 Humble, 2GB RAM, 5GB storage
- **Network**: Stable connection between PC and robot
- **Hardware**: Camera, LiDAR, motors, servos on robot

---

## 🎉 **Success Checklist**

### **Setup Complete When:**
- ✅ PC GUI opens without errors
- ✅ Robot system starts without errors
- ✅ PC connects to robot successfully
- ✅ Camera server works (if needed)
- ✅ Line following works correctly
- ✅ Rat detection works (LiDAR or manual)

### **Trial Ready When:**
- ✅ Robot positioned at start line
- ✅ Line colors configured correctly
- ✅ Lighting is consistent
- ✅ k-ToM parameters set
- ✅ Data collection ready
- ✅ Emergency stop accessible

### **Data Quality Indicators:**
- ✅ Complete search sequences recorded
- ✅ No missing trial data
- ✅ Consistent trial conditions
- ✅ Proper k-ToM recommendations
- ✅ Accurate timing measurements

---

**🎯 You're now ready to conduct hide-and-seek experiments with your FL_robot system!**
