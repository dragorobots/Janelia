# 🤖 Complete Hide-and-Seek Robot Tutorial for Beginners

## 📋 **Overview**

This tutorial will guide you through setting up and using a hide-and-seek robot system that plays with rats using advanced AI (k-ToM modeling). You'll learn how to:

1. **Set up your Windows PC** with all necessary software
2. **Connect to your robot** via RealVNC
3. **Install and configure** the robot software
4. **Run complete experiments** with the robot
5. **Collect and analyze data** from trials

**Time Required**: 2-3 hours for initial setup, then 10-15 minutes per experiment session

---

## 🖥️ **Step 1: Setting Up Your Windows PC**

### **1.1 Install Required Software**

#### **Install Python (if not already installed)**
1. Open **Command Prompt** (Press Windows key + R, type `cmd`, press Enter)
2. Check if Python is installed:
   ```cmd
   python --version
   ```
3. If you see a version number (like "Python 3.9.0"), you're good!
4. If not, install Python from [python.org](https://www.python.org/downloads/)
   - **Important**: Check "Add Python to PATH" during installation

#### **Install Git (if not already installed)**
1. In Command Prompt, check if Git is installed:
   ```cmd
   git --version
   ```
2. If you see a version number, you're good!
3. If not, install Git:
   - **Option A (Recommended)**: Download from [git-scm.com](https://git-scm.com/download/win)
   - **Option B**: Use winget (if available):
     ```cmd
     winget install Git.Git
     ```
   - **Option C**: Use Chocolatey (if you have it installed):
     ```cmd
     choco install git
     ```

#### **Verify Git Installation**
After installing Git, run:
```cmd
git --version
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### **1.2 Download the Robot Software**

1. Open Command Prompt
2. Navigate to your Documents folder:
   ```cmd
   cd %USERPROFILE%\Documents
   ```
3. Download the robot software:
   ```cmd
   git clone https://github.com/dragorobots/Janelia.git
   ```
4. Navigate to the robot folder:
   ```cmd
   cd Janelia\FL_robot
   ```

### **1.3 Install Python Packages**

1. In Command Prompt (still in the FL_robot folder), run:
   ```cmd
   first_boot_pc.bat
   ```

**What this does:**
- Installs all necessary Python packages
- Sets up the GUI interface
- Tests that everything works

**You should see:**
- ✅ All packages installing successfully
- ✅ "Setup completed successfully!" message
- ✅ "k-ToM GUI is ready to use!" message

---

## 🤖 **Step 2: Setting Up Your Robot**

### **2.1 Connect to Your Robot via RealVNC**

1. Open **RealVNC Viewer** on your Windows PC
2. Connect to your robot using the IP address provided by your lab
3. You should see the robot's desktop

### **2.2 Access the Robot's Terminal**

1. In RealVNC, open a terminal window
2. You should see a command prompt like: `root@robot:~#`

### **2.3 Install Git on Robot (if not already installed)**

1. In the robot's terminal, check if Git is installed:
   ```bash
   git --version
   ```

2. If Git is not installed, install it:
   ```bash
   sudo apt update
   sudo apt install git -y
   ```

3. Configure Git on the robot:
   ```bash
   git config --global user.name "Robot"
   git config --global user.email "robot@lab.com"
   ```

### **2.4 Install Robot Software**

1. In the robot's terminal, navigate to the workspace:
   ```bash
   cd /root/yahboomcar_ws/src
   ```

2. Download the robot software:
   ```bash
   git clone https://github.com/dragorobots/Janelia.git
   ```

3. Navigate to the robot folder:
   ```bash
   cd Janelia/FL_robot
   ```

4. Install all robot packages:
   ```bash
   chmod +x first_boot_robot.sh
   ./first_boot_robot.sh
   ```

**What this does:**
- Installs ROS2 (robot operating system)
- Installs Python packages for robot control
- Sets up camera and sensor access
- Tests that everything works

**You should see:**
- ✅ "Robot setup completed successfully!" message
- ✅ All packages installing without errors

### **2.5 Set Up Git Access (One-time setup)**

1. In the robot's terminal, generate an SSH key:
   ```bash
   ssh-keygen -t ed25519 -C "robot-setup" -f ~/.ssh/id_ed25519 -N ""
   ```

2. Display the public key:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```

3. Copy the entire output (starts with "ssh-ed25519" and ends with "robot-setup")

4. **On your Windows PC**, open a web browser and go to [GitHub SSH Keys](https://github.com/settings/keys)

5. Click "New SSH key"
6. Give it a title like "Robot SSH Key"
7. Paste the key you copied from the robot
8. Click "Add SSH key"

### **2.6 Configure Robot for Git Access**

1. Back in the robot's terminal, test the connection:
   ```bash
   ssh -T git@github.com
   ```

2. You should see: "Hi dragorobots! You've successfully authenticated..."

3. Set up the robot's working directory:
   ```bash
   cd /root/yahboomcar_ws/src
   mv yahboomcar_astra/hide_and_seek yahboomcar_astra/hide_and_seek.backup
   ln -s /root/yahboomcar_ws/src/Janelia/FL_robot yahboomcar_astra/hide_and_seek
   ```

---

## 🔧 **Step 3: Finding Your Robot's IP Address**

### **3.1 Get the Robot's IP Address**

1. In the robot's terminal, run:
   ```bash
   hostname -I
   ```

2. You'll see something like: `10.0.0.234 192.168.1.100`
3. **Write down the first IP address** (this is your robot's IP)

### **3.2 Test Connection from Your Windows PC**

1. On your Windows PC, open Command Prompt
2. Test if you can reach the robot:
   ```cmd
   ping YOUR_ROBOT_IP
   ```
   (Replace YOUR_ROBOT_IP with the IP you wrote down)

3. You should see responses like "Reply from YOUR_ROBOT_IP: bytes=32 time=2ms TTL=64"

---

## 🚀 **Step 4: Starting the Robot System**

### **4.1 Start the Robot Software**

1. In the robot's terminal (via RealVNC), navigate to the robot folder:
   ```bash
   cd /root/yahboomcar_ws/src/Janelia/FL_robot
   ```

2. Start the robot system:
   ```bash
   ./start_hide_and_seek.sh
   ```

**You should see:**
- "Starting Hide and Seek Robot System..."
- "Starting bridge node..."
- "Starting main hide and seek node..."
- "Both nodes started. Bridge PID: XXXX, Main PID: XXXX"

**Keep this terminal window open!** The robot system is now running.

### **4.2 Start the Camera Server (Optional)**

1. Open a **new terminal window** in RealVNC
2. Navigate to the robot folder:
   ```bash
   cd /root/yahboomcar_ws/src/Janelia/FL_robot
   ```

3. Start the camera server:
   ```bash
   ./setup_camera_server.sh
   ```

**You should see:**
- "Starting camera server..."
- "Camera server running on port 8080"

---

## 💻 **Step 5: Starting the PC Interface**

### **5.1 Launch the Experiment GUI**

1. On your Windows PC, open Command Prompt
2. Navigate to the robot folder:
   ```cmd
   cd %USERPROFILE%\Documents\Janelia\FL_robot
   ```

3. Start the experiment interface:
   ```cmd
   python ktom_experimenter.py
   ```

**You should see:**
- A window titled "k-ToM Experimenter" opens
- Multiple tabs: "Setup", "Robot Control", "Trial Progress", "Data"

### **5.2 Connect to Your Robot**

1. In the GUI, click the **"Robot Control"** tab
2. In the "Robot IP Address" field, enter your robot's IP address
3. Click **"Connect to Robot"**

**You should see:**
- "Connected to robot successfully!" message
- Status indicators turn green
- Robot status information appears

---

## 🧪 **Step 6: Running Your First Experiment**

### **6.1 Configure the Experiment**

1. Click the **"Setup"** tab in the GUI
2. Set the following parameters:
   - **Robot k-Level**: `2` (for beginners)
   - **Number of Hiding Spots**: `4`
   - **Learning Rate**: `0.7`
   - **Beta**: `3.0`

### **6.2 Start a Trial**

1. Click the **"Trial Progress"** tab
2. Click **"Start New Trial"**
3. The robot will:
   - Analyze the situation using k-ToM
   - Recommend a hiding spot
   - Show you the recommendation

### **6.3 Send the Robot to Hide**

1. Click **"Auto-Send Target to Robot"**
2. Watch the robot:
   - Follow the colored line to the hiding spot
   - Wait for the rat to approach
   - Dispense a reward when the rat is detected
   - Return to the starting position

### **6.4 Record the Results**

1. When the trial is complete, you'll see a data entry form
2. Enter the rat's search sequence (e.g., "A,B,C,D" or "1,2,3,4")
3. Click **"Save Trial Data"**

---

## 📊 **Step 7: Understanding the Data**

### **7.1 What Data is Collected**

Each trial records:
- **Trial number**: Sequential trial number
- **Robot hiding spot**: Where the robot hid (1-4)
- **Rat search sequence**: Where the rat looked (A-D or 1-4)
- **Time to find**: How long it took the rat to find the robot
- **k-ToM beliefs**: What the robot thinks about the rat's intelligence
- **Predictions**: What the robot predicted the rat would do

### **7.2 Viewing the Data**

1. In the GUI, click the **"Data"** tab
2. You can see:
   - Real-time model state
   - Belief evolution across trials
   - Search predictions
   - Trial outcomes

### **7.3 Exporting Data**

1. Click **"Export Data to CSV"**
2. Choose a location on your Windows PC to save the file
3. The file will contain all trial data in spreadsheet format

---

## 🔄 **Step 8: Daily Workflow**

### **Morning Setup (5 minutes)**

1. **Connect to robot** via RealVNC
2. **Start robot system**:
   ```bash
   cd /root/yahboomcar_ws/src/Janelia/FL_robot
   ./start_hide_and_seek.sh
   ```
3. **Start PC GUI** on your Windows PC:
   ```cmd
   cd %USERPROFILE%\Documents\Janelia\FL_robot
   python ktom_experimenter.py
   ```
4. **Connect to robot** in the GUI

### **Running Experiments (10-15 minutes each)**

1. **Configure parameters** in Setup tab
2. **Start trial** in Trial Progress tab
3. **Send robot to hide** using Auto-Send
4. **Record results** when trial completes
5. **Repeat** for desired number of trials

### **End of Day (2 minutes)**

1. **Stop robot system**: Press Ctrl+C in robot terminal
2. **Close PC GUI**: Close the k-ToM window
3. **Export data** if needed

---

## 🆘 **Troubleshooting**

### **Common Issues and Solutions**

#### **"Git not found"**
- **On Windows**: Install Git from [git-scm.com](https://git-scm.com/download/win)
- **On Robot**: Run `sudo apt install git -y`

#### **"Python not found"**
- Install Python from [python.org](https://www.python.org/downloads/)
- Make sure to check "Add Python to PATH" during installation
- Restart Command Prompt after installation

#### **"Connection Failed"**
- Check robot IP address is correct
- Ensure robot system is running (`./start_hide_and_seek.sh`)
- Verify network connection: `ping YOUR_ROBOT_IP`

#### **"Robot not moving"**
- Check if robot is in manual mode
- Ensure line following is enabled
- Verify camera is working

#### **"Camera not working"**
- Start camera server: `./setup_camera_server.sh`
- Check camera permissions
- Restart robot system

#### **"GUI won't start"**
- Ensure Python packages are installed: `first_boot_pc.bat`
- Check Python version: `python --version`
- Restart Command Prompt and try again

#### **"Robot software errors"**
- Update robot software:
  ```bash
  cd /root/yahboomcar_ws/src/Janelia/FL_robot
  ./update_robot.sh
  ```
- Restart robot system

#### **"Package installation errors on robot"**
- If you see errors like "Unable to locate package" during robot setup:
  - This is normal for some Linux distributions
  - The script will continue and install packages via pip instead
  - Look for "⚠️" warnings - these are expected and not errors
  - If the script completes successfully, you can proceed
  - If the script fails completely, try running it again:
    ```bash
    cd /root/yahboomcar_ws/src/Janelia/FL_robot
    ./first_boot_robot.sh
    ```

### **Emergency Stop**

**If the robot is moving unexpectedly:**
1. Press **"🚨 ABORT MISSION"** button in GUI
2. Or press **Escape** key
3. Or press **Ctrl+A** in the GUI

**If the robot is completely unresponsive:**
1. In RealVNC, open terminal
2. Run: `pkill -f hide_and_seek`
3. Restart robot system

---

## 📚 **Advanced Features**

### **Color Measurement Tool**

To calibrate line colors:
1. On your Windows PC, run:
   ```cmd
   python color_measurer.py --ip YOUR_ROBOT_IP --port 8080
   ```
2. Use the tool to measure line colors
3. Update color values in the robot software

### **Manual Control Mode**

1. In GUI, set "Drive Mode" to "Manual Drive"
2. Use arrow keys or WASD to control robot manually
3. Useful for testing and debugging

### **Different k-Levels**

- **k=0**: Robot assumes rat is random
- **k=1**: Robot assumes rat thinks robot is random
- **k=2**: Robot assumes rat thinks robot thinks rat is random
- **k=3**: More sophisticated reasoning

---

## 📞 **Getting Help**

### **When to Ask for Help**

- Robot won't connect after following all steps
- Software installation fails
- Robot behaves unexpectedly
- Data collection issues

### **What Information to Provide**

- Your operating system (Windows)
- Robot IP address
- Error messages (copy and paste exactly)
- Steps you've already tried

---

## 🎉 **Success Checklist**

**You're ready to run experiments when:**

✅ **Windows PC Setup:**
- Python 3.8+ installed and added to PATH
- Git installed and configured
- All packages installed successfully
- GUI opens without errors

✅ **Robot Setup:**
- RealVNC connection working
- Git installed on robot
- Robot software installed
- SSH key added to GitHub
- Robot IP address known

✅ **System Running:**
- Robot system started (`./start_hide_and_seek.sh`)
- PC GUI connected to robot
- Camera working (optional)

✅ **First Trial:**
- Robot responds to commands
- Line following works
- Data is saved successfully

---

## 📝 **Quick Reference Commands**

### **On Your Windows PC:**
```cmd
# Start GUI
cd %USERPROFILE%\Documents\Janelia\FL_robot
python ktom_experimenter.py

# Test connection
ping YOUR_ROBOT_IP

# Reinstall packages (if needed)
first_boot_pc.bat

# Check Git installation
git --version

# Check Python installation
python --version
```

### **On Robot (via RealVNC):**
```bash
# Start robot system
cd /root/yahboomcar_ws/src/Janelia/FL_robot
./start_hide_and_seek.sh

# Update software
./update_robot.sh

# Start camera server
./setup_camera_server.sh

# Emergency stop
pkill -f hide_and_seek

# Check Git installation
git --version
```

---

**Congratulations!** You're now ready to run sophisticated hide-and-seek experiments with your robot. The system will automatically adapt to the rat's behavior using advanced AI modeling, making each trial more interesting and scientifically valuable.

Remember: The key to success is following the setup steps carefully and not being afraid to ask for help if something doesn't work as expected. Good luck with your experiments! 🐀🤖
