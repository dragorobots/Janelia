# Camera Setup Guide for Robot Color Measurer

## 🎯 Overview

The color measurer tool needs access to the robot's camera to measure line colors. Since the robot doesn't have a camera server running by default, we need to set one up.

## 🔍 Current Status

- ✅ Robot is reachable at `10.0.0.234` (ping successful)
- ✅ Port 10090 is open (ROS bridge)
- ❌ No camera server running on common ports (8080, etc.)

## 🚀 Solution: Set Up Camera Server

### Step 1: Install Dependencies on Robot

First, you need to install the required packages on the robot:

```bash
# On the robot, run:
pip3 install flask opencv-python numpy
```

### Step 2: Start Camera Server on Robot

You have several options to start the camera server:

#### Option A: Use the Setup Script (Recommended)
```bash
# On Linux/Mac robot:
chmod +x setup_camera_server.sh
./setup_camera_server.sh

# On Windows robot:
setup_camera_server.bat
```

#### Option B: Manual Start
```bash
# On the robot, run:
python3 camera_server.py --port 8080 --quality 80
```

#### Option C: Different Camera Index
If camera 0 doesn't work, try other indices:
```bash
python3 camera_server.py --camera 1 --port 8080
python3 camera_server.py --camera 2 --port 8080
```

### Step 3: Verify Camera Server

Once the camera server is running, you should see:
```
🎥 Robot Camera Server
==============================
Camera Index: 0
Port: 8080
Quality: 80

✅ Camera opened successfully at index 0
🌐 Starting web server on port 8080...
📺 Camera feed available at:
   http://localhost:8080/
   http://localhost:8080/stream.mjpg
   http://localhost:8080/video_feed
```

### Step 4: Test Camera Access

From your PC, test if the camera is accessible:
```bash
# Test the camera stream
python find_camera_port.py --ports 8080
```

You should see:
```
✅ Port 8080 is open
✅ http://10.0.0.234:8080/stream.mjpg - Shape: (480, 640, 3)
```

## 🎨 Using the Color Measurer

### Step 1: Launch Color Measurer
```bash
# From your PC:
python color_measurer.py --ip 10.0.0.234 --port 8080
```

### Step 2: Connect to Camera
1. Open the color measurer GUI
2. Verify the IP is `10.0.0.234` and port is `8080`
3. Click "Connect to Camera"
4. You should see the camera feed appear

### Step 3: Measure Colors
1. Click and drag on the video to select a region of interest (ROI)
2. The RGB and HSV values will be displayed
3. Click "Copy RGB Values" to copy to clipboard
4. Use these values in the main k-ToM GUI

## 🔧 Troubleshooting

### Camera Server Won't Start
- **Error**: "Failed to open camera at index 0"
  - **Solution**: Try different camera indices (1, 2, 3)
  - **Solution**: Check if camera is connected and working

### Can't Connect from PC
- **Error**: "Connection failed"
  - **Solution**: Make sure camera server is running on robot
  - **Solution**: Check firewall settings
  - **Solution**: Verify robot IP address

### Poor Video Quality
- **Solution**: Adjust quality parameter:
  ```bash
  python3 camera_server.py --quality 90
  ```

### High Latency
- **Solution**: Reduce quality for better performance:
  ```bash
  python3 camera_server.py --quality 60
  ```

## 📋 Quick Commands

### On Robot (Start Camera Server)
```bash
# Quick start with default settings
python3 camera_server.py

# With custom settings
python3 camera_server.py --camera 1 --port 8080 --quality 80
```

### On PC (Use Color Measurer)
```bash
# Quick start
python color_measurer.py

# With custom settings
python color_measurer.py --ip 10.0.0.234 --port 8080
```

### Test Camera Access
```bash
# Scan for camera ports
python find_camera_port.py

# Test specific port
python find_camera_port.py --ports 8080
```

## 🎯 Next Steps

1. **Start the camera server on your robot** using one of the methods above
2. **Test the connection** using the port finder
3. **Launch the color measurer** from the k-ToM GUI
4. **Measure your line colors** and configure them in the main GUI

The color measurer should now work properly once the camera server is running on the robot! 🎨
