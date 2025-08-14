# Hide and Seek Robot System

A complete ROS2-based robot control system for automated hide-and-seek experiments with rats, featuring k-ToM (Theory of Mind) modeling and PC-GUI integration.

## 🚀 System Overview

This system consists of:
- **Robot Node** (`hide_and_seek.py`): Main robot behavior controller
- **Bridge Node** (`hide_and_seek_bridge.py`): PC-robot communication bridge
- **PC GUI** (`ktom_experimenter.py`): Experiment control and k-ToM analysis interface
- **Integration Tests** (`test_integration.py`): System validation tests

## 📋 Features

### Robot Capabilities
- **Line Following**: Computer vision-based colored line tracking
- **Rat Detection**: LiDAR-based proximity detection
- **Reward Dispensing**: Automated cheerio dispenser
- **State Machine**: Complete trial automation
- **Manual Override**: PC-controlled manual operation

### k-ToM Integration
- **Multi-level Theory of Mind**: k=0, k=1, k=2, k=3 modeling
- **Adaptive Strategy**: Robot learns and adapts to rat behavior
- **Real-time Analysis**: Live belief and prediction tracking
- **Data Logging**: Comprehensive trial outcome recording

### PC Control Interface
- **Real-time Status**: Live robot state monitoring
- **Manual Control**: Direct robot movement control
- **Experiment Setup**: k-ToM level and parameter configuration
- **Data Export**: CSV logging with timestamped folders

## 🏗️ Architecture

```
PC GUI (ktom_experimenter.py)
    ↓ WebSocket (port 10090)
Bridge Node (hide_and_seek_bridge.py)
    ↓ ROS2 Topics
Robot Node (hide_and_seek.py)
    ↓ Hardware
Robot (Camera, LiDAR, Motors, Servos)
```

### Communication Protocol

| PC → Robot | Topic | Message Type | Purpose |
|------------|-------|--------------|---------|
| Target Spot | `/hide_and_seek/target_spot` | `std_msgs/Int32` | Set hiding location (0-3) |
| Drive Mode | `/hide_and_seek/toggles` | `std_msgs/String` | `drive_mode=auto_line/manual_line/manual_drive` |
| Rat Mode | `/hide_and_seek/toggles` | `std_msgs/String` | `rat_mode=auto/manual` |
| Manual Found | `/hide_and_seek/manual_found` | `std_msgs/Bool` | Manual rat detection signal |
| Line Color | `/hide_and_seek/line_color` | `std_msgs/String` | `hue=220` (degrees) |
| Velocity | `/hide_and_seek/cmd_vel` | `geometry_msgs/Twist` | Manual driving commands |

| Robot → PC | Topic | Message Type | Purpose |
|------------|-------|--------------|---------|
| Line Status | `/line_follow/status` | `std_msgs/String` | `following/searching/stopped` |
| Rat Detection | `/rat_detection/found` | `std_msgs/Bool` | Rat proximity status |
| Progress | `/hide_and_seek/progress` | `std_msgs/String` | Current trial phase |

## 🛠️ Installation

### Prerequisites
- Python 3.8+
- ROS2 (Humble or later)
- OpenCV
- NumPy, SciPy

### Dependencies
```bash
pip install -r requirements.txt
```

### Robot Setup
1. Ensure ROS2 is installed and sourced
2. Install required ROS2 packages:
   ```bash
   sudo apt install ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-std-msgs
   ```

## 🚀 Usage

### 1. Deploy to Robot
```bash
./update_robot.sh
```

### 2. Start Robot System
```bash
./start_hide_and_seek.sh
```

### 3. Start PC GUI
```bash
python ktom_experimenter.py
```

### 4. Connect and Run Experiments
1. Open the "Robot Control" tab in the GUI
2. Enter robot IP address (default: 192.168.1.100)
3. Click "Connect to Robot"
4. Configure experiment parameters in "Setup" tab
5. Run trials and monitor results

### Emergency Abort Functionality
- **🚨 ABORT MISSION Button**: Located in Robot Control tab
- **Keyboard Shortcuts**: Press `Escape` or `Ctrl+A` for emergency stop
- **E-Stop Features**:
  - Immediately halts robot movement
  - Sends abort signal to robot
  - Option to kill hide_and_seek processes via SSH
  - Visual status indicator shows abort mode

## 📊 Experiment Workflow

### 1. Setup Phase
- Configure robot's k-ToM level (0-3)
- Set number of hiding spots (2-10)
- For k=0: Configure strategy (patterned/percentage)

### 2. Trial Execution
- Robot recommends hiding spot based on k-ToM model
- Robot follows colored line to target location
- Robot waits for rat detection (LiDAR or manual)
- Robot dispenses reward when rat approaches
- Robot returns home via line following
- **Data Collection**: Record complete rat search sequence (A-D or 1-4, comma-separated)

### 3. Data Collection
- Trial outcomes automatically logged
- k-ToM beliefs and predictions recorded
- CSV export with timestamped folders
- Real-time model state display

## 🧪 Testing

Run comprehensive integration tests:
```bash
python test_integration.py
```

Tests include:
- ✅ RosLink communication
- ✅ GUI integration
- ✅ k-ToM functionality
- ✅ File saving

## 📁 File Structure

```
FL_robot/
├── hide_and_seek.py              # Main robot node
├── hide_and_seek_bridge.py       # PC-robot bridge
├── ktom_experimenter.py          # PC GUI application
├── test_integration.py           # Integration tests
├── start_hide_and_seek.sh        # Robot launch script
├── update_robot.sh               # Deployment script
├── requirements.txt              # Python dependencies
├── CURSOR_BRIEF.md              # Development context
└── README.md                    # This file
```

## 🔧 Configuration

### Robot Parameters
- `linear_speed`: Line following speed (default: 0.12 m/s)
- `LIDAR_PROXIMITY_LIMIT`: Rat detection distance (default: 0.15m)
- `WAIT_DURATION`: Maximum wait time (default: 120s)
- `turn_duration`: Return turn duration (default: 7.5s)

### k-ToM Parameters
- `learning_rate`: Belief update rate (default: 0.7)
- `beta`: Softmax temperature (default: 3.0)
- `LIDAR_PERSISTENCE_COUNT`: Detection confirmation (default: 3)

## 🐛 Troubleshooting

### Common Issues

1. **Connection Failed**
   - Check robot IP address
   - Ensure rosbridge server is running
   - Verify network connectivity

2. **Line Following Issues**
   - Adjust HSV color range
   - Check camera calibration
   - Verify lighting conditions

3. **LiDAR Detection Problems**
   - Clean LiDAR sensor
   - Adjust proximity limits
   - Check for environmental interference

### Debug Mode
Enable verbose logging by setting ROS log level:
```bash
ros2 run hide_and_seek hide_and_seek_node --ros-args --log-level debug
```

## 📈 Data Analysis

### CSV Output Format
- `trial_num`: Sequential trial number
- `robot_k_level`: Robot's k-ToM level
- `robot_hiding_spot`: Where robot hid (1-based)
- `rat_first_search`: Rat's first search location
- `was_found`: Whether rat found robot
- `time_to_find`: Time to discovery (if found)
- `search_sequence`: Complete search sequence (comma-separated A-D or 1-4)
- `belief_rat_is_k*`: Robot's beliefs about rat's sophistication
- `pred_rat_searches_spot*`: Robot's search predictions

### Analysis Tools
- Use the GUI's real-time model state display
- Export CSV data for statistical analysis
- Monitor belief evolution across trials

## 🤝 Contributing

1. Follow the established code structure
2. Add tests for new features
3. Update documentation
4. Test on both PC and robot platforms

## 📄 License

This project is part of the Janelia Research Campus FL_robot system.

## 🆘 Support

For issues and questions:
1. Check the troubleshooting section
2. Run integration tests
3. Review ROS2 logs
4. Consult the CURSOR_BRIEF.md for development context
