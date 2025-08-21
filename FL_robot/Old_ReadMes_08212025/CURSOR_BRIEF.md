# Cursor Brief: Janelia FL_robot System

## Repository Context
- **Repo**: dragorobots/Janelia → folder `FL_robot/`
- **Robot workspace**: `/root/yahboomcar_ws/src/Janelia/FL_robot` (symlinked to working dir)
- **Live link**: rosbridge WebSocket on port 10090
- **Robot-side bridge**: `hide_and_seek_bridge.py` (subscribes to PC commands, publishes telemetry)

## Robot Management Scripts
- **Update/build on robot**: `/root/yahboomcar_ws/src/Janelia/FL_robot/update_robot.sh`
- **Start live link on robot**: `/root/yahboomcar_ws/src/Janelia/FL_robot/start_pc_link.sh`

## PC ⇄ Robot I/O (Single Source of Truth)

### Inputs (PC GUI → Robot)

| Purpose | Topic | ROS msg type | Payload schema (examples) |
|---------|-------|--------------|---------------------------|
| Start a trial | `/hide_and_seek/trial` | `std_msgs/msg/String` | `"start"` |
| Pick target spot | `/hide_and_seek/target_spot` | `std_msgs/msg/String` | `"A"`, `"B"`, `"C"`, `"D"` |
| Drive/line mode | `/hide_and_seek/toggles` | `std_msgs/msg/String` | `"drive_mode=auto_line"`, `"drive_mode=manual_line"`, `"drive_mode=manual_drive"` |
| Rat detect mode | `/hide_and_seek/toggles` | `std_msgs/msg/String` | `"rat_mode=auto"`, `"rat_mode=manual"` |
| Manual "found" button | `/hide_and_seek/manual_found` | `std_msgs/msg/Bool` | `True` when user clicks |
| Manual line color pick (when manual_line) | `/hide_and_seek/line_color` | `std_msgs/msg/String` | e.g. `"hue=220"` (degrees) |
| Manual driving | `/hide_and_seek/cmd_vel` | `geometry_msgs/msg/Twist` | standard v/w |

### Outputs (Robot → PC GUI)

| Purpose | Topic | ROS msg type | Payload schema |
|---------|-------|--------------|----------------|
| Robot status | `/hide_and_seek/status` | `std_msgs/msg/String` | Current state info |
| Trial results | `/hide_and_seek/trial_result` | `std_msgs/msg/String` | Trial outcome data |
| Sensor data | `/hide_and_seek/sensors` | `std_msgs/msg/String` | LiDAR, camera, etc. |
| Error messages | `/hide_and_seek/errors` | `std_msgs/msg/String` | Error notifications |

## Key Files Overview

### Robot Control
- `hide_and_seek.py` - Main robot control system with state machine
- `hide_and_seek_bridge.py` - ROS bridge for PC communication
- `pc_link.py` - PC-side communication interface

### Experiment Control
- `ktom_experimenter.py` - k-ToM experiment GUI (standalone tkinter app)
- `requirements.txt` - Python dependencies

### System Management
- `update_robot.sh` - Robot update/build script
- `start_pc_link.sh` - Start live link on robot
- `README.md` - System documentation

## Architecture Notes

### Robot State Machine (hide_and_seek.py)
- **States**: START → PICK_COLOR → FOLLOWING_LINE → WAITING_FOR_RAT → DISPENSING → TURNING → RETURNING_HOME → RESET → MANUAL_CONTROL
- **Line Following**: PID control with HSV color detection
- **Rat Detection**: LiDAR-based proximity detection with baseline comparison
- **Reward Dispensing**: Servo-controlled cheerio dispenser with count tracking

### k-ToM Experiment System (ktom_experimenter.py)
- **Levels**: k=0 (non-adaptive), k=1,2,3 (Theory of Mind)
- **Strategies**: Patterned vs percentage-based for k=0
- **Data Export**: CSV logs with timestamped folders
- **GUI**: Tabbed interface with setup and trial execution

### Communication Protocol
- **WebSocket**: Port 10090 for real-time PC-robot communication
- **ROS Topics**: Standard ROS2 message types
- **Bridge Pattern**: PC GUI ↔ Bridge ↔ Robot Control

## Development Workflow

1. **Local Development**: Work on PC-side files (ktom_experimenter.py, pc_link.py)
2. **Robot Updates**: Use `update_robot.sh` to deploy changes
3. **Live Testing**: Use `start_pc_link.sh` to establish connection
4. **Experiment Control**: Use ktom_experimenter.py GUI for trial management

## Key Dependencies
- **Robot**: ROS2, OpenCV, NumPy, SciPy
- **PC**: tkinter, NumPy, SciPy, rosbridge_suite
- **Communication**: WebSocket, ROS2 topics

## Common Tasks
- **Add new robot states**: Modify state machine in hide_and_seek.py
- **Change communication protocol**: Update both bridge.py and pc_link.py
- **Modify experiment parameters**: Edit ktom_experimenter.py GUI
- **Add new sensors**: Extend sensor processing in hide_and_seek.py
- **Deploy to robot**: Run update_robot.sh script
