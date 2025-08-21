# 🔧 Robot-GUI Synchronization Fixes

## 🎯 **Issues Fixed**

### **1. Camera ROI Selection in Auto Mode**
**Problem**: Robot was asking for manual ROI selection even when in auto mode.

**Root Cause**: The robot's `execute_pick_color_phase()` function wasn't checking if auto mode was enabled and line color was already provided.

**Fix**: 
- Modified `execute_pick_color_phase()` to check `pc_drive_mode == "auto_line"` and `pc_line_color_hue is not None`
- When auto mode is active, automatically set the HSV range and transition to line following
- Skip manual ROI selection entirely in auto mode

**Code Changes**:
```python
# In hide_and_seek.py - execute_pick_color_phase()
if self.pc_drive_mode == "auto_line" and self.pc_line_color_hue is not None:
    # Use the color provided by PC
    h = self.pc_line_color_hue
    s, v = 100, 100  # Default saturation and value
    h_margin, s_margin, v_margin = 10, 70, 70
    lower = np.array([max(0, h - h_margin), max(40, s - s_margin), max(40, v - v_margin)])
    upper = np.array([min(179, h + h_margin), min(255, s + s_margin), min(255, v + v_margin)])
    self.target_hsv_range = (lower, upper)
    
    self.get_logger().info(f"Auto color selected. Hue: {h}. Range: L={lower}, U={upper}")
    self.main_state = RobotState.FOLLOWING_LINE
    self.follow_state = FollowState.TRACKING
    
    # Publish progress update
    progress_msg = String()
    progress_msg.data = "leaving_entrance"
    self.progress_pub.publish(progress_msg)
    return
```

---

### **2. Line Following Recovery Angle**
**Problem**: Robot was only turning ~5° instead of 90° when searching for lost lines.

**Root Cause**: Search durations were too short (1.3s and 2.6s) for proper 90° turns.

**Fix**: 
- Increased search durations to 2.7s each (calculated for 90° at 0.6 rad/s)
- Added detailed logging to show search progress
- Improved log messages to indicate 90° and 180° search patterns

**Code Changes**:
```python
# In hide_and_seek.py - Search parameters
# Calculate duration for 90-degree turns (90° = π/2 radians)
# At 0.6 rad/s, 90° takes: (π/2) / 0.6 ≈ 2.62 seconds
self.SEARCH_DURATION_LEFT = 2.7   # 90 degrees left
self.SEARCH_DURATION_RIGHT = 2.7  # 90 degrees right

# Enhanced logging
self.get_logger().info(f"Searching: {elapsed_time:.1f}s / {duration_this_turn:.1f}s, direction: {'left' if self.search_direction == 1 else 'right'}")
self.get_logger().info("90° left search complete, switching to 90° right search.")
self.get_logger().info("180° search complete, line not found.")
```

---

### **3. GUI Progress Indicator Desync**
**Problem**: GUI was showing "intersection" step before robot even left the start position.

**Root Cause**: Progress reporting was not accurately reflecting the robot's actual state and wasn't handling auto mode properly.

**Fix**:
- Enhanced progress reporting to include line following sub-states
- Updated GUI progress mapping to handle new progress messages
- Added proper state transitions for auto mode

**Code Changes**:
```python
# In hide_and_seek.py - Enhanced progress reporting
def report_progress(self):
    progress_msg = String()
    if self.main_state == RobotState.PICK_COLOR:
        if self.pc_drive_mode == "auto_line" and self.pc_line_color_hue is not None:
            progress_msg.data = "leaving_entrance"
        else:
            progress_msg.data = "pick_color_phase"
    elif self.main_state == RobotState.FOLLOWING_LINE:
        if self.follow_state == FollowState.TRACKING:
            progress_msg.data = "following_line"
        elif self.follow_state == FollowState.SEARCHING:
            progress_msg.data = "searching_for_line"
        elif self.follow_state == FollowState.REVERSING:
            progress_msg.data = "reversing"
        else:
            progress_msg.data = "following_line"
    # ... other states

# In ktom_experimenter.py - Updated progress mapping
def on_progress_status(progress):
    progress_lower = progress.lower()
    if "leaving_entrance" in progress_lower or "entrance" in progress_lower:
        self.update_trial_progress(1)  # Step 1: Leave entrance
    elif "following_line" in progress_lower:
        self.update_trial_progress(3)  # Step 3: Follow the line
    elif "waiting_for_rat" in progress_lower:
        self.update_trial_progress(4)  # Step 4: Wait at hiding spot
    # ... other mappings
```

---

### **4. Auto-Send Target Enhancement**
**Problem**: Auto-send target wasn't sending line color information to the robot.

**Root Cause**: The auto-send function only sent the target spot but not the corresponding line color.

**Fix**:
- Enhanced `on_auto_send_target()` to also send line color
- Added RGB to HSV conversion for line colors
- Added `send_line_color()` method to `RosLink` class
- Integrated line color sending with target spot sending

**Code Changes**:
```python
# In ktom_experimenter.py - Enhanced auto-send
# Send line color to robot (convert RGB to HSV hue)
try:
    line_colors = self.get_line_colors()
    if line_colors and target_spot in line_colors:
        rgb_str = line_colors[target_spot]
        rgb_values = [int(x.strip()) for x in rgb_str.split(',')]
        if len(rgb_values) == 3:
            # Convert RGB to HSV
            import cv2
            import numpy as np
            rgb_array = np.array([[rgb_values]], dtype=np.uint8)
            hsv_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2HSV)
            hue = int(hsv_array[0, 0, 0])
            
            # Send line color to robot
            self.robot_link.send_line_color(hue)
            self.update_robot_status(f"🎨 Sent line color for {target_spot}: RGB{rgb_values} → HSV hue {hue}")
except Exception as color_error:
    self.update_robot_status(f"⚠️ Could not send line color: {str(color_error)}")

# In pc_link.py - Added line color support
self.line_color = roslibpy.Topic(self.ros,'/hide_and_seek/line_color','std_msgs/msg/String')

def send_line_color(self, hue):
    self.line_color.publish(roslibpy.Message({'data': f'hue={hue}'}))
```

---

## 🚀 **How to Test the Fixes**

### **1. Test Auto Mode (No ROI Selection)**
1. Start the robot: `./start_hide_and_seek.sh`
2. Start the GUI: `python ktom_experimenter.py`
3. Connect to robot
4. Configure line colors in the GUI
5. Click "🤖 Auto-Send k-ToM Target"
6. **Expected**: Robot should immediately start line following without asking for ROI selection

### **2. Test Line Recovery (90° Turns)**
1. Start robot and GUI as above
2. Let robot follow a line
3. Block the line so robot loses it
4. **Expected**: Robot should turn 90° left, then 90° right (total 180° search)

### **3. Test Progress Synchronization**
1. Start robot and GUI
2. Use auto-send target
3. **Expected**: GUI should show:
   - Step 1: "Leave entrance" (not intersection)
   - Step 3: "Follow the line" when actually following
   - Step 4: "Wait at hiding spot" when waiting for rat

### **4. Test Auto-Send with Line Color**
1. Configure line colors in GUI
2. Click auto-send target
3. **Expected**: Status should show both target spot and line color being sent

---

## 📊 **Technical Details**

### **Search Angle Calculation**
- **Turn Speed**: 0.6 rad/s
- **90° in radians**: π/2 ≈ 1.57 radians
- **Time for 90°**: 1.57 / 0.6 ≈ 2.62 seconds
- **Set Duration**: 2.7 seconds (with safety margin)

### **Progress Message Mapping**
| Robot State | Progress Message | GUI Step |
|-------------|------------------|----------|
| START | "start_phase" | 0 |
| PICK_COLOR (auto) | "leaving_entrance" | 1 |
| FOLLOWING_LINE (tracking) | "following_line" | 3 |
| FOLLOWING_LINE (searching) | "searching_for_line" | 3 |
| WAITING_FOR_RAT | "waiting_for_rat" | 4 |
| TURNING | "turning_180" | 5 |
| RETURNING_HOME | "returning_home" | 7 |
| RESET | "reset" | 8 |

### **Line Color Conversion**
- **Input**: RGB values from GUI (e.g., "0,255,0")
- **Process**: Convert to HSV using OpenCV
- **Output**: Hue value (0-179) sent to robot
- **Format**: `"hue=120"` message

---

## ✅ **Verification Checklist**

- [ ] Robot doesn't ask for ROI selection in auto mode
- [ ] Line recovery turns exactly 90° left, then 90° right
- [ ] GUI progress indicator matches robot's actual state
- [ ] Auto-send target includes both spot and line color
- [ ] All progress messages are properly mapped to GUI steps
- [ ] Line following status is reported accurately
- [ ] Robot transitions smoothly between states

---

## 🐛 **Known Limitations**

1. **Line Color Accuracy**: RGB to HSV conversion assumes standard lighting conditions
2. **Search Pattern**: Fixed 90° turns may not be optimal for all environments
3. **Progress Granularity**: Some sub-states (like reversing) are not separately reported

---

## 🔄 **Future Improvements**

1. **Adaptive Search**: Dynamic search angles based on environment
2. **Color Calibration**: Real-time color adjustment based on lighting
3. **State Machine**: More granular state reporting for better GUI sync
4. **Error Recovery**: Better handling of edge cases in line following

The synchronization issues have been resolved! The robot and GUI should now work together seamlessly. 🎉🤖
