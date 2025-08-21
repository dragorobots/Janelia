# Line Following and Intersection Detection Fixes

## Problem Summary

The robot was skipping the initial line following phase and jumping directly to the intersection, causing it to think it was already at the intersection when the trial started. The robot needed to:

1. **Follow the start line** from the entrance to the intersection
2. **Detect when it reaches the intersection** 
3. **Perform centering maneuvers** at the intersection (right 2.7s, left 5.5s, right 2.7s)
4. **Follow the target line** to the hiding spot
5. **Handle hiding spot selection** (auto or manual based on toggle)

## Fixes Implemented

### 1. Enhanced State Machine

**File: `hide_and_seek.py`**

Added new states to properly separate the different line following phases:

```python
class RobotState:
    START = 0
    PICK_COLOR = 1
    FOLLOWING_START_LINE = 2  # NEW: Follow start line to intersection
    AT_INTERSECTION = 3       # NEW: At intersection, perform centering
    FOLLOWING_TARGET_LINE = 4 # NEW: Follow target line to hiding spot
    WAITING_FOR_RAT = 5
    TURNING = 6
    RETURNING_HOME = 7
    RESET = 8
    MANUAL_CONTROL = 9
```

### 2. Dual Line Color Support

**File: `hide_and_seek.py`**

Added support for tracking both start line and target line colors:

```python
# NEW: HSV range for start line
self.start_line_hsv_range = None  
# Existing: HSV range for target line  
self.target_hsv_range = None
```

### 3. Intersection Detection Logic

**File: `hide_and_seek.py`**

Added logic to detect when the robot reaches the intersection by looking for the target line color while following the start line:

```python
# Check for intersection detection (when following start line)
if self.main_state == RobotState.FOLLOWING_START_LINE and not self.intersection_detected:
    # Look for the target line color in the frame to detect intersection
    target_mask = cv2.inRange(hsv_frame, self.target_hsv_range[0], self.target_hsv_range[1])
    target_contours, _ = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # If we detect the target line color, we've reached the intersection
    if any(cv2.contourArea(contour) > self.min_line_area for contour in target_contours):
        self.get_logger().info("INTERSECTION DETECTED! Starting centering maneuver.")
        self.intersection_detected = True
        self.main_state = RobotState.AT_INTERSECTION
        self.centering_phase = 0
        self.action_start_time = time.time()
        self.stop_robot()
```

### 4. Intersection Centering Maneuver

**File: `hide_and_seek.py`**

Added the `execute_intersection_centering()` method to perform the required centering sequence:

```python
def execute_intersection_centering(self, frame):
    """Execute the centering maneuver at the intersection"""
    # Phase 0: Right turn for 2.7s
    # Phase 1: Left turn for 5.5s  
    # Phase 2: Right turn for 2.7s
    # Phase 3: Complete, start following target line
```

**Centering Sequence:**
- **Right turn**: 2.7 seconds at 0.6 rad/s
- **Left turn**: 5.5 seconds at -0.6 rad/s  
- **Right turn**: 2.7 seconds at 0.6 rad/s
- **Complete**: Transition to following target line

### 5. Enhanced Line Following Logic

**File: `hide_and_seek.py`**

Updated `execute_line_follow()` to handle different line colors based on current state:

```python
# Determine which line color to follow based on current state
if self.main_state == RobotState.FOLLOWING_START_LINE:
    current_hsv_range = self.start_line_hsv_range
elif self.main_state == RobotState.FOLLOWING_TARGET_LINE:
    current_hsv_range = self.target_hsv_range
```

### 6. Updated Progress Tracking

**File: `hide_and_seek.py`**

Added new progress messages for the enhanced state machine:

```python
elif self.main_state == RobotState.FOLLOWING_START_LINE:
    progress_msg.data = "following_start_line"
elif self.main_state == RobotState.AT_INTERSECTION:
    progress_msg.data = "at_intersection_centering"
elif self.main_state == RobotState.FOLLOWING_TARGET_LINE:
    progress_msg.data = "following_target_line"
```

### 7. K-ToM Experimenter Integration

**File: `ktom_experimenter.py`**

Added line color topic support for sending target line colors to the robot:

```python
# NEW: Line color topic
self.line_color = roslibpy.Topic(self.ros,'/hide_and_seek/line_color','std_msgs/msg/String')

def send_line_color(self, hue):
    if self.connected:
        self.line_color.publish(roslibpy.Message({'data': f'hue={hue}'}))
```

## Expected Behavior After Fixes

### Trial Flow:
1. **START**: Robot waits for trial start command
2. **PICK_COLOR**: Robot selects line colors (auto or manual)
3. **FOLLOWING_START_LINE**: Robot follows start line (red) to intersection
4. **AT_INTERSECTION**: Robot performs centering maneuver
5. **FOLLOWING_TARGET_LINE**: Robot follows target line to hiding spot
6. **WAITING_FOR_RAT**: Robot waits for rat detection
7. **TURNING**: Robot turns 180° after rat found or timeout
8. **RETURNING_HOME**: Robot follows line back to start
9. **RESET**: Robot resets for next trial

### Key Improvements:
- ✅ **No more skipped line following**: Robot properly follows start line first
- ✅ **Intersection detection**: Robot detects when it reaches the intersection
- ✅ **Centering maneuver**: Robot performs the required turning sequence
- ✅ **Proper state transitions**: Clear separation between different phases
- ✅ **Progress tracking**: Accurate progress updates for GUI
- ✅ **Line color handling**: Support for both start and target line colors

## Testing

Run the test script to verify all fixes are properly implemented:

```bash
python test_line_following_fixes.py
```

This will test:
- State machine updates
- Intersection detection logic
- Line color handling
- K-tom integration
- Progress tracking updates

## Configuration Notes

### Start Line Color
Currently hardcoded to red (hue 0) in auto mode. This should be made configurable in the future.

### Target Line Color
Set via the k-tom experimenter GUI when using predefined colors or auto-send functionality.

### Centering Durations
The centering maneuver durations are:
- Right turn: 2.7 seconds
- Left turn: 5.5 seconds  
- Right turn: 2.7 seconds

These can be adjusted in the `execute_intersection_centering()` method if needed.
