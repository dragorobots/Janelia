# Line Following Robustness Fixes

## Issues Addressed

### 1. **Robot Publishing Line Following Status Before Trial Start**
**Problem**: The robot was publishing "Line Follow: following" and "Line Follow: searching" messages immediately when `./start_hide_and_seek.sh` was run, even though the trial hadn't started yet. It was also publishing line following status when in idle states like `PICK_COLOR`.

**Solution**: Modified `publish_status_updates()` in `hide_and_seek.py` to only publish line following status when the robot is actually in line following states (`FOLLOWING_START_LINE`, `FOLLOWING_HIDING_LINE`, `RETURNING_HOME`) AND the `follow_state` is not `STOPPED`. Also ensured that when transitioning to non-line-following states, the `follow_state` is properly set to `STOPPED`.

**Code Changes**:
```python
# Only publish line follow status if we're actually line following
if (self.main_state in [RobotState.FOLLOWING_START_LINE, RobotState.FOLLOWING_HIDING_LINE, RobotState.RETURNING_HOME] and 
    self.follow_state != FollowState.STOPPED):
    # ... publish line status
# Don't publish line follow status when not line following (e.g., in START state)

# In start_trial():
self.follow_state = FollowState.STOPPED  # Ensure line following is stopped

# In execute_manual_control():
self.follow_state = FollowState.STOPPED  # Ensure line following is stopped
```

### 2. **Missing Corrective Turn After Line Following Failures**
**Problem**: When line following failed, the robot would immediately transition to the next state without making a corrective turn to try to get back to the midline.

**Solution**: Implemented the correct line following recovery pattern. When a line is lost, the robot follows this sequence:
1. Reverse for 0.5 seconds
2. Turn left for 2.7 seconds (searching for line)
3. Turn right for 5.4 seconds (searching for line)
4. **Line officially lost** - Turn left for 2.7 seconds (corrective turn to return to center, not searching)
5. Then transition to the next state

**Code Changes**:
```python
class FollowState:
    TRACKING = 0
    REVERSING = 1
    SEARCHING = 2
    STOPPED = 3
    CORRECTIVE_TURN = 4  # Added for corrective turn

# Search durations:
self.SEARCH_DURATION_LEFT = 2.7   # Left search duration
self.SEARCH_DURATION_RIGHT = 5.4  # Right search duration

# In execute_line_follow():
elif self.follow_state == FollowState.SEARCHING:
    # ... search logic ...
    else:
        if self.search_direction == 1:
            # Left search complete, switch to right search
            self.search_direction = -1
            self.action_start_time = time.time()
        else:
            # Right search complete, line not found
            # Line is officially lost - always do corrective turn to return to center
            self.get_logger().info("Line officially lost. Making corrective turn to left for 2.7s to return to center.")
            self.follow_state = FollowState.CORRECTIVE_TURN
            self.action_start_time = time.time()

elif self.follow_state == FollowState.CORRECTIVE_TURN:
    # Corrective turn to the left for 2.7s after line following failure
    elapsed_time = time.time() - self.action_start_time
    if elapsed_time < 2.7:
        twist.angular.z = self.SEARCH_TURN_SPEED  # Turn left
        self.cmd_vel_pub.publish(twist)
    else:
        self.stop_robot()
        self.handle_line_ended()
```

### 3. **GUI Trial Progress Not Driven by Robot Status**
**Problem**: The GUI's trial progress was being updated based on line following status messages rather than the robot's actual progress messages, causing incorrect progress indication.

**Solution**: Modified `on_connect_robot()` in `ktom_experimenter.py` to remove the old logic that updated progress based on line following status. Now the progress is only updated based on the robot's progress messages.

**Code Changes**:
```python
def on_line_follow_status(status):
    self.update_robot_status(f"Line Follow: {status}")
    # Don't update progress based on line following status - only use progress messages

def on_rat_detection_status(found):
    self.update_robot_status(f"Rat Detection: {'Found' if found else 'Not Found'}")
    # Don't update progress based on rat detection - only use progress messages
```

### 4. **Return Journey Line Switching Error**
**Problem**: During the return journey, when switching from the hiding line back to the start line, there was no error handling if the start line HSV range was `None`.

**Solution**: Enhanced `handle_line_ended()` method to include robust error handling for line switching during the return journey.

**Code Changes**:
```python
elif self.main_state == RobotState.RETURNING_HOME:
    if self.current_line_hsv_range == self.hiding_line_hsv_range:
        # We were following hiding line, now switch to start line
        if self.start_line_hsv_range is not None:
            self.current_line_hsv_range = self.start_line_hsv_range
            self.line_lost_count = 0
            self.follow_state = FollowState.TRACKING
            self.get_logger().info("Successfully switched to start line for return journey.")
        else:
            self.get_logger().error("Start line HSV range is None! Cannot switch lines.")
            # Fallback: try to continue with current line
            self.line_lost_count = 0
            self.follow_state = FollowState.TRACKING
```

## Expected Behavior After Fixes

### Robot Startup
- When `./start_hide_and_seek.sh` is run, the robot should only show "waiting for start trial" status
- No line following status messages should be published until the trial actually starts
- No line following status messages should be published when in `PICK_COLOR` state
- Trial progress should show step 0 (waiting for start)

### Line Following Robustness
- When line following fails (line lost), the robot will:
  1. Reverse for 0.5 seconds
  2. Turn left for 2.7 seconds (searching for line)
  3. Turn right for 5.4 seconds (searching for line)
  4. **Line officially lost** - Turn left for 2.7 seconds (corrective turn to return to center, not searching)
  5. Then transition to the next state

### Trial Progress Accuracy
- Trial progress is now driven entirely by robot progress messages
- Progress updates only occur when the robot actually transitions between states
- No more incorrect progress indication based on line following status

### Return Journey Robustness
- When switching from hiding line to start line during return, the robot checks if the start line HSV range exists
- If the start line HSV range is missing, it logs an error and continues with the current line
- Successful line switching is logged for debugging

## Files Modified

1. **`hide_and_seek.py`**:
   - Added `CORRECTIVE_TURN` state to `FollowState`
   - Modified `publish_status_updates()` to control line following status publishing with follow_state check
   - Enhanced `execute_line_follow()` with corrective turn logic
   - Improved `handle_line_ended()` with robust error handling
   - Ensured `follow_state` is set to `STOPPED` in `start_trial()` and `execute_manual_control()`

2. **`ktom_experimenter.py`**:
   - Modified `on_connect_robot()` to remove old progress update logic
   - Ensured progress is only driven by robot progress messages

3. **`test_line_following_fixes.py`**:
   - Updated test script to verify all fixes are properly implemented

## Testing

Run the test script to verify all fixes:
```bash
python test_line_following_fixes.py
```

All tests should pass, confirming that:
- ✅ Corrective turn state and logic are implemented correctly
- ✅ Line following follows the correct pattern: reverse → search left → search right → corrective turn
- ✅ Progress messaging is properly controlled
- ✅ GUI progress is driven by robot messages only
- ✅ Return journey robustness is improved
- ✅ Idle state line following status publishing is fixed
