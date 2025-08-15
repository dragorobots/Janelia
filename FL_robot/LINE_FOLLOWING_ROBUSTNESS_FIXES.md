# Line Following Robustness Fixes

## Issues Addressed

### 1. **Robot Publishing Line Following Status Before Trial Start**
**Problem**: The robot was publishing "Line Follow: following" and "Line Follow: searching" messages immediately when `./start_hide_and_seek.sh` was run, even though the trial hadn't started yet.

**Solution**: Modified `publish_status_updates()` in `hide_and_seek.py` to only publish line following status when the robot is actually in line following states (`FOLLOWING_START_LINE`, `FOLLOWING_HIDING_LINE`, `RETURNING_HOME`). Removed the "idle" status publishing.

**Code Changes**:
```python
# Only publish line follow status if we're actually line following
if self.main_state in [RobotState.FOLLOWING_START_LINE, RobotState.FOLLOWING_HIDING_LINE, RobotState.RETURNING_HOME]:
    # ... publish line status
# Don't publish line follow status when not line following (e.g., in START state)
```

### 2. **Missing Corrective Turn After Line Following Failures**
**Problem**: When line following failed, the robot would immediately transition to the next state without making a corrective turn to try to get back to the midline.

**Solution**: Added a new `CORRECTIVE_TURN` state to `FollowState` and implemented corrective turn logic. After line following fails (after `MAX_LINE_LOSSES`), the robot now makes a 2.7-second turn to the left before ending the line following phase.

**Code Changes**:
```python
class FollowState:
    TRACKING = 0
    REVERSING = 1
    SEARCHING = 2
    STOPPED = 3
    CORRECTIVE_TURN = 4  # Added for corrective turn

# In execute_line_follow():
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
- Trial progress should show step 0 (waiting for start)

### Line Following Robustness
- When line following fails (line lost for `MAX_LINE_LOSSES` times), the robot will:
  1. Reverse for 0.5 seconds
  2. Turn right for 2.7 seconds
  3. Turn left for 5.5 seconds
  4. Turn right for 2.7 seconds
  5. **NEW**: Make a corrective turn to the left for 2.7 seconds
  6. Then transition to the next state

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
   - Modified `publish_status_updates()` to control line following status publishing
   - Enhanced `execute_line_follow()` with corrective turn logic
   - Improved `handle_line_ended()` with robust error handling

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
- ✅ Corrective turn state and logic are implemented
- ✅ Progress messaging is properly controlled
- ✅ GUI progress is driven by robot messages only
- ✅ Return journey robustness is improved
