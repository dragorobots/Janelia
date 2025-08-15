# Line Following and Progress Fixes

## Issues Fixed

### 1. Line Following Search Turning Too Conservative
**Problem**: When the robot loses the line, it was only turning 90° left and 90° right, which wasn't aggressive enough to find the line.

**Root Cause**: The `SEARCH_DURATION_RIGHT` was set to 2.7 seconds (90°), the same as the left turn.

**Solution**: 
- Increased `SEARCH_DURATION_RIGHT` from 2.7s to 8.1s (270°)
- This provides 3x more turning to the right as requested
- Total search pattern: 90° left + 270° right = 360° total search

**Code Changes**:
```python
# Before
self.SEARCH_DURATION_LEFT = 2.7   # 90 degrees left
self.SEARCH_DURATION_RIGHT = 2.7  # 90 degrees right

# After  
self.SEARCH_DURATION_LEFT = 2.7   # 90 degrees left
self.SEARCH_DURATION_RIGHT = 8.1  # 270 degrees right (3x more turning)
```

### 2. Progress Indicator Flickering
**Problem**: The trial progress indicator was flashing through steps 1-4 repeatedly after connecting to the robot.

**Root Cause**: The robot was publishing progress messages every 0.1 seconds in the main loop, even when the state hadn't changed.

**Solution**: 
- Added `last_published_progress` tracking variable
- Modified progress publishing to only send messages when the state actually changes
- This prevents continuous publishing of the same progress message

**Code Changes**:
```python
# Added tracking variable
self.last_published_progress = None

# Modified publishing logic
if self.last_published_progress != progress_msg.data:
    self.progress_pub.publish(progress_msg)
    self.last_published_progress = progress_msg.data
```

### 3. Progress Jumping to Wrong Step
**Problem**: When starting a trial, the progress indicator immediately jumped to "wait at hiding spot" (step 4) instead of properly progressing through the steps.

**Root Cause**: The progress message mapping in the GUI was incorrect, and the robot was publishing multiple progress messages rapidly during state transitions.

**Solution**: 
- Improved progress message mapping in the GUI
- Added proper filtering for "searching_for_line" messages
- Fixed the step progression logic

**Code Changes**:
```python
# Improved mapping in ktom_experimenter.py
elif "trial_started" in progress_lower:
    new_step = 1  # Trial started - beginning line following
elif "leaving_entrance" in progress_lower or "entrance" in progress_lower:
    new_step = 1  # Step 1: Leave entrance (line following)
elif "following_line" in progress_lower:
    new_step = 3  # Step 3: Follow the line
elif "searching_for_line" in progress_lower:
    new_step = 3  # Step 3: Still in line following phase
```

## Technical Details

### Line Following Search Pattern
- **Left Turn**: 2.7 seconds at 0.6 rad/s = 90°
- **Right Turn**: 8.1 seconds at 0.6 rad/s = 270°
- **Total Search**: 360° coverage (90° + 270°)
- **Ratio**: Right turn is exactly 3x longer than left turn

### Progress Message Flow
1. **waiting_for_start** → Step 0: Waiting for trial to begin
2. **trial_started** → Step 1: Trial started, beginning line following
3. **leaving_entrance** → Step 1: Leave entrance (line following)
4. **following_line** → Step 3: Follow the line
5. **searching_for_line** → Step 3: Still in line following phase
6. **intersection** → Step 2: Reach intersection
7. **waiting_for_rat** → Step 4: Wait at hiding spot
8. **turning_180** → Step 5: Wait 10s, turn 180°
9. **returning_home** → Step 7: Return to start
10. **reset** → Step 8: Wait for new command

### State Tracking Improvements
- **Robot Side**: Added `last_published_progress` to prevent duplicate messages
- **GUI Side**: Added `_last_progress_step` to prevent unnecessary updates
- **Result**: Progress indicator only updates when robot state actually changes

## Testing

A test script `test_line_following_fixes.py` has been created to verify these fixes:

```bash
python test_line_following_fixes.py
```

This script tests:
1. **Turning Durations**: Verifies the 3x ratio between left and right turns
2. **Progress Mapping**: Tests all progress message mappings
3. **Flickering Fix**: Confirms the state tracking improvements

## Expected Behavior After Fixes

### Line Following Search
- Robot loses line → turns 90° left (2.7s)
- No line found → turns 270° right (8.1s) 
- Total search time: ~10.8 seconds
- Much more aggressive search pattern

### Progress Indicator
- **No more flickering**: Progress only updates when state changes
- **Accurate progression**: Proper step-by-step advancement
- **Correct mapping**: Trial start properly goes from step 0 → 1 → 3

### Trial Start Sequence
1. **Step 0**: Waiting for start
2. **Step 1**: Trial started, beginning line following
3. **Step 3**: Following line (or searching if line lost)
4. **Step 4**: Only when actually waiting for rat at hiding spot

## Files Modified

- `hide_and_seek.py`: Line following search durations and progress tracking
- `ktom_experimenter.py`: Progress message mapping improvements
- `test_line_following_fixes.py`: Test script for verification

## Verification Steps

1. **Test line following**: Start a trial and let the robot lose the line
2. **Observe search pattern**: Should turn 90° left, then 270° right
3. **Check progress indicator**: Should not flicker and progress correctly
4. **Verify trial start**: Should go step 0 → 1 → 3, not jump to step 4
