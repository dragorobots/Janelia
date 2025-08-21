# GUI Flashing and Button Fixes

## Issues Fixed

### 1. Trial Progress Indicator Flashing
**Problem**: The trial progress indicator was flashing through options 1-4 repeatedly after connecting to the robot.

**Root Cause**: The `on_progress_status` callback was being triggered multiple times per second with the same progress messages, causing the GUI to update the progress indicator continuously even when the step hadn't actually changed.

**Solution**: 
- Added state tracking with `_last_progress_step` to remember the current step
- Modified the progress callback to only update the GUI when the step actually changes
- Added progress state reset when connecting to prevent stale state from causing flashing

**Code Changes**:
```python
def on_progress_status(progress):
    # Only update if the progress actually changed to prevent flashing
    current_step = getattr(self, '_last_progress_step', -1)
    new_step = -1
    
    # Map progress messages to steps...
    
    # Only update if step actually changed
    if new_step != current_step:
        self._last_progress_step = new_step
        if new_step >= 0:
            self.update_trial_progress(new_step)
```

### 2. Missing "Start Trial" Button
**Problem**: The "Start Trial" button was not visible to the user, even though it was being created.

**Root Cause**: The button was being placed at the bottom of the entire parent frame using `side='bottom'`, which was causing layout conflicts and potentially hiding the button behind other elements.

**Solution**: 
- Moved the Start Trial button to be part of the right panel instead of the bottom of the entire frame
- This ensures the button is properly positioned within the Robot Control tab layout

**Code Changes**:
```python
# Before: Button at bottom of entire parent frame
start_trial_frame = ttk.Frame(parent)
start_trial_frame.pack(side='bottom', fill='x', padx=10, pady=10)

# After: Button in right panel
start_trial_frame = ttk.Frame(right_panel)
start_trial_frame.pack(fill='x', pady=10)
```

### 3. Progress State Reset on Connection
**Problem**: When connecting to the robot, any stale progress state could cause immediate flashing.

**Solution**: 
- Added progress state reset when successfully connecting to the robot
- This ensures a clean state for the progress tracking

**Code Changes**:
```python
# Reset progress tracking to prevent flashing
self._last_progress_step = -1
self.update_trial_progress(0)  # Reset to waiting state
```

## Testing

A test script `test_gui_fixes.py` has been created to verify these fixes:

1. **Start Trial Button Visibility**: Checks that the button exists, is visible, and has proper dimensions
2. **Progress Tracking State**: Verifies that the progress tracking state management works correctly
3. **Progress Updates**: Tests that progress updates work without causing flashing
4. **Tab Organization**: Confirms the Robot Control tab is in the correct position

## Expected Behavior After Fixes

1. **Trial Progress Indicator**: 
   - Should only update when the robot's progress actually changes
   - Should not flash or cycle through steps rapidly
   - Should reset to "waiting" state when connecting

2. **Start Trial Button**:
   - Should be visible in the right panel of the Robot Control tab
   - Should be large and green as intended
   - Should be positioned below the trial progress indicators

3. **Connection Behavior**:
   - When connecting to the robot, progress should reset to step 0
   - No flashing should occur during connection
   - Progress updates should only happen when robot state actually changes

## Files Modified

- `ktom_experimenter.py`: Main GUI fixes
- `test_gui_fixes.py`: Test script to verify fixes

## Usage

To test the fixes:

```bash
python test_gui_fixes.py
```

This will open the GUI and run automated tests, then keep the window open for manual inspection.

## Verification Steps

1. **Start the GUI**: `python ktom_experimenter.py`
2. **Navigate to Robot Control tab**: Should be the second tab
3. **Check Start Trial button**: Should be visible in the right panel, large and green
4. **Connect to robot**: Progress should reset to "waiting" without flashing
5. **Monitor progress**: Should only update when robot state actually changes
