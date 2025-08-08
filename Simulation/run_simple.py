import mujoco
import mujoco.viewer
import keyboard

# Load the new, simple model
try:
    model = mujoco.MjModel.from_xml_path('simple_car.xml')
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# --- Control Parameters ---
DRIVE_SPEED = 1.0
TURN_RATIO = 0.8

def set_drive_speeds(left_speed, right_speed):
    """Sets the speed for the wheel motors."""
    # Right side motors (indices 0, 2)
    data.ctrl[0] = right_speed
    data.ctrl[2] = right_speed
    
    # Left side motors (indices 1, 3) are inverted
    data.ctrl[1] = -left_speed
    data.ctrl[3] = -left_speed

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simple car simulation started. Use arrow keys to drive.")
    
    while viewer.is_running():
        left = 0.0
        right = 0.0

        if keyboard.is_pressed('up arrow'):
            left, right = DRIVE_SPEED, DRIVE_SPEED
        elif keyboard.is_pressed('down arrow'):
            left, right = -DRIVE_SPEED, -DRIVE_SPEED
        elif keyboard.is_pressed('left arrow'):
            left, right = -DRIVE_SPEED * TURN_RATIO, DRIVE_SPEED * TURN_RATIO
        elif keyboard.is_pressed('right arrow'):
            left, right = DRIVE_SPEED * TURN_RATIO, -DRIVE_SPEED * TURN_RATIO
            
        set_drive_speeds(left, right)
        
        mujoco.mj_step(model, data)
        viewer.sync()
            
        if keyboard.is_pressed('q'):
            break

print("\nSimulation ended.")