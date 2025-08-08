import time
import keyboard
import numpy as np
import glfw
import mujoco
import mujoco.viewer
import itertools
dbg_counter = itertools.count()   # global step counter
DBG_SKIP = 10                     # print / overlay every N steps

# -----------------------------------------------------------------
# tuning constants
WHEEL_MAX_TORQUE       = 0.15
WHEEL_JOINT_DAMPING    = 0.009
DRIVE_SPEED            = 0.5
TURN_RATIO             = 1.0
CAMERA_STEP            = 0.05
TARGET_SPEED           = 0.3
KP_SPEED               = 1.0
HEADING_KP             = 2.0  # proportional gain for heading control
# ── NEW CONSTANTS ───────────────────────────────────────────────────────────────
SAFE_FRONT_DIST   = 0.2   # must keep the *front* beams above this
SAFE_SIDE_DIST    = 0.2   # any lidar below this → obstacle
ARRIVAL_TOLERANCE = 0.2   # stop this far from the beacon
# ── globals ────────────────────────────────────────────────────────────────────
last_debug_time = 0.0       # wall-clock time of previous print
# ── NEW / ADD ────────────────────────────────────────────────────────────────
DEBUG_PERIOD = 1.0      # seconds
_last_dbg    = 0.0
REVERSE_STEPS      = 200     # simulation steps to back up (~0.2 s if dt = 0.001 s)
REVERSE_SPEED      = -0.4    # wheel drive value for a gentle reverse

# ── NEW STATE FLAG ──────────────────────────────────────────────────────────────
obstacle_mode = False      # True while we are spinning to clear an obstacle
reverse_cnt = 0              # counts down while backing up
# friction triplets
NOMINAL_FRICTION = np.array([0.4, 0.002, 0.2], dtype=float)
ICE_FRICTION     = np.array([1e-5, 1e-5, 1e-5], dtype=float)

TURN_SPEED        = 1.0      # max wheel speed while turning
HEADING_TOLERANCE = 0.05     # rad; how close is “aligned”


WHEEL_GEOMS = ["zq_collision", "yq_collision", "yh_collision", "zh_collision"]
# -----------------------------------------------------------------

# load model
model = mujoco.MjModel.from_xml_path('main_scene.xml')
data  = mujoco.MjData(model)
# fixed navigation beacons (x, y)
BEACONS = {
    'a': model.site('pylon_A').id,
    'b': model.site('pylon_B').id,
    'c': model.site('pylon_C').id,
    'd': model.site('pylon_D').id,
}

# apply motor torque limits
for name in ["front_left_motor","front_right_motor","back_left_motor","back_right_motor"]:
    aid = model.actuator(name).id
    model.actuator_forcerange[aid] = np.array([-WHEEL_MAX_TORQUE, WHEEL_MAX_TORQUE])

# friction helper
def apply_friction(triplet: np.ndarray):
    for g in WHEEL_GEOMS:
        model.geom_friction[model.geom(g).id] = triplet
    model.geom_friction[model.geom('floor').id] = triplet
    mujoco.mj_forward(model, data)

# start with nominal friction
friction_enabled = True
apply_friction(NOMINAL_FRICTION)

# set wheel joint damping
for j in ["zq_Joint","yq_Joint","yh_Joint","zh_Joint"]:
    adr = model.jnt_dofadr[model.joint(j).id]
    model.dof_damping[adr] = WHEEL_JOINT_DAMPING

# pan-tilt joint limits
yaw_range   = model.jnt_range[model.joint('camera_yaw_joint').id]
pitch_range = model.jnt_range[model.joint('camera_pitch_joint').id]
target_yaw   = 0.0
target_pitch = 0.0

# state flags
autonomous_mode     = False
last_key_press_time = 0.0

# beacon state
beacon = None
last_beacon_time = 0.0
BEACON_DEBOUNCE = 0.3
axis_mode = None        # 'X' or 'Y' once a leg is chosen
# quaternion to yaw
def quat_to_yaw(quat):
    return np.arctan2(
        2*(quat[0]*quat[3] + quat[1]*quat[2]),
        1 - 2*(quat[2]**2 + quat[3]**2)
    )
def _axis_heading(dx, dy):
    """
    Given vector to beacon, return:
      desired_yaw  – one of {0, ±π/2, π}
      axis         – 'X' or 'Y'
    Chooses the axis with the larger remaining distance.
    """
    if abs(dx) >= abs(dy):           # go East/West first
        return (0 if dx >= 0 else np.pi), 'X'
    else:                            # go North/South first
        return (np.pi/2 if dy >= 0 else -np.pi/2), 'Y'

def _turn_cmd(err):
    """Spin in place toward beacon."""
    if err > 0:                             # beacon is to the left → CCW
        return  TURN_SPEED, -TURN_SPEED
    else:                                   # beacon to the right → CW
        return -TURN_SPEED,  TURN_SPEED


def autonomous_control(d):
    """
    Returns a tuple (left_cmd, right_cmd) – each in {+TURN_SPEED, -TURN_SPEED, DRIVE_SPEED}.
    Uses the global `beacon` (target x-y) and `obstacle_mode` flag.
    """

    global obstacle_mode, beacon, reverse_cnt, axis_mode
    if beacon is None:
        return 0.0, 0.0                        # nothing to do

    # ── 1.  LIDAR processing ---------------------------------------------------
    lidar    = d.sensordata[0:8].copy()
    fwd_arc  = min(lidar[[0, 1, 7]])           # tight 3-beam front cone
    any_hit  = (lidar < SAFE_SIDE_DIST).any()  # any beam closer than side limit

        # ── 2.  Pose & axis-aligned heading --------------------------------------
    global axis_mode
    pos  = d.sensordata[9:12]
    yaw  = quat_to_yaw(d.sensordata[12:16])
    dx, dy = beacon[0] - pos[0], beacon[1] - pos[1]
    dist = np.hypot(dx, dy)

    # Decide which axis we are tackling
    if axis_mode is None:
        desired_yaw, axis_mode = _axis_heading(dx, dy)
    else:
        # If we finished current axis, switch to the other one
        if (axis_mode == 'X' and abs(dx) < ARRIVAL_TOLERANCE) or \
           (axis_mode == 'Y' and abs(dy) < ARRIVAL_TOLERANCE):
            if axis_mode == 'X' and abs(dy) >= ARRIVAL_TOLERANCE:
                desired_yaw, axis_mode = _axis_heading(0, dy)
            elif axis_mode == 'Y' and abs(dx) >= ARRIVAL_TOLERANCE:
                desired_yaw, axis_mode = _axis_heading(dx, 0)
            else:                          # both axes done → arrived
                debug_out(0, 0, dist, 0, 0, "ARRIVED")
                return 0.0, 0.0
        else:
            desired_yaw = 0 if axis_mode == 'X' \
                             else np.pi/2
            if axis_mode == 'X' and dx < 0:
                desired_yaw = np.pi
            if axis_mode == 'Y' and dy < 0:
                desired_yaw = -np.pi/2

    # Compute heading error to that right-angle target
    err = (desired_yaw - yaw + np.pi) % (2*np.pi) - np.pi

    # ── 3.  Obstacle state machine --------------------------------------------
    global reverse_cnt

    # 3-A. If we are currently reversing: keep backing up until counter hits 0
    if reverse_cnt > 0:
        reverse_cnt -= 1
        left, right = -REVERSE_SPEED, -REVERSE_SPEED
        debug_out(fwd_arc, err, dist, left, right, "REVERSE")
        return left, right

    # 3-B. Normal obstacle logic
    if obstacle_mode:
        if fwd_arc > SAFE_FRONT_DIST and not any_hit:      # path clear again
            obstacle_mode = False
        else:                                              # spin to clear
            left, right = _turn_cmd(err)
            debug_out(fwd_arc, err, dist, left, right, "AVOID")
            return left, right
    else:
        if fwd_arc < SAFE_FRONT_DIST or any_hit:           # obstacle detected
            obstacle_mode = True
            reverse_cnt = REVERSE_STEPS                    # back up first
            left, right = -REVERSE_SPEED, -REVERSE_SPEED
            debug_out(fwd_arc, err, dist, left, right, "TRIGGER_REVERSE")
            return left, right

 
    # ── 4.  Nominal discrete driving ------------------------------------------
    if abs(err) > HEADING_TOLERANCE:                   # not aligned yet
        left, right = _turn_cmd(err)                   # spin toward beacon
        debug_out(fwd_arc, err, dist, left, right, "ALIGN")
        return left, right

    # aligned → full-speed forward
    left, right = DRIVE_SPEED, DRIVE_SPEED
    debug_out(fwd_arc, err, dist, left, right, "FORWARD")
    return left, right

# apply control
def set_actuators(drive, camera):
    data.ctrl[0] = -drive[0]
    data.ctrl[1] =  drive[1]
    data.ctrl[2] = -drive[0]
    data.ctrl[3] =  drive[1]
    data.ctrl[4] =  camera['yaw']
    data.ctrl[5] =  camera['pitch']
# ── replacement debug_out ──────────────────────────────────────────────────────
# ── NEW / REPLACE THIS WHOLE FUNCTION ───────────────────────────────────────
# ── new debug_out ─────────────────────────────────────────────────────────────
def debug_out(front_arc, ang_err, dist, left_cmd, right_cmd, note):
    """
    Log once per DEBUG_PERIOD:
      • front lidar distance (front_arc)
      • heading error to beacon (ang_err, deg)
      • straight-line distance to beacon (dist, m)
      • wheel commands before sign flip
      • state label
    """
    global _last_dbg
    now = time.time()
    if now - _last_dbg < DEBUG_PERIOD:
        return
    _last_dbg = now

    txt = (f"front_arc: {front_arc:.3f} m   "
           f"ang_err: {np.degrees(ang_err):+.1f}°   "
           f"dist: {dist:.2f} m\n"
           f"cmd L/R: {left_cmd:+.2f}  {right_cmd:+.2f}   "
           f"state: {note}")
    print(txt)

    if hasattr(viewer, "add_overlay"):
        if hasattr(viewer, "clear_overlays"):
            viewer.clear_overlays()
        viewer.add_overlay(
            mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
            "DEBUG",
            txt,
        )

# run viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Drive: ↑↓←→ | Camera: I,J,K,L | Space: auto | F: friction | B: beacon | Q: quit")
    while viewer.is_running():
        t0 = time.time()
        glfw.poll_events()
        win = glfw.get_current_context()

        # choose a target beacon with A / B / C / D
        for key in BEACONS.keys():
            if keyboard.is_pressed(key) and time.time() - last_key_press_time > 0.3:
                site_id = BEACONS[key]
                beacon = tuple(data.site_xpos[site_id][:2])  # x,y only
                autonomous_mode = True                       # jump straight into auto
                print(f"\nTarget set to {key.upper()} at {beacon}")
                last_key_press_time = time.time()


        # toggle autonomous mode
        if keyboard.is_pressed('space') and time.time()-last_key_press_time>0.3:
            autonomous_mode = not autonomous_mode
            print(f"Autonomous {'ON' if autonomous_mode else 'OFF'}")
            last_key_press_time=time.time()

        # toggle friction
        if keyboard.is_pressed('f') and time.time()-last_key_press_time>0.3:
            friction_enabled=not friction_enabled
            apply_friction(NOMINAL_FRICTION if friction_enabled else ICE_FRICTION)
            print(f"Friction {'ON' if friction_enabled else 'OFF'}")
            last_key_press_time=time.time()

        # determine drive_cmd
        if autonomous_mode:
            drive_cmd = autonomous_control(data)
        else:
            if keyboard.is_pressed('up'):
                drive_cmd=(DRIVE_SPEED,DRIVE_SPEED)
            elif keyboard.is_pressed('down'):
                drive_cmd=(-DRIVE_SPEED,-DRIVE_SPEED)
            elif keyboard.is_pressed('left'):
                drive_cmd=(-DRIVE_SPEED*TURN_RATIO,DRIVE_SPEED*TURN_RATIO)
            elif keyboard.is_pressed('right'):
                drive_cmd=(DRIVE_SPEED*TURN_RATIO,-DRIVE_SPEED*TURN_RATIO)
            else:
                drive_cmd=(0.0,0.0)

        # camera control
        if keyboard.is_pressed('j'): target_yaw+=CAMERA_STEP
        if keyboard.is_pressed('l'): target_yaw-=CAMERA_STEP
        if keyboard.is_pressed('i'): target_pitch+=CAMERA_STEP
        if keyboard.is_pressed('k'): target_pitch-=CAMERA_STEP
        target_yaw=np.clip(target_yaw,yaw_range[0],yaw_range[1])
        target_pitch=np.clip(target_pitch,pitch_range[0],pitch_range[1])

        set_actuators(drive_cmd,{'yaw':target_yaw,'pitch':target_pitch})
        mujoco.mj_step(model,data)
        viewer.sync()

        # pacing
        dt=model.opt.timestep-(time.time()-t0)
        if dt>0: time.sleep(dt)

        # quit
        if keyboard.is_pressed('q'): break
print("Simulation ended.")
