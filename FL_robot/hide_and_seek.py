#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt16, Int32, String, Bool
import cv2
import numpy as np
import time
import os
import math
import sys, select, termios, tty

# --- Constants and Configuration ---
COUNT_FILE = os.path.expanduser("~/cheerio_count.txt")

MANUAL_CONTROL_MSG = """
_____________________

RESET ROBOT FOR NEXT TRIAL
_____________________

---------------------------
Entering Manual Control Mode
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/,    = drive forward/backward
u/o  = forward veer left/right
j/l    = turn in place left/right
m/.  = backward veer left/right

s : Start next trial
space key, k : force stop
CTRL-C to quit
"""

# State Machine for the main Hide and Seek task
class RobotState:
    START = 0
    PICK_COLOR = 1
    FOLLOWING_LINE = 2
    WAITING_FOR_RAT = 3
    DISPENSING = 4
    TURNING = 5
    RETURNING_HOME = 6
    RESET = 7
    MANUAL_CONTROL = 8

# State machine for the more detailed line-following logic
class FollowState:
    TRACKING = 0
    REVERSING = 1
    SEARCHING = 2
    STOPPED = 3

class HideAndSeekNode(Node):
    """
    Master ROS2 Node to control the robot's Hide and Seek behavior.
    """
    def __init__(self):
        super().__init__('hide_and_seek_node')
        self.get_logger().info("Initializing Hide and Seek Node...")

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buzzer_pub = self.create_publisher(UInt16, '/beep', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.servo1_pub = self.create_publisher(Int32, '/servo_s1', 10) # Camera Yaw
        self.servo2_pub = self.create_publisher(Int32, '/servo_s2', 10) # Dispenser
        
        # --- PC Communication Subscribers ---
        self.target_spot_sub = self.create_subscription(Int32, '/hide_and_seek/target_spot', self.target_spot_callback, 10)
        self.toggles_sub = self.create_subscription(String, '/hide_and_seek/toggles', self.toggles_callback, 10)
        self.cmd_vel_pc_sub = self.create_subscription(Twist, '/hide_and_seek/cmd_vel', self.cmd_vel_pc_callback, 10)
        self.manual_found_sub = self.create_subscription(Bool, '/hide_and_seek/manual_found', self.manual_found_callback, 10)
        self.line_color_sub = self.create_subscription(String, '/hide_and_seek/line_color', self.line_color_callback, 10)
        
        # --- PC Communication Publishers ---
        self.line_follow_status_pub = self.create_publisher(String, '/line_follow/status', 10)
        self.rat_detection_pub = self.create_publisher(Bool, '/rat_detection/found', 10)
        self.progress_pub = self.create_publisher(String, '/hide_and_seek/progress', 10)

        # --- Main Robot State ---
        self.main_state = RobotState.START
        
        # --- Line Following Logic and State ---
        self.follow_state = FollowState.STOPPED
        self.target_hsv_range = None
        self.action_start_time = None
        self.search_direction = 1

        # --- Parameters ---
        self.linear_speed = 0.12
        self.pid_controller = simplePID(p=[0.03, 0], i=[0.0, 0], d=[0.03, 0])
        self.min_radius_threshold = 40
        self.camera_offset = 0
        self.img_flip = True
        self.REVERSE_SPEED = -0.1
        self.REVERSE_DURATION = 0.33
        self.SEARCH_TURN_SPEED = 0.6
        self.SEARCH_DURATION_LEFT = 1.3
        self.SEARCH_DURATION_RIGHT = 2.6
        self.initial_camera_yaw = 0
        self.initial_dispenser_angle = 0
        self.turn_duration = 7.5
        self.WAIT_DURATION = 120.0

        # --- LiDAR Parameters (from successful diagnostic) ---
        self.rat_detected = False
        self.lidar_baseline = None
        self.LIDAR_GRACE_PERIOD = 3.0
        self.LIDAR_PROXIMITY_LIMIT = 0.15
        self.LIDAR_CHANGE_THRESHOLD = 0.20
        self.LIDAR_PERSISTENCE_COUNT = 3
        self.LIDAR_MIN_POINTS_CHANGED = 20
        self.lidar_ready_time = None
        self.lidar_detection_counter = 0

        # --- OpenCV and Camera ---
        self.capture = cv2.VideoCapture(0)
        self.cv_window_name = "Robot View"
        cv2.namedWindow(self.cv_window_name)
        cv2.setMouseCallback(self.cv_window_name, self.mouse_callback)
        
        self.selecting_box = False
        self.box_start_point = (0, 0)
        self.box_end_point = (0, 0)

        # --- Manual Control ---
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except (termios.error, OSError):
            # Handle non-interactive environments (like when running as ROS2 node)
            self.settings = None
        self.manual_speed = 0.2
        self.manual_turn = 1.0
        
        # --- PC Communication Variables ---
        self.pc_target_spot = 0
        self.pc_drive_mode = "auto_line"  # auto_line, manual_line, manual_drive
        self.pc_rat_mode = "auto"  # auto, manual
        self.pc_manual_found = False
        self.pc_line_color_hue = None
        self.pc_override_active = False

        # --- Main Control Loop ---
        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("Node initialized. Starting in START state.")

    # --- PC Communication Callbacks ---
    def target_spot_callback(self, msg):
        """Handle target spot selection from PC"""
        self.pc_target_spot = msg.data
        self.get_logger().info(f"PC set target spot: {self.pc_target_spot}")
        
        # Publish progress update
        progress_msg = String()
        progress_msg.data = f"target_spot_set:{self.pc_target_spot}"
        self.progress_pub.publish(progress_msg)

    def toggles_callback(self, msg):
        """Handle toggle commands from PC"""
        toggle_str = msg.data
        self.get_logger().info(f"PC toggle: {toggle_str}")
        
        if toggle_str.startswith("drive_mode="):
            self.pc_drive_mode = toggle_str.split("=")[1]
            self.get_logger().info(f"PC drive mode: {self.pc_drive_mode}")
            
        elif toggle_str.startswith("rat_mode="):
            self.pc_rat_mode = toggle_str.split("=")[1]
            self.get_logger().info(f"PC rat mode: {self.pc_rat_mode}")
            
        elif toggle_str == "manual_found=true":
            self.pc_manual_found = True
            self.get_logger().info("PC manual 'rat found' signal")

    def cmd_vel_pc_callback(self, msg):
        """Handle velocity commands from PC"""
        if self.pc_drive_mode == "manual_drive":
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f"PC manual drive: linear={msg.linear.x}, angular={msg.angular.z}")

    def manual_found_callback(self, msg):
        """Handle manual found signal from PC"""
        if msg.data:
            self.pc_manual_found = True
            self.get_logger().info("PC manual 'rat found' signal")

    def line_color_callback(self, msg):
        """Handle line color selection from PC"""
        color_str = msg.data
        if color_str.startswith("hue="):
            try:
                self.pc_line_color_hue = int(color_str.split("=")[1])
                self.get_logger().info(f"PC line color hue: {self.pc_line_color_hue}")
                
                # Publish progress update
                progress_msg = String()
                progress_msg.data = f"line_color_set:{self.pc_line_color_hue}"
                self.progress_pub.publish(progress_msg)
            except ValueError:
                self.get_logger().error(f"Invalid PC hue value: {color_str}")

    def getKey(self):
        if self.settings is None:
            # Non-interactive environment, return empty key
            return ''
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except (termios.error, OSError):
            # Handle any terminal errors gracefully
            return ''

    def main_loop(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("Failed to get frame from camera.")
            return

        frame = cv2.resize(frame, (640, 480))
        if self.img_flip:
            frame = cv2.flip(frame, 1)
        
        display_view = frame.copy()

        if self.main_state == RobotState.START:
            self.execute_start_phase()
        elif self.main_state == RobotState.PICK_COLOR:
            self.execute_pick_color_phase(display_view)
        elif self.main_state in [RobotState.FOLLOWING_LINE, RobotState.RETURNING_HOME]:
            mask_display = self.execute_line_follow(display_view, returning=(self.main_state == RobotState.RETURNING_HOME))
            if mask_display is not None:
                display_view = cv2.hconcat([display_view, mask_display])
        elif self.main_state == RobotState.WAITING_FOR_RAT:
            self.execute_wait_for_rat(display_view)
        elif self.main_state == RobotState.DISPENSING:
            self.execute_dispense_phase()
        elif self.main_state == RobotState.TURNING:
            self.execute_turn_phase(display_view)
        elif self.main_state == RobotState.RESET:
            self.execute_reset_phase()
        elif self.main_state == RobotState.MANUAL_CONTROL:
            self.execute_manual_control()

        # Publish status updates to PC
        self.publish_status_updates()
        
        cv2.imshow(self.cv_window_name, display_view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def publish_status_updates(self):
        """Publish current status to PC"""
        # Publish line follow status
        line_status_msg = String()
        if self.pc_drive_mode == "manual_drive":
            line_status_msg.data = "manual_drive_mode"
        elif self.pc_drive_mode == "manual_line":
            line_status_msg.data = "manual_line_mode"
        elif self.main_state == RobotState.FOLLOWING_LINE:
            if self.follow_state == FollowState.TRACKING:
                line_status_msg.data = "following"
            elif self.follow_state == FollowState.SEARCHING:
                line_status_msg.data = "searching"
            elif self.follow_state == FollowState.REVERSING:
                line_status_msg.data = "reversing"
            else:
                line_status_msg.data = "stopped"
        else:
            line_status_msg.data = "idle"
        self.line_follow_status_pub.publish(line_status_msg)
        
        # Publish progress updates
        progress_msg = String()
        if self.main_state == RobotState.START:
            progress_msg.data = "start_phase"
        elif self.main_state == RobotState.PICK_COLOR:
            progress_msg.data = "pick_color_phase"
        elif self.main_state == RobotState.FOLLOWING_LINE:
            progress_msg.data = "following_line"
        elif self.main_state == RobotState.WAITING_FOR_RAT:
            progress_msg.data = "waiting_for_rat"
        elif self.main_state == RobotState.DISPENSING:
            progress_msg.data = "dispensing"
        elif self.main_state == RobotState.TURNING:
            progress_msg.data = "turning"
        elif self.main_state == RobotState.RETURNING_HOME:
            progress_msg.data = "returning_home"
        elif self.main_state == RobotState.RESET:
            progress_msg.data = "reset"
        elif self.main_state == RobotState.MANUAL_CONTROL:
            progress_msg.data = "manual_control"
        self.progress_pub.publish(progress_msg)

    def execute_start_phase(self):
        self.get_logger().info("STATE: START - Signaling trial start.")
        self.get_logger().info("Centering camera and setting dispenser to zero position.")
        s1_msg = Int32(); s1_msg.data = self.initial_camera_yaw
        self.servo1_pub.publish(s1_msg)
        s2_msg = Int32(); s2_msg.data = self.initial_dispenser_angle
        self.servo2_pub.publish(s2_msg)
        time.sleep(1.0)

        for _ in range(3):
            self.buzzer_pub.publish(UInt16(data=1)); time.sleep(0.4)
            self.buzzer_pub.publish(UInt16(data=0)); time.sleep(0.4)
        
        twist = Twist(); twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist); time.sleep(1)
        twist.linear.x = -0.1
        self.cmd_vel_pub.publish(twist); time.sleep(1)
        self.stop_robot()
        
        self.main_state = RobotState.PICK_COLOR
        self.get_logger().info("Transitioning to PICK_COLOR state.")

    def execute_pick_color_phase(self, frame):
        cv2.putText(frame, "Click and drag to select the line color", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if self.selecting_box:
            cv2.rectangle(frame, self.box_start_point, self.box_end_point, (0, 255, 0), 2)

    def execute_line_follow(self, frame, returning=False):
        # Check if PC has set a drive mode that overrides line following
        if self.pc_drive_mode == "manual_drive":
            # PC is controlling the robot directly
            return None
            
        if self.target_hsv_range is None:
            self.stop_robot()
            return None

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if len(self.target_hsv_range) == 4:
            mask1 = cv2.inRange(hsv_frame, self.target_hsv_range[0], self.target_hsv_range[1])
            mask2 = cv2.inRange(hsv_frame, self.target_hsv_range[2], self.target_hsv_range[3])
            mask = cv2.add(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_frame, self.target_hsv_range[0], self.target_hsv_range[1])
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        M = cv2.moments(mask)
        radius = int(math.sqrt(M["m00"] / math.pi)) if M["m00"] > 0 else 0

        twist = Twist()
        if self.follow_state == FollowState.TRACKING:
            if radius > self.min_radius_threshold:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                error = cx - (frame.shape[1] // 2 + self.camera_offset)
                [z_pid, _] = self.pid_controller.update([error * 1.0 / 16, 0])
                twist.linear.x = self.linear_speed
                twist.angular.z = float(z_pid)
                self.cmd_vel_pub.publish(twist)
                cv2.circle(frame, (cx, cy), 10, (255, 0, 255), -1)
            else:
                self.get_logger().info("Line lost, reversing...")
                self.follow_state = FollowState.REVERSING
                self.action_start_time = time.time()
                self.stop_robot()
        
        elif self.follow_state == FollowState.REVERSING:
            twist.linear.x = self.REVERSE_SPEED
            self.cmd_vel_pub.publish(twist)
            if time.time() - self.action_start_time > self.REVERSE_DURATION:
                self.get_logger().info("Reversing complete, searching...")
                self.follow_state = FollowState.SEARCHING
                self.action_start_time = time.time()
                self.search_direction = 1
                self.stop_robot()

        elif self.follow_state == FollowState.SEARCHING:
            if radius > self.min_radius_threshold:
                self.get_logger().info("Line reacquired!")
                self.follow_state = FollowState.TRACKING
                self.stop_robot()
            else:
                elapsed_time = time.time() - self.action_start_time
                duration_this_turn = self.SEARCH_DURATION_LEFT if self.search_direction == 1 else self.SEARCH_DURATION_RIGHT
                if elapsed_time < duration_this_turn:
                    twist.angular.z = self.SEARCH_TURN_SPEED * self.search_direction
                    self.cmd_vel_pub.publish(twist)
                else:
                    if self.search_direction == 1:
                        self.get_logger().info("Switching to search right.")
                        self.search_direction = -1
                        self.action_start_time = time.time()
                    else:
                        self.get_logger().info("Search complete, line not found.")
                        self.follow_state = FollowState.STOPPED
                        self.stop_robot()
                        if not returning: 
                            self.main_state = RobotState.WAITING_FOR_RAT
                            self.lidar_ready_time = time.time() + self.LIDAR_GRACE_PERIOD
                            self.lidar_baseline = None
                        else: 
                            self.main_state = RobotState.RESET

        elif self.follow_state == FollowState.STOPPED:
            self.stop_robot()

        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    def execute_wait_for_rat(self, frame):
        if self.action_start_time is None:
            self.action_start_time = time.time()

        cv2.putText(frame, "Waiting for the rat...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Check for PC manual found signal
        if self.pc_manual_found and self.pc_rat_mode == "manual":
            self.get_logger().info("STATE: WAITING_FOR_RAT - PC manual 'rat found' signal!")
            self.main_state = RobotState.DISPENSING
            self.pc_manual_found = False
            self.action_start_time = None
            self.lidar_baseline = None
            
            # Publish rat detection status
            found_msg = Bool()
            found_msg.data = True
            self.rat_detection_pub.publish(found_msg)
        
        elif self.rat_detected and self.pc_rat_mode == "auto":
            self.get_logger().info("STATE: WAITING_FOR_RAT - Rat detected!")
            self.main_state = RobotState.DISPENSING
            self.rat_detected = False
            self.action_start_time = None
            self.lidar_baseline = None
            
            # Publish rat detection status
            found_msg = Bool()
            found_msg.data = True
            self.rat_detection_pub.publish(found_msg)
        
        elif time.time() - self.action_start_time > self.WAIT_DURATION:
            self.get_logger().info("STATE: WAITING_FOR_RAT - Timeout. No rat detected.")
            self.main_state = RobotState.TURNING
            self.action_start_time = None
            self.lidar_baseline = None


    def execute_dispense_phase(self):
        self.get_logger().info("STATE: DISPENSING - Dispensing reward.")
        self.dispense_cheerio()
        self.get_logger().info("Waiting for rat to retrieve reward...")
        time.sleep(5.0)
        self.main_state = RobotState.TURNING
        self.action_start_time = None

    def execute_turn_phase(self, frame):
        if self.action_start_time is None:
            self.get_logger().info("STATE: TURNING - Beginning blind 180-degree turn.")
            self.action_start_time = time.time()
        
        elapsed_time = time.time() - self.action_start_time
        
        if elapsed_time < self.turn_duration:
            cv2.putText(frame, "Executing blind turn...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            twist = Twist(); twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
        else:
            self.get_logger().info("STATE: TURNING - Blind turn complete. Now searching for line.")
            self.stop_robot()
            self.main_state = RobotState.RETURNING_HOME
            self.follow_state = FollowState.SEARCHING
            self.action_start_time = time.time()
            self.search_direction = 1

    def execute_reset_phase(self):
        self.get_logger().info("STATE: RESET - Trial complete.")
        self.stop_robot()
        self.buzzer_pub.publish(UInt16(data=1)); time.sleep(0.5)
        self.buzzer_pub.publish(UInt16(data=0))
        self.main_state = RobotState.MANUAL_CONTROL
        print(MANUAL_CONTROL_MSG)

    def execute_manual_control(self):
        key = self.getKey()
        moveBindings = {'i': (1, 0), 'o': (1, -1), 'j': (0, 1), 'l': (0, -1), 'u': (1, 1), ',': (-1, 0), '.': (-1, 1), 'm': (-1, -1)}

        if key in moveBindings.keys():
            x = moveBindings[key][0]
            th = moveBindings[key][1]
            twist = Twist()
            twist.linear.x = float(x * self.manual_speed)
            twist.angular.z = float(th * self.manual_turn)
            self.cmd_vel_pub.publish(twist)
        elif key == ' ' or key == 'k':
            self.stop_robot()
        elif key == 's':
            self.get_logger().info("Manual control finished. Ready for next trial.")
            self.stop_robot()
            self.target_hsv_range = None
            self.main_state = RobotState.START
        else:
             self.stop_robot()

    def mouse_callback(self, event, x, y, flags, param):
        if self.main_state != RobotState.PICK_COLOR: return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.selecting_box = True
            self.box_start_point = (x, y)
            self.box_end_point = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_box:
            self.box_end_point = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            if not self.selecting_box: return
            self.selecting_box = False
            if self.box_start_point == (x,y): return

            ret, frame = self.capture.read()
            if not ret: return
            if self.img_flip: frame = cv2.flip(frame, 1)
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            x1, y1 = self.box_start_point; x2, y2 = self.box_end_point
            roi = hsv_frame[min(y1, y2):max(y1, y2), min(x1, x2):max(x1, x2)]
            if roi.size == 0: return

            avg_color = cv2.mean(roi)
            h, s, v = int(avg_color[0]), int(avg_color[1]), int(avg_color[2])
            
            h_margin, s_margin, v_margin = 10, 70, 70
            lower = np.array([max(0, h - h_margin), max(40, s - s_margin), max(40, v - v_margin)])
            upper = np.array([min(179, h + h_margin), min(255, s + s_margin), min(255, v + v_margin)])
            self.target_hsv_range = (lower, upper)
            
            self.get_logger().info(f"Color selected. Avg HSV: [{h}, {s}, {v}]. Range: L={lower}, U={upper}")
            self.main_state = RobotState.FOLLOWING_LINE
            self.follow_state = FollowState.TRACKING

    def lidar_callback(self, msg):
        if self.main_state != RobotState.WAITING_FOR_RAT: 
            self.lidar_detection_counter = 0
            return
        
        if self.lidar_ready_time is not None and time.time() < self.lidar_ready_time:
            return

        current_ranges = np.array(msg.ranges)
        current_ranges[np.isinf(current_ranges)] = msg.range_max
        current_ranges[np.isnan(current_ranges)] = msg.range_max

        if self.lidar_baseline is None:
            self.get_logger().info("LiDAR stabilized. Capturing baseline...")
            self.lidar_baseline = current_ranges
            return

        diff = self.lidar_baseline - current_ranges
        
        changed_indices = np.where(
            (diff > self.LIDAR_CHANGE_THRESHOLD) & 
            (current_ranges < self.LIDAR_PROXIMITY_LIMIT) &
            (current_ranges > 0.01)
        )[0]
        
        is_significant_change = len(changed_indices) >= self.LIDAR_MIN_POINTS_CHANGED

        if is_significant_change:
            self.lidar_detection_counter += 1
        else:
            self.lidar_detection_counter = max(0, self.lidar_detection_counter - 1)

        if self.lidar_detection_counter >= self.LIDAR_PERSISTENCE_COUNT:
            self.get_logger().info("Persistent, significant, AND close change detected in LiDAR readings!")
            self.rat_detected = True
            self.lidar_detection_counter = 0

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def get_cheerio_count(self):
        if not os.path.exists(COUNT_FILE):
            with open(COUNT_FILE, "w") as f: f.write("0")
            return 0
        try:
            with open(COUNT_FILE, "r") as f: return int(f.read().strip())
        except (IOError, ValueError): return 0

    def save_cheerio_count(self, count):
        with open(COUNT_FILE, "w") as f: f.write(str(count))

    def dispense_cheerio(self):
        self.get_logger().info("Dispensing a cheerio...")
        try:
            count = self.get_cheerio_count()
            if count >= 12:
                self.get_logger().warn("Cheerio dispenser is empty! Please reload.")
                self.save_cheerio_count(0)
                msg = Int32(); msg.data = self.initial_dispenser_angle
                self.servo2_pub.publish(msg)
                return

            target_angle = (count + 1) * 15
            self.get_logger().info(f"Dispensing cheerio #{count + 1}. Moving to {target_angle} degrees.")

            msg = Int32()
            msg.data = target_angle
            self.servo2_pub.publish(msg)
            time.sleep(1.0)

            self.get_logger().info("Jostling to release...")
            msg.data = target_angle - 5; self.servo2_pub.publish(msg); time.sleep(0.5)
            msg.data = target_angle + 5; self.servo2_pub.publish(msg); time.sleep(0.5)
            msg.data = target_angle; self.servo2_pub.publish(msg); time.sleep(0.5)

            self.save_cheerio_count(count + 1)
            self.get_logger().info("Dispense complete.")
        except Exception as e:
            self.get_logger().error(f"Failed to dispense cheerio: {e}")

class simplePID:
    def __init__(self, p, i, d):
        self.p, self.i, self.d = p, i, d
        self.cp, self.ci, self.cd = 0, 0, 0
        self.error = [0.0] * 2
        self.result = [0.0] * 2

    def update(self, error):
        self.error[1] = error[0] - self.error[0]
        self.error[0] = error[0]
        self.cp = self.p[0] * self.error[0]
        self.ci += self.i[0] * self.error[0]
        self.cd = self.d[0] * self.error[1]
        if self.i[1] != 0: self.ci = max(-self.i[1], min(self.ci, self.i[1]))
        self.result[0] = self.cp + self.ci + self.cd
        if self.p[1] != 0: self.result[0] = max(-self.p[1], min(self.result[0], self.p[1]))
        return self.result

def main(args=None):
    rclpy.init(args=args)
    node = HideAndSeekNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

