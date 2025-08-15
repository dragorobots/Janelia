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
    FOLLOWING_START_LINE = 2  # Follow start line until it ends (intersection)
    SELECTING_HIDING_SPOT = 3 # At intersection, select hiding spot color
    FOLLOWING_HIDING_LINE = 4 # Follow hiding spot line until it ends
    WAITING_FOR_RAT = 5       # At hiding spot, wait for rat
    TURNING_180 = 6           # Turn 180 degrees after rat detection
    RETURNING_HOME = 7        # Follow hiding line back to intersection, then start line
    RESET = 8                 # Turn 180 and reset for next trial
    MANUAL_CONTROL = 9

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
        self.start_line_hsv_range = None  # Color of the start line (first color selected)
        self.hiding_line_hsv_range = None # Color of the hiding spot line
        self.current_line_hsv_range = None # Current line color to follow
        self.action_start_time = None
        self.search_direction = 1
        self.line_lost_count = 0  # Count consecutive line losses
        self.MAX_LINE_LOSSES = 3  # After this many losses, consider line ended

        # --- Parameters ---
        self.linear_speed = 0.12
        self.pid_controller = simplePID(p=[0.03, 0], i=[0.0, 0], d=[0.03, 0])
        self.min_radius_threshold = 80
        self.min_line_area = 1000
        self.camera_offset = 0
        self.img_flip = True
        self.REVERSE_SPEED = -0.1
        self.REVERSE_DURATION = 0.5  # Changed to 0.5s as requested
        self.SEARCH_TURN_SPEED = 0.6
        self.SEARCH_DURATION_LEFT = 2.7
        self.SEARCH_DURATION_RIGHT = 5.5
        self.initial_camera_yaw = 0
        self.turn_duration = 7.5
        self.WAIT_DURATION = 120.0

        # --- LiDAR Parameters ---
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
            self.settings = None
        self.manual_speed = 0.2
        self.manual_turn = 1.0
        
        # --- PC Communication Variables ---
        self.pc_target_spot = 0
        self.pc_drive_mode = "auto_line"
        self.pc_rat_mode = "auto"
        self.pc_manual_found = False
        self.pc_line_color_hue = None
        self.pc_override_active = False
        self.pc_color_selection_mode = "auto"
        self.last_published_progress = None

        # --- Main Control Loop ---
        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("Node initialized. Starting in START state.")

    # --- PC Communication Callbacks ---
    def target_spot_callback(self, msg):
        """Handle target spot selection from PC"""
        self.pc_target_spot = msg.data
        self.get_logger().info(f"PC set target spot: {self.pc_target_spot}")
        
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
            
        elif toggle_str.startswith("color_mode="):
            self.pc_color_selection_mode = toggle_str.split("=")[1]
            self.get_logger().info(f"PC color selection mode: {self.pc_color_selection_mode}")
            
        elif toggle_str == "start_trial=true":
            self.start_trial()
            self.get_logger().info("PC start trial command received")
            
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
                
                progress_msg = String()
                progress_msg.data = f"line_color_set:{self.pc_line_color_hue}"
                self.progress_pub.publish(progress_msg)
            except ValueError:
                self.get_logger().error(f"Invalid PC hue value: {color_str}")

    def getKey(self):
        if self.settings is None:
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

        # Execute current state
        if self.main_state == RobotState.START:
            self.execute_start_phase(display_view)
        elif self.main_state == RobotState.PICK_COLOR:
            self.execute_pick_color_phase(display_view)
        elif self.main_state in [RobotState.FOLLOWING_START_LINE, RobotState.FOLLOWING_HIDING_LINE, RobotState.RETURNING_HOME]:
            mask_display = self.execute_line_follow(display_view)
            if mask_display is not None and mask_display.size > 0:
                if mask_display.shape[0] != display_view.shape[0]:
                    mask_display = cv2.resize(mask_display, (mask_display.shape[1], display_view.shape[0]))
                display_view = cv2.hconcat([display_view, mask_display])
        elif self.main_state == RobotState.SELECTING_HIDING_SPOT:
            self.execute_selecting_hiding_spot(display_view)
        elif self.main_state == RobotState.WAITING_FOR_RAT:
            self.execute_wait_for_rat(display_view)
        elif self.main_state == RobotState.TURNING_180:
            self.execute_turn_180(display_view)
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
        # Only publish line follow status if we're actually line following
        line_status_msg = String()
        if self.main_state in [RobotState.FOLLOWING_START_LINE, RobotState.FOLLOWING_HIDING_LINE, RobotState.RETURNING_HOME]:
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
            progress_msg.data = "waiting_for_start"
        elif self.main_state == RobotState.PICK_COLOR:
            progress_msg.data = "pick_color_phase"
        elif self.main_state == RobotState.FOLLOWING_START_LINE:
            progress_msg.data = "following_start_line"
        elif self.main_state == RobotState.SELECTING_HIDING_SPOT:
            progress_msg.data = "selecting_hiding_spot"
        elif self.main_state == RobotState.FOLLOWING_HIDING_LINE:
            progress_msg.data = "following_hiding_line"
        elif self.main_state == RobotState.WAITING_FOR_RAT:
            progress_msg.data = "waiting_for_rat"
        elif self.main_state == RobotState.TURNING_180:
            progress_msg.data = "turning_180"
        elif self.main_state == RobotState.RETURNING_HOME:
            progress_msg.data = "returning_home"
        elif self.main_state == RobotState.RESET:
            progress_msg.data = "reset"
        elif self.main_state == RobotState.MANUAL_CONTROL:
            progress_msg.data = "manual_control"
        
        # Only publish if progress has changed
        if self.last_published_progress != progress_msg.data:
            self.progress_pub.publish(progress_msg)
            self.last_published_progress = progress_msg.data

    def execute_start_phase(self, frame):
        self.get_logger().info("STATE: START - Waiting for trial start command from PC.")
        cv2.putText(frame, "Waiting for START TRIAL command from PC...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "Press 's' to start manually", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        key = self.getKey()
        if key == 's':
            self.start_trial()

    def start_trial(self):
        """Start the trial - called from PC or manual key press"""
        self.get_logger().info("TRIAL STARTING - Signaling trial start.")
        self.get_logger().info("Centering camera to zero position.")
        
        # Center camera
        s1_msg = Int32(); s1_msg.data = self.initial_camera_yaw
        self.servo1_pub.publish(s1_msg)
        time.sleep(1.0)

        # Beep sequence
        for _ in range(3):
            self.buzzer_pub.publish(UInt16(data=1)); time.sleep(0.4)
            self.buzzer_pub.publish(UInt16(data=0)); time.sleep(0.4)
        
        # Small movement to indicate start
        twist = Twist(); twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist); time.sleep(1)
        twist.linear.x = -0.1
        self.cmd_vel_pub.publish(twist); time.sleep(1)
        self.stop_robot()
        
        # Transition to color selection
        self.main_state = RobotState.PICK_COLOR
        self.get_logger().info("Transitioning to PICK_COLOR state.")
        
        progress_msg = String()
        progress_msg.data = "trial_started"
        self.progress_pub.publish(progress_msg)

    def execute_pick_color_phase(self, frame):
        # Check if we have auto color selection
        if self.pc_color_selection_mode == "auto" and self.pc_line_color_hue is not None:
            # Auto mode: Use the color provided by PC for start line
            h = self.pc_line_color_hue
            s, v = 100, 100
            h_margin, s_margin, v_margin = 10, 70, 70
            lower = np.array([max(0, h - h_margin), max(40, s - s_margin), max(40, v - v_margin)])
            upper = np.array([min(179, h + h_margin), min(255, s + s_margin), min(255, v + v_margin)])
            self.start_line_hsv_range = (lower, upper)
            self.current_line_hsv_range = self.start_line_hsv_range
            
            self.get_logger().info(f"Auto start line color selected. Hue: {h}")
            self.main_state = RobotState.FOLLOWING_START_LINE
            self.follow_state = FollowState.TRACKING
            self.line_lost_count = 0
            
            progress_msg = String()
            progress_msg.data = "following_start_line"
            self.progress_pub.publish(progress_msg)
            return
        
        # Manual color selection mode
        cv2.putText(frame, "MANUAL MODE: Click and drag to select the START line color", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame, "Draw a rectangle around the start line you want to follow", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        if self.selecting_box:
            cv2.rectangle(frame, self.box_start_point, self.box_end_point, (0, 255, 0), 2)
            cv2.putText(frame, "Release mouse to confirm selection", (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def execute_selecting_hiding_spot(self, frame):
        """At intersection, select hiding spot color"""
        cv2.putText(frame, "At intersection - Select hiding spot line color", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Check if we have auto color selection for hiding spot
        if self.pc_color_selection_mode == "auto" and self.pc_line_color_hue is not None:
            # Use the color provided by PC for hiding line
            h = self.pc_line_color_hue
            s, v = 100, 100
            h_margin, s_margin, v_margin = 10, 70, 70
            lower = np.array([max(0, h - h_margin), max(40, s - s_margin), max(40, v - v_margin)])
            upper = np.array([min(179, h + h_margin), min(255, s + s_margin), min(255, v + v_margin)])
            self.hiding_line_hsv_range = (lower, upper)
            self.current_line_hsv_range = self.hiding_line_hsv_range
            
            self.get_logger().info(f"Auto hiding line color selected. Hue: {h}")
            self.main_state = RobotState.FOLLOWING_HIDING_LINE
            self.follow_state = FollowState.TRACKING
            self.line_lost_count = 0
            
            progress_msg = String()
            progress_msg.data = "following_hiding_line"
            self.progress_pub.publish(progress_msg)
            return
        
        # Manual color selection mode
        cv2.putText(frame, "MANUAL MODE: Click and drag to select the HIDING line color", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        if self.selecting_box:
            cv2.rectangle(frame, self.box_start_point, self.box_end_point, (0, 255, 0), 2)
            cv2.putText(frame, "Release mouse to confirm selection", (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def execute_line_follow(self, frame):
        """Robust line following logic"""
        if self.current_line_hsv_range is None:
            self.stop_robot()
            return None
            
        # Create HSV mask for line detection
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Handle different HSV range formats
        if isinstance(self.current_line_hsv_range, tuple) and len(self.current_line_hsv_range) == 2:
            mask = cv2.inRange(hsv_frame, self.current_line_hsv_range[0], self.current_line_hsv_range[1])
        elif isinstance(self.current_line_hsv_range, tuple) and len(self.current_line_hsv_range) == 4:
            mask1 = cv2.inRange(hsv_frame, self.current_line_hsv_range[0], self.current_line_hsv_range[1])
            mask2 = cv2.inRange(hsv_frame, self.current_line_hsv_range[2], self.current_line_hsv_range[3])
            mask = cv2.add(mask1, mask2)
        else:
            self.get_logger().error(f"Invalid HSV range format: {self.current_line_hsv_range}")
            return None
        
        # Enhanced noise filtering
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours to filter by shape and area
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area and find the largest valid contour
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_line_area:
                valid_contours.append(contour)
        
        # Create a clean mask with only valid contours
        clean_mask = np.zeros_like(mask)
        if valid_contours:
            largest_contour = max(valid_contours, key=cv2.contourArea)
            cv2.fillPoly(clean_mask, [largest_contour], 255)
        
        # Calculate moments from the clean mask
        M = cv2.moments(clean_mask)
        radius = int(math.sqrt(M["m00"] / math.pi)) if M["m00"] > 0 else 0
        area = M["m00"] if M["m00"] > 0 else 0

        # Add debugging information to frame
        cv2.putText(frame, f"Line Radius: {radius} (min: {self.min_radius_threshold})", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Line Area: {area:.0f} (min: {self.min_line_area})", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Valid Contours: {len(valid_contours)}", (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"State: {self.follow_state}", (50, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Line Lost Count: {self.line_lost_count}", (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Show masks for debugging
        mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        clean_mask_display = cv2.cvtColor(clean_mask, cv2.COLOR_GRAY2BGR)
        if valid_contours:
            cv2.drawContours(clean_mask_display, valid_contours, -1, (0, 255, 0), 2)
        
        mask_display = cv2.resize(mask_display, (320, 240))
        clean_mask_display = cv2.resize(clean_mask_display, (320, 240))
        mask_display = cv2.hconcat([mask_display, clean_mask_display])

        twist = Twist()
        if self.follow_state == FollowState.TRACKING:
            # Check both radius and area for robust line detection
            if radius > self.min_radius_threshold and area > self.min_line_area:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                error = cx - (frame.shape[1] // 2 + self.camera_offset)
                [z_pid, _] = self.pid_controller.update([error * 1.0 / 16, 0])
                twist.linear.x = self.linear_speed
                twist.angular.z = float(z_pid)
                self.cmd_vel_pub.publish(twist)
                cv2.circle(frame, (cx, cy), 10, (255, 0, 255), -1)
                cv2.putText(frame, f"Following: cx={cx}, error={error:.1f}, z={z_pid:.3f}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self.line_lost_count = 0  # Reset line lost count when tracking
            else:
                self.line_lost_count += 1
                self.get_logger().info(f"Line lost (radius: {radius} < {self.min_radius_threshold} or area: {area:.0f} < {self.min_line_area}), count: {self.line_lost_count}")
                
                if self.line_lost_count >= self.MAX_LINE_LOSSES:
                    # Line has ended - transition to next state
                    self.handle_line_ended()
                else:
                    # Try to recover line
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
            if radius > self.min_radius_threshold and area > self.min_line_area:
                self.get_logger().info("Line reacquired!")
                self.follow_state = FollowState.TRACKING
                self.line_lost_count = 0
                self.stop_robot()
            else:
                elapsed_time = time.time() - self.action_start_time
                duration_this_turn = self.SEARCH_DURATION_LEFT if self.search_direction == 1 else self.SEARCH_DURATION_RIGHT
                if elapsed_time < duration_this_turn:
                    twist.angular.z = self.SEARCH_TURN_SPEED * self.search_direction
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info(f"Searching: {elapsed_time:.1f}s / {duration_this_turn:.1f}s, direction: {'left' if self.search_direction == 1 else 'right'}")
                else:
                    if self.search_direction == 1:
                        self.get_logger().info("Left search complete, switching to right search.")
                        self.search_direction = -1
                        self.action_start_time = time.time()
                    else:
                        self.get_logger().info("Right search complete, line not found.")
                        self.line_lost_count += 1
                        if self.line_lost_count >= self.MAX_LINE_LOSSES:
                            self.handle_line_ended()
                        else:
                            self.follow_state = FollowState.TRACKING
                            self.stop_robot()

        elif self.follow_state == FollowState.STOPPED:
            self.stop_robot()
            
        return mask_display

    def handle_line_ended(self):
        """Handle when a line has ended (reached intersection or hiding spot)"""
        self.get_logger().info(f"Line ended. Current state: {self.main_state}")
        
        if self.main_state == RobotState.FOLLOWING_START_LINE:
            # Reached intersection - select hiding spot
            self.get_logger().info("Reached intersection. Selecting hiding spot.")
            self.main_state = RobotState.SELECTING_HIDING_SPOT
            self.follow_state = FollowState.STOPPED
            self.line_lost_count = 0
            
            progress_msg = String()
            progress_msg.data = "selecting_hiding_spot"
            self.progress_pub.publish(progress_msg)
            
        elif self.main_state == RobotState.FOLLOWING_HIDING_LINE:
            # Reached hiding spot - wait for rat
            self.get_logger().info("Reached hiding spot. Starting rat detection.")
            self.main_state = RobotState.WAITING_FOR_RAT
            self.follow_state = FollowState.STOPPED
            self.line_lost_count = 0
            self.lidar_ready_time = time.time() + self.LIDAR_GRACE_PERIOD
            self.lidar_baseline = None
            
            progress_msg = String()
            progress_msg.data = "waiting_for_rat"
            self.progress_pub.publish(progress_msg)
            
        elif self.main_state == RobotState.RETURNING_HOME:
            # During return journey, we need to switch line colors at intersection
            if self.current_line_hsv_range == self.hiding_line_hsv_range:
                # We were following hiding line, now switch to start line
                self.get_logger().info("Reached intersection during return. Switching to start line.")
                self.current_line_hsv_range = self.start_line_hsv_range
                self.line_lost_count = 0
                self.follow_state = FollowState.TRACKING
            else:
                # We were following start line, reached start position - reset
                self.get_logger().info("Reached start position. Resetting for next trial.")
                self.main_state = RobotState.RESET
                self.follow_state = FollowState.STOPPED
                self.line_lost_count = 0

    def execute_wait_for_rat(self, frame):
        if self.action_start_time is None:
            self.action_start_time = time.time()

        cv2.putText(frame, "Waiting for the rat...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Check for PC manual found signal
        if self.pc_manual_found and self.pc_rat_mode == "manual":
            self.get_logger().info("STATE: WAITING_FOR_RAT - PC manual 'rat found' signal!")
            self.main_state = RobotState.TURNING_180
            self.pc_manual_found = False
            self.action_start_time = None
            self.lidar_baseline = None
            
            found_msg = Bool()
            found_msg.data = True
            self.rat_detection_pub.publish(found_msg)
        
        elif self.rat_detected and self.pc_rat_mode == "auto":
            self.get_logger().info("STATE: WAITING_FOR_RAT - Rat detected!")
            self.main_state = RobotState.TURNING_180
            self.rat_detected = False
            self.action_start_time = None
            self.lidar_baseline = None
            
            found_msg = Bool()
            found_msg.data = True
            self.rat_detection_pub.publish(found_msg)
        
        elif time.time() - self.action_start_time > self.WAIT_DURATION:
            self.get_logger().info("STATE: WAITING_FOR_RAT - Timeout. No rat detected.")
            self.main_state = RobotState.TURNING_180
            self.action_start_time = None
            self.lidar_baseline = None

    def execute_turn_180(self, frame):
        if self.action_start_time is None:
            self.get_logger().info("STATE: TURNING_180 - Beginning 180-degree turn.")
            self.action_start_time = time.time()
        
        elapsed_time = time.time() - self.action_start_time
        
        if elapsed_time < self.turn_duration:
            cv2.putText(frame, "Executing 180° turn...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            twist = Twist(); twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
        else:
            self.get_logger().info("STATE: TURNING_180 - Turn complete. Starting return journey.")
            self.stop_robot()
            self.main_state = RobotState.RETURNING_HOME
            self.follow_state = FollowState.TRACKING
            self.line_lost_count = 0
            
            # Start with hiding line color, will switch to start line color at intersection
            self.current_line_hsv_range = self.hiding_line_hsv_range
            
            progress_msg = String()
            progress_msg.data = "returning_home"
            self.progress_pub.publish(progress_msg)

    def execute_reset_phase(self):
        self.get_logger().info("STATE: RESET - Trial complete.")
        self.stop_robot()
        self.buzzer_pub.publish(UInt16(data=1)); time.sleep(0.5)
        self.buzzer_pub.publish(UInt16(data=0))
        
        # Turn 180 degrees in place
        twist = Twist(); twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.turn_duration)
        self.stop_robot()
        
        # Reset for next trial
        self.main_state = RobotState.START
        self.follow_state = FollowState.STOPPED
        self.start_line_hsv_range = None
        self.hiding_line_hsv_range = None
        self.current_line_hsv_range = None
        self.line_lost_count = 0
        self.get_logger().info("Returned to START state. Ready for next trial.")

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
            self.main_state = RobotState.START
        else:
             self.stop_robot()

    def mouse_callback(self, event, x, y, flags, param):
        if self.main_state not in [RobotState.PICK_COLOR, RobotState.SELECTING_HIDING_SPOT]: 
            return

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
            
            self.get_logger().info(f"Color selected. Avg HSV: [{h}, {s}, {v}]. Range: L={lower}, U={upper}")
            
            if self.main_state == RobotState.PICK_COLOR:
                # This is the start line color
                self.start_line_hsv_range = (lower, upper)
                self.current_line_hsv_range = self.start_line_hsv_range
                self.get_logger().info(f"Start line color selected. Hue: {h}")
                self.main_state = RobotState.FOLLOWING_START_LINE
                self.follow_state = FollowState.TRACKING
                self.line_lost_count = 0
                
                progress_msg = String()
                progress_msg.data = "following_start_line"
                self.progress_pub.publish(progress_msg)
                
            elif self.main_state == RobotState.SELECTING_HIDING_SPOT:
                # This is the hiding line color
                self.hiding_line_hsv_range = (lower, upper)
                self.current_line_hsv_range = self.hiding_line_hsv_range
                self.get_logger().info(f"Hiding line color selected. Hue: {h}")
                self.main_state = RobotState.FOLLOWING_HIDING_LINE
                self.follow_state = FollowState.TRACKING
                self.line_lost_count = 0
                
                progress_msg = String()
                progress_msg.data = "following_hiding_line"
                self.progress_pub.publish(progress_msg)

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

