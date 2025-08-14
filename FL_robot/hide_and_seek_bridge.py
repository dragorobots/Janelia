#!/usr/bin/env python3
"""
ROS2 Bridge Node for Hide and Seek Robot
Subscribes to PC commands and publishes telemetry back to PC
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import threading

class HideAndSeekBridge(Node):
    def __init__(self):
        super().__init__('hide_and_seek_bridge')
        self.get_logger().info("Initializing Hide and Seek Bridge...")

        # --- PC Command Subscribers ---
        self.target_spot_sub = self.create_subscription(
            Int32, '/hide_and_seek/target_spot', self.target_spot_callback, 10)
        self.toggles_sub = self.create_subscription(
            String, '/hide_and_seek/toggles', self.toggles_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/hide_and_seek/cmd_vel', self.cmd_vel_callback, 10)
        self.manual_found_sub = self.create_subscription(
            Bool, '/hide_and_seek/manual_found', self.manual_found_callback, 10)
        self.line_color_sub = self.create_subscription(
            String, '/hide_and_seek/line_color', self.line_color_callback, 10)

        # --- Telemetry Publishers ---
        self.line_follow_status_pub = self.create_publisher(
            String, '/line_follow/status', 10)
        self.rat_detection_pub = self.create_publisher(
            Bool, '/rat_detection/found', 10)
        self.progress_pub = self.create_publisher(
            String, '/hide_and_seek/progress', 10)

        # --- Robot State Publishers (for robot control) ---
        self.cmd_vel_robot_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo1_pub = self.create_publisher(Int32, '/servo_s1', 10)
        self.servo2_pub = self.create_publisher(Int32, '/servo_s2', 10)
        self.buzzer_pub = self.create_publisher(Int32, '/beep', 10)

        # --- State Variables ---
        self.current_target_spot = 0
        self.drive_mode = "auto_line"  # auto_line, manual_line, manual_drive
        self.rat_mode = "auto"  # auto, manual
        self.manual_found_triggered = False
        self.line_color_hue = None

        # --- Status tracking ---
        self.last_status_update = time.time()
        self.status_update_interval = 0.5  # seconds

        # --- Start status publishing thread ---
        self.status_thread = threading.Thread(target=self.status_publisher_loop, daemon=True)
        self.status_thread.start()

        self.get_logger().info("Bridge initialized and ready for PC communication")

    def target_spot_callback(self, msg):
        """Handle target spot selection from PC"""
        self.current_target_spot = msg.data
        self.get_logger().info(f"Received target spot: {self.current_target_spot}")
        
        # Publish progress update
        progress_msg = String()
        progress_msg.data = f"target_spot_set:{self.current_target_spot}"
        self.progress_pub.publish(progress_msg)

    def toggles_callback(self, msg):
        """Handle toggle commands from PC"""
        toggle_str = msg.data
        self.get_logger().info(f"Received toggle: {toggle_str}")
        
        if toggle_str.startswith("drive_mode="):
            self.drive_mode = toggle_str.split("=")[1]
            self.get_logger().info(f"Drive mode set to: {self.drive_mode}")
            
        elif toggle_str.startswith("rat_mode="):
            self.rat_mode = toggle_str.split("=")[1]
            self.get_logger().info(f"Rat detection mode set to: {self.rat_mode}")
            
        elif toggle_str == "manual_found=true":
            self.manual_found_triggered = True
            self.get_logger().info("Manual 'rat found' signal received")
            
            # Publish to rat detection
            found_msg = Bool()
            found_msg.data = True
            self.rat_detection_pub.publish(found_msg)

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from PC"""
        if self.drive_mode == "manual_drive":
            self.cmd_vel_robot_pub.publish(msg)
            self.get_logger().info(f"Manual drive command: linear={msg.linear.x}, angular={msg.angular.z}")

    def manual_found_callback(self, msg):
        """Handle manual found signal from PC"""
        if msg.data:
            self.manual_found_triggered = True
            self.get_logger().info("Manual 'rat found' signal received")
            
            # Publish to rat detection
            found_msg = Bool()
            found_msg.data = True
            self.rat_detection_pub.publish(found_msg)

    def line_color_callback(self, msg):
        """Handle line color selection from PC"""
        color_str = msg.data
        if color_str.startswith("hue="):
            try:
                self.line_color_hue = int(color_str.split("=")[1])
                self.get_logger().info(f"Line color hue set to: {self.line_color_hue}")
                
                # Publish progress update
                progress_msg = String()
                progress_msg.data = f"line_color_set:{self.line_color_hue}"
                self.progress_pub.publish(progress_msg)
            except ValueError:
                self.get_logger().error(f"Invalid hue value: {color_str}")

    def status_publisher_loop(self):
        """Background thread to publish status updates"""
        while rclpy.ok():
            try:
                # Publish line follow status
                line_status_msg = String()
                if self.drive_mode == "auto_line":
                    line_status_msg.data = "following"
                elif self.drive_mode == "manual_line":
                    line_status_msg.data = "manual_line_mode"
                else:
                    line_status_msg.data = "manual_drive_mode"
                self.line_follow_status_pub.publish(line_status_msg)

                # Publish rat detection status
                rat_status_msg = Bool()
                rat_status_msg.data = self.manual_found_triggered
                self.rat_detection_pub.publish(rat_status_msg)

                # Reset manual found flag
                if self.manual_found_triggered:
                    self.manual_found_triggered = False

                time.sleep(self.status_update_interval)
                
            except Exception as e:
                self.get_logger().error(f"Error in status publisher: {e}")
                time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    bridge = HideAndSeekBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
