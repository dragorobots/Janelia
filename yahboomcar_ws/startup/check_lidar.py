#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarCheck(Node):
    def __init__(self):
        super().__init__('lidar_check')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.get_logger().info("?? Listening to /scan... Press Ctrl+C to stop.")

    def lidar_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not (r == float('inf') or r == 0.0)]
        if valid_ranges:
            print(f"[?] LIDAR Active: Min = {min(valid_ranges):.2f} m, Max = {max(valid_ranges):.2f} m")
        else:
            print("[??] No valid LIDAR ranges detected.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
