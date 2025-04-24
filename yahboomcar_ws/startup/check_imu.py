#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
class IMUTester(Node):
    def __init__(self):
        super().__init__('imu_tester')
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.get_logger().info("?? Listening to /imu... Press Ctrl+C to stop.")
    def imu_callback(self, msg):
        acc = msg.linear_acceleration
        ang = msg.angular_velocity
        print("[IMU] Accel (m/s^2): x={:.2f}, y={:.2f}, z={:.2f} | Gyro (rad/s): x={:.2f}, y={:.2f}, z={:.2f}".format(
            acc.x, acc.y, acc.z, ang.x, ang.y, ang.z))

def main(args=None):
    rclpy.init(args=args)
    node = IMUTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
