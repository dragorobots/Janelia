import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import Twist

class HideSeekBridge(Node):
    def __init__(self):
        super().__init__('hide_and_seek_bridge')
        self.create_subscription(Int32, '/hide_and_seek/target_spot', self.on_target, 10)
        self.create_subscription(String,'/hide_and_seek/toggles', self.on_toggle, 10)
        self.create_subscription(Twist, '/hide_and_seek/cmd_vel', self.on_cmdvel, 10)
        self.lf_pub    = self.create_publisher(String, '/line_follow/status', 10)
        self.rat_pub   = self.create_publisher(Bool,   '/rat_detection/found', 10)
        self.trial_pub = self.create_publisher(String, '/hide_and_seek/progress', 10)
    def on_target(self, msg): self.get_logger().info(f"target_spot={msg.data}")
    def on_toggle(self, msg): self.get_logger().info(f"toggles={msg.data}")
    def on_cmdvel(self, msg): self.get_logger().info(f"cmdvel v={msg.linear.x:.2f} w={msg.angular.z:.2f}")

def main():
    rclpy.init(); n=HideSeekBridge(); rclpy.spin(n); rclpy.shutdown()
if __name__=='__main__': main()
