import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


class CommandAdapter(Node):
    def __init__(self):
        super().__init__(
            "nav2_twist_stamper",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.command_time = None
        self.odom_time = None
        self.linear = 0.0
        self.angular = 0.0

        self.publisher = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )
        self.create_subscription(
            Twist, "/nav2_baseline/cmd_vel_smoothed",
            self.on_command, 10,
        )
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom",
            self.on_odom, qos_profile_sensor_data,
        )
        self.create_timer(0.05, self.tick)
        self.get_logger().info("Command adapter ready; waiting for Nav2 commands.")

    def on_command(self, msg):
        x, z = msg.linear.x, msg.angular.z
        if not (math.isfinite(x) and math.isfinite(z)):
            self.linear = self.angular = 0.0
            self.command_time = None
            return
        self.linear = max(-0.15, min(0.15, x))
        self.angular = max(-0.30, min(0.30, z))
        self.command_time = self.get_clock().now()

    def on_odom(self, msg):
        self.odom_time = self.get_clock().now()

    def publish(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.publisher.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        fresh = (
            self.command_time is not None
            and self.odom_time is not None
            and 0 <= (now - self.command_time).nanoseconds <= 350_000_000
            and 0 <= (now - self.odom_time).nanoseconds <= 500_000_000
        )
        self.publish(
            self.linear if fresh else 0.0,
            self.angular if fresh else 0.0,
        )


rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = CommandAdapter()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    for _ in range(5):
        node.publish(0.0, 0.0)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()
