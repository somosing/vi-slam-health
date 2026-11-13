#!/usr/bin/env python3
"""Fail closed: forward Nav2 velocity only while visual SLAM tracking is healthy."""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from vif_interfaces.msg import TrackingStatus


class SlamTrackingCmdGate(Node):
    def __init__(self) -> None:
        super().__init__("slam_tracking_cmd_gate")
        self.timeout_sec = float(self.declare_parameter("tracking_timeout_sec", 0.5).value)
        self.command_timeout_sec = float(self.declare_parameter("command_timeout_sec", 0.25).value)
        self.tracking_ok = False
        self.tracking_stamp: Time | None = None
        self.command_stamp: Time | None = None
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(TrackingStatus, "/vif/slam/tracking", self.on_tracking, 10)
        self.create_subscription(Twist, "cmd_vel_nav", self.on_command, 10)
        self.create_timer(0.05, self.watchdog)

    def on_tracking(self, msg: TrackingStatus) -> None:
        self.tracking_ok = msg.state == TrackingStatus.TRACKING
        self.tracking_stamp = Time.from_msg(msg.header.stamp)

    def tracking_is_fresh(self) -> bool:
        return (
            self.tracking_ok
            and self.tracking_stamp is not None
            and (self.get_clock().now() - self.tracking_stamp).nanoseconds
            <= int(self.timeout_sec * 1e9)
        )

    def on_command(self, msg: Twist) -> None:
        self.command_stamp = self.get_clock().now()
        self.publisher.publish(msg if self.tracking_is_fresh() else Twist())

    def watchdog(self) -> None:
        now = self.get_clock().now()
        command_fresh = (
            self.command_stamp is not None
            and (now - self.command_stamp).nanoseconds
            <= int(self.command_timeout_sec * 1e9)
        )
        if not self.tracking_is_fresh() or not command_fresh:
            self.publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = SlamTrackingCmdGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
