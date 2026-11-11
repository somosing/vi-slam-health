#!/usr/bin/env python3
"""Publish map -> odom from ORB-SLAM3 pose and local odometry TF."""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener, TransformBroadcaster
from vif_interfaces.msg import TrackingStatus


def normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in q)


def multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(
    q: tuple[float, float, float, float], v: tuple[float, float, float]
) -> tuple[float, float, float]:
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class SlamMapOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("slam_map_odom_bridge")
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.tracking_timeout = float(
            self.declare_parameter("tracking_timeout_sec", 0.5).value
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tracking_valid = False
        self.last_tracking_time: Time | None = None
        self.create_subscription(
            TrackingStatus, "/vif/slam/tracking", self.on_tracking, 10
        )
        self.create_subscription(
            Odometry, "/vif/slam/odometry", self.on_odometry, 10
        )

    def on_tracking(self, msg: TrackingStatus) -> None:
        self.tracking_valid = msg.state == TrackingStatus.TRACKING
        self.last_tracking_time = Time.from_msg(msg.header.stamp)

    def on_odometry(self, msg: Odometry) -> None:
        now = self.get_clock().now()
        if (
            not self.tracking_valid
            or self.last_tracking_time is None
            or (now - self.last_tracking_time).nanoseconds
            > int(self.tracking_timeout * 1e9)
        ):
            return

        map_frame = msg.header.frame_id
        base_frame = msg.child_frame_id or self.base_frame
        stamp = Time.from_msg(msg.header.stamp)
        try:
            odom_from_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                base_frame,
                stamp,
                timeout=Duration(seconds=0.02),
            ).transform
        except TransformException as error:
            self.get_logger().debug(f"Waiting for odom TF: {error}")
            return

        pose = msg.pose.pose
        map_from_base_q = normalize(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        odom_from_base_q = normalize(
            (
                odom_from_base.rotation.x,
                odom_from_base.rotation.y,
                odom_from_base.rotation.z,
                odom_from_base.rotation.w,
            )
        )
        map_from_odom_q = normalize(
            multiply(
                map_from_base_q,
                (
                    -odom_from_base_q[0],
                    -odom_from_base_q[1],
                    -odom_from_base_q[2],
                    odom_from_base_q[3],
                ),
            )
        )
        odom_from_base_p = (
            odom_from_base.translation.x,
            odom_from_base.translation.y,
            odom_from_base.translation.z,
        )
        rotated = rotate(map_from_odom_q, odom_from_base_p)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = pose.position.x - rotated[0]
        transform.transform.translation.y = pose.position.y - rotated[1]
        transform.transform.translation.z = pose.position.z - rotated[2]
        transform.transform.rotation.x = map_from_odom_q[0]
        transform.transform.rotation.y = map_from_odom_q[1]
        transform.transform.rotation.z = map_from_odom_q[2]
        transform.transform.rotation.w = map_from_odom_q[3]
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = SlamMapOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
