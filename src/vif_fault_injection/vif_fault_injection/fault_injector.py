from __future__ import annotations

import copy
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from vif_interfaces.msg import FaultEvent

from .fault_schedule import FaultSchedule
from .image_faults import occluded_copy


class FaultInjector(Node):
    """Relays sensor messages and applies a deterministic, labelled fault window."""

    VALID_FAULTS = {
        "camera_blackout",
        "imu_dropout",
        "frame_drop",
        "timestamp_delay",
        "visual_occlusion",
    }

    def __init__(self) -> None:
        super().__init__("fault_injector")
        self.declare_parameter("enabled", True)
        self.declare_parameter("fault_type", "camera_blackout")
        self.declare_parameter("start_after_sec", 8.0)
        self.declare_parameter("duration_sec", 3.0)
        self.declare_parameter("frame_drop_probability", 0.5)
        self.declare_parameter("timestamp_delay_ms", 100.0)
        self.declare_parameter("random_seed", 7)

        self._fault_type = str(self.get_parameter("fault_type").value)
        if self._fault_type not in self.VALID_FAULTS:
            raise ValueError(f"fault_type must be one of {sorted(self.VALID_FAULTS)}")
        self._schedule = FaultSchedule(
            enabled=bool(self.get_parameter("enabled").value),
            start_after_sec=float(self.get_parameter("start_after_sec").value),
            duration_sec=float(self.get_parameter("duration_sec").value),
        )
        self._drop_probability = float(self.get_parameter("frame_drop_probability").value)
        if not 0.0 <= self._drop_probability <= 1.0:
            raise ValueError("frame_drop_probability must be in [0, 1]")
        self._delay_ns = int(float(self.get_parameter("timestamp_delay_ms").value) * 1e6)
        self._rng = random.Random(int(self.get_parameter("random_seed").value))
        self._start_ns = self.get_clock().now().nanoseconds
        self._was_active = False

        self._left_pub = self.create_publisher(
            Image, "/vif/camera/left/image_raw", qos_profile_sensor_data
        )
        self._right_pub = self.create_publisher(
            Image, "/vif/camera/right/image_raw", qos_profile_sensor_data
        )
        self._imu_pub = self.create_publisher(Imu, "/vif/imu", qos_profile_sensor_data)
        self._fault_event_pub = self.create_publisher(
            FaultEvent, "/vif/fault/events", 10
        )
        self.create_subscription(
            Image,
            "/vif/raw/camera/left/image_raw",
            lambda msg: self._relay_image(msg, self._left_pub),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/vif/raw/camera/right/image_raw",
            lambda msg: self._relay_image(msg, self._right_pub),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, "/vif/raw/imu", self._relay_imu, qos_profile_sensor_data
        )
        self.create_timer(0.1, self._announce_transitions)

    def _elapsed(self) -> float:
        return (self.get_clock().now().nanoseconds - self._start_ns) / 1e9

    def _active(self) -> bool:
        return self._schedule.active(self._elapsed())

    def _announce_transitions(self) -> None:
        active = self._active()
        if active and not self._was_active:
            self.get_logger().warning(f"FAULT START: {self._fault_type}")
            self._publish_fault_event(FaultEvent.STARTED)
        elif self._was_active and not active:
            self.get_logger().warning(f"FAULT END: {self._fault_type}")
            self._publish_fault_event(FaultEvent.ENDED)
        self._was_active = active

    def _publish_fault_event(self, event: int) -> None:
        message = FaultEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.event = event
        message.fault_type = self._fault_type
        message.elapsed_sec = self._elapsed()
        self._fault_event_pub.publish(message)

    def _relay_image(self, msg: Image, publisher) -> None:
        active = self._active()
        if active and self._fault_type == "camera_blackout":
            return
        if active and self._fault_type == "visual_occlusion":
            publisher.publish(occluded_copy(msg))
            return
        if (
            active
            and self._fault_type == "frame_drop"
            and self._rng.random() < self._drop_probability
        ):
            return
        if active and self._fault_type == "timestamp_delay":
            msg = self._delayed_copy(msg)
        publisher.publish(msg)

    def _relay_imu(self, msg: Imu) -> None:
        if self._active() and self._fault_type == "imu_dropout":
            return
        if self._active() and self._fault_type == "timestamp_delay":
            msg = self._delayed_copy(msg)
        self._imu_pub.publish(msg)

    def _delayed_copy(self, msg):
        delayed = copy.deepcopy(msg)
        stamp_ns = delayed.header.stamp.sec * 1_000_000_000 + delayed.header.stamp.nanosec
        stamp_ns -= self._delay_ns
        delayed.header.stamp.sec = stamp_ns // 1_000_000_000
        delayed.header.stamp.nanosec = stamp_ns % 1_000_000_000
        return delayed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
