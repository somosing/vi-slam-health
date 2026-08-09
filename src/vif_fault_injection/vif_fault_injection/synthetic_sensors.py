from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class SyntheticSensors(Node):
    """Publishes timestamp-correct test messages; it is not a sensor simulator."""

    def __init__(self) -> None:
        super().__init__("synthetic_sensors")
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("image_rate_hz", 20.0)
        self.declare_parameter("imu_rate_hz", 200.0)

        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        image_rate = float(self.get_parameter("image_rate_hz").value)
        imu_rate = float(self.get_parameter("imu_rate_hz").value)
        if self._width <= 0 or self._height <= 0 or image_rate <= 0.0 or imu_rate <= 0.0:
            raise ValueError("dimensions and publication rates must be positive")

        self._left_pub = self.create_publisher(
            Image, "/vif/raw/camera/left/image_raw", qos_profile_sensor_data
        )
        self._right_pub = self.create_publisher(
            Image, "/vif/raw/camera/right/image_raw", qos_profile_sensor_data
        )
        self._imu_pub = self.create_publisher(Imu, "/vif/raw/imu", qos_profile_sensor_data)
        self._image_timer = self.create_timer(1.0 / image_rate, self._publish_stereo)
        self._imu_timer = self.create_timer(1.0 / imu_rate, self._publish_imu)
        self._image_index = 0
        self._imu_index = 0
        self._pattern = self._checkerboard(self._width, self._height)
        self.get_logger().warning("Synthetic sensor traffic is for integration testing only")

    @staticmethod
    def _checkerboard(width: int, height: int) -> bytes:
        return bytes(
            185 if ((x // 16) + (y // 16)) % 2 else 55
            for y in range(height)
            for x in range(width)
        )

    def _image(self, frame_id: str, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = self._height
        msg.width = self._width
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = self._width
        msg.data = self._pattern
        return msg

    def _publish_stereo(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._left_pub.publish(self._image("camera_left_optical_frame", stamp))
        self._right_pub.publish(self._image("camera_right_optical_frame", stamp))
        self._image_index += 1

    def _publish_imu(self) -> None:
        stamp = self.get_clock().now().to_msg()
        t = self._imu_index / float(self.get_parameter("imu_rate_hz").value)
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.z = 0.12
        msg.linear_acceleration.x = 0.03 * math.sin(0.5 * t)
        msg.linear_acceleration.y = 0.03 * math.cos(0.5 * t)
        msg.linear_acceleration.z = 9.81
        msg.angular_velocity_covariance[0] = 1.0e-4
        msg.angular_velocity_covariance[4] = 1.0e-4
        msg.angular_velocity_covariance[8] = 1.0e-4
        msg.linear_acceleration_covariance[0] = 1.0e-3
        msg.linear_acceleration_covariance[4] = 1.0e-3
        msg.linear_acceleration_covariance[8] = 1.0e-3
        self._imu_pub.publish(msg)
        self._imu_index += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
