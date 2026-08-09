"""Timestamp-faithful EuRoC stereo-inertial publisher for ROS 2."""

from __future__ import annotations

import bisect
import threading
import time
from pathlib import Path

import cv2
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

from .euroc_io import EurocDataset, ImuSample, StereoSample, load_euroc_dataset


NANOSECONDS_PER_SECOND = 1_000_000_000


def _time_message(timestamp_ns: int) -> Time:
    message = Time()
    message.sec = timestamp_ns // NANOSECONDS_PER_SECOND
    message.nanosec = timestamp_ns % NANOSECONDS_PER_SECOND
    return message


class EurocPlayer(Node):
    def __init__(self) -> None:
        super().__init__("euroc_player")
        self.declare_parameter("dataset_path", "")
        self.declare_parameter("playback_rate", 1.0)
        self.declare_parameter("start_offset_sec", 0.0)
        self.declare_parameter("duration_sec", 0.0)
        self.declare_parameter("wait_for_subscribers_sec", 120.0)
        self.declare_parameter("required_subscriber_count", 2)
        self.declare_parameter("left_topic", "/vif/camera/left/image_raw")
        self.declare_parameter("right_topic", "/vif/camera/right/image_raw")
        self.declare_parameter("imu_topic", "/vif/imu")
        self.declare_parameter("left_frame_id", "camera_left_optical_frame")
        self.declare_parameter("right_frame_id", "camera_right_optical_frame")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("shutdown_on_complete", True)

        dataset_path = str(self.get_parameter("dataset_path").value)
        if not dataset_path:
            raise ValueError("dataset_path must name an extracted EuRoC sequence")
        self._playback_rate = float(self.get_parameter("playback_rate").value)
        self._start_offset_sec = float(self.get_parameter("start_offset_sec").value)
        self._duration_sec = float(self.get_parameter("duration_sec").value)
        self._subscriber_wait_sec = float(
            self.get_parameter("wait_for_subscribers_sec").value
        )
        self._required_subscribers = int(
            self.get_parameter("required_subscriber_count").value
        )
        self._shutdown_on_complete = bool(
            self.get_parameter("shutdown_on_complete").value
        )
        if self._playback_rate <= 0.0:
            raise ValueError("playback_rate must be positive")
        if self._start_offset_sec < 0.0 or self._duration_sec < 0.0:
            raise ValueError("start_offset_sec and duration_sec cannot be negative")
        if self._subscriber_wait_sec < 0.0 or self._required_subscribers < 0:
            raise ValueError("subscriber wait parameters cannot be negative")

        self._left_frame_id = str(self.get_parameter("left_frame_id").value)
        self._right_frame_id = str(self.get_parameter("right_frame_id").value)
        self._imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self._dataset = load_euroc_dataset(Path(dataset_path))
        self._left_pub = self.create_publisher(
            Image, str(self.get_parameter("left_topic").value), qos_profile_sensor_data
        )
        self._right_pub = self.create_publisher(
            Image, str(self.get_parameter("right_topic").value), qos_profile_sensor_data
        )
        self._imu_pub = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), qos_profile_sensor_data
        )

        self._stop = threading.Event()
        self._finished = threading.Event()
        self._failure: Exception | None = None
        self._reported_completion = False
        self._worker = threading.Thread(target=self._run, name="euroc-playback", daemon=True)
        self._worker.start()
        self._completion_timer = self.create_timer(0.1, self._check_completion)
        duration = (
            self._dataset.last_timestamp_ns - self._dataset.first_timestamp_ns
        ) / NANOSECONDS_PER_SECOND
        self.get_logger().info(
            f"Validated EuRoC sequence {self._dataset.root}: "
            f"{len(self._dataset.stereo)} stereo pairs, {len(self._dataset.imu)} IMU "
            f"samples, {duration:.2f} s"
        )

    @property
    def failure(self) -> Exception | None:
        return self._failure

    def stop(self) -> None:
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)

    def _wait_for_subscribers(self) -> None:
        if self._required_subscribers == 0:
            return
        deadline = time.monotonic() + self._subscriber_wait_sec
        while not self._stop.is_set():
            counts = (
                self._left_pub.get_subscription_count(),
                self._right_pub.get_subscription_count(),
                self._imu_pub.get_subscription_count(),
            )
            if min(counts) >= self._required_subscribers:
                self.get_logger().info(
                    "All VIF input subscribers are ready; starting EuRoC playback"
                )
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "timed out waiting for VIF input subscribers "
                    f"(left={counts[0]}, right={counts[1]}, imu={counts[2]}, "
                    f"required={self._required_subscribers})"
                )
            self._stop.wait(0.1)

    def _selection(self) -> tuple[int, int, int, int, int]:
        dataset = self._dataset
        start_ns = dataset.first_timestamp_ns + round(
            self._start_offset_sec * NANOSECONDS_PER_SECOND
        )
        end_ns = dataset.last_timestamp_ns + 1
        if self._duration_sec > 0.0:
            end_ns = min(
                end_ns,
                start_ns + round(self._duration_sec * NANOSECONDS_PER_SECOND),
            )
        stereo_stamps = [sample.timestamp_ns for sample in dataset.stereo]
        imu_stamps = [sample.timestamp_ns for sample in dataset.imu]
        stereo_begin = bisect.bisect_left(stereo_stamps, start_ns)
        stereo_end = bisect.bisect_left(stereo_stamps, end_ns)
        imu_begin = bisect.bisect_left(imu_stamps, start_ns)
        imu_end = bisect.bisect_left(imu_stamps, end_ns)
        if stereo_begin == stereo_end or imu_begin == imu_end:
            raise ValueError("the selected EuRoC time window contains no stereo or IMU data")
        first_event_ns = min(stereo_stamps[stereo_begin], imu_stamps[imu_begin])
        return stereo_begin, stereo_end, imu_begin, imu_end, first_event_ns

    def _run(self) -> None:
        stereo_published = 0
        imu_published = 0
        try:
            self._wait_for_subscribers()
            stereo_index, stereo_end, imu_index, imu_end, first_event_ns = self._selection()
            wall_start_ns = time.monotonic_ns() + 250_000_000
            while (
                not self._stop.is_set()
                and (stereo_index < stereo_end or imu_index < imu_end)
            ):
                next_stereo_ns = (
                    self._dataset.stereo[stereo_index].timestamp_ns
                    if stereo_index < stereo_end
                    else None
                )
                next_imu_ns = (
                    self._dataset.imu[imu_index].timestamp_ns
                    if imu_index < imu_end
                    else None
                )
                publish_imu = next_imu_ns is not None and (
                    next_stereo_ns is None or next_imu_ns <= next_stereo_ns
                )
                event_ns = next_imu_ns if publish_imu else next_stereo_ns
                assert event_ns is not None
                target_wall_ns = wall_start_ns + round(
                    (event_ns - first_event_ns) / self._playback_rate
                )
                self._wait_until(target_wall_ns)
                if self._stop.is_set():
                    break
                if publish_imu:
                    self._publish_imu(self._dataset.imu[imu_index])
                    imu_index += 1
                    imu_published += 1
                else:
                    self._publish_stereo(self._dataset.stereo[stereo_index])
                    stereo_index += 1
                    stereo_published += 1
            if not self._stop.is_set():
                self.get_logger().info(
                    f"EuRoC playback complete: {stereo_published} stereo pairs, "
                    f"{imu_published} IMU samples"
                )
        except Exception as error:  # surfaced on the ROS executor thread below
            self._failure = error
        finally:
            self._finished.set()

    def _wait_until(self, target_ns: int) -> None:
        while not self._stop.is_set():
            remaining_ns = target_ns - time.monotonic_ns()
            if remaining_ns <= 0:
                return
            self._stop.wait(min(remaining_ns / NANOSECONDS_PER_SECOND, 0.01))

    def _publish_stereo(self, sample: StereoSample) -> None:
        left_image = cv2.imread(str(sample.left_path), cv2.IMREAD_GRAYSCALE)
        right_image = cv2.imread(str(sample.right_path), cv2.IMREAD_GRAYSCALE)
        if left_image is None or right_image is None:
            raise RuntimeError(f"failed to decode stereo pair at {sample.timestamp_ns}")
        if left_image.shape != right_image.shape:
            raise ValueError(f"stereo dimensions differ at {sample.timestamp_ns}")
        stamp = _time_message(sample.timestamp_ns)
        self._left_pub.publish(
            self._image_message(left_image, stamp, self._left_frame_id)
        )
        self._right_pub.publish(
            self._image_message(right_image, stamp, self._right_frame_id)
        )

    @staticmethod
    def _image_message(image, stamp: Time, frame_id: str) -> Image:
        if image.ndim != 2 or image.dtype.name != "uint8":
            raise ValueError("EuRoC camera image must decode as mono8")
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height = int(image.shape[0])
        message.width = int(image.shape[1])
        message.encoding = "mono8"
        message.is_bigendian = False
        message.step = message.width
        message.data = image.tobytes(order="C")
        return message

    def _publish_imu(self, sample: ImuSample) -> None:
        message = Imu()
        message.header.stamp = _time_message(sample.timestamp_ns)
        message.header.frame_id = self._imu_frame_id
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = sample.angular_velocity[0]
        message.angular_velocity.y = sample.angular_velocity[1]
        message.angular_velocity.z = sample.angular_velocity[2]
        message.linear_acceleration.x = sample.linear_acceleration[0]
        message.linear_acceleration.y = sample.linear_acceleration[1]
        message.linear_acceleration.z = sample.linear_acceleration[2]
        self._imu_pub.publish(message)

    def _check_completion(self) -> None:
        if not self._finished.is_set() or self._reported_completion:
            return
        self._reported_completion = True
        if self._failure is not None:
            self.get_logger().error(f"EuRoC playback failed: {self._failure}")
        if self._shutdown_on_complete:
            self.context.try_shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: EurocPlayer | None = None
    failure: Exception | None = None
    try:
        node = EurocPlayer()
        rclpy.spin(node)
        failure = node.failure
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
