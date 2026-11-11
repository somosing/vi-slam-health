#!/usr/bin/env python3
"""Fuse a recorded RGB-D PointCloud2 bag with wheel odometry into PCD + Nav2 map.

The bag must contain PointCloud2, nav_msgs/Odometry, and /tf_static. Each cloud
is transformed camera -> base_link using the recorded static transform, then
base_link -> odom using the latest preceding odometry sample. The resulting map
is an odometry-based simulation baseline, not an ORB-SLAM3 map.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quaternion_matrix(q: np.ndarray) -> np.ndarray:
    """Return a 3x3 rotation matrix for quaternion [x, y, z, w]."""
    x, y, z, w = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-12:
        raise ValueError("Encountered a zero-length quaternion.")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def point_arrays(msg, pixel_stride: int, max_range: float):
    fields = {field.name: field for field in msg.fields}
    required = ("x", "y", "z")
    if any(name not in fields for name in required):
        raise ValueError("PointCloud2 must contain x, y, and z fields.")

    byte_order = ">" if msg.is_bigendian else "<"
    names = ["x", "y", "z"]
    formats = [byte_order + "f4"] * 3
    offsets = [fields[name].offset for name in names]
    color_name = "rgb" if "rgb" in fields else ("rgba" if "rgba" in fields else None)
    if color_name:
        names.append(color_name)
        formats.append(byte_order + "f4")
        offsets.append(fields[color_name].offset)

    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": msg.point_step,
        }
    )
    records = np.ndarray(
        shape=(msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    )[::pixel_stride, ::pixel_stride]

    xyz = np.column_stack((records["x"].ravel(), records["y"].ravel(), records["z"].ravel()))
    ranges = np.linalg.norm(xyz, axis=1)
    valid = np.isfinite(xyz).all(axis=1) & (ranges >= 0.10) & (ranges <= max_range)
    xyz = xyz[valid].astype(np.float64, copy=False)

    colors = None
    if color_name and xyz.shape[0]:
        packed = np.asarray(records[color_name]).ravel()[valid].view(
            np.dtype(byte_order + "u4")
        )
        red = ((packed >> 16) & 255).astype(np.float32)
        green = ((packed >> 8) & 255).astype(np.float32)
        blue = (packed & 255).astype(np.float32)
        colors = np.column_stack((red, green, blue)) / 255.0
    return xyz, colors


def write_nav_map(points: np.ndarray, output_prefix: Path, resolution: float,
                  padding: float, floor_z: float, floor_tolerance: float,
                  obstacle_min_z: float, obstacle_max_z: float) -> None:
    x_min = float(points[:, 0].min() - padding)
    x_max = float(points[:, 0].max() + padding)
    y_min = float(points[:, 1].min() - padding)
    y_max = float(points[:, 1].max() + padding)
    width = max(1, int(np.ceil((x_max - x_min) / resolution)))
    height = max(1, int(np.ceil((y_max - y_min) / resolution)))
    grid = np.full((height, width), 205, dtype=np.uint8)

    ix = np.floor((points[:, 0] - x_min) / resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - y_min) / resolution).astype(np.int64)
    inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    z = points[inside, 2]
    ix, iy = ix[inside], iy[inside]
    rows = height - 1 - iy

    floor = np.abs(z - floor_z) <= floor_tolerance
    obstacle = (z >= obstacle_min_z) & (z <= obstacle_max_z)
    grid[rows[floor], ix[floor]] = 254
    grid[rows[obstacle], ix[obstacle]] = 0  # occupied evidence takes precedence

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    image_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    with image_path.open("wb") as image:
        image.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        image.write(grid.tobytes())
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {image_path.name}",
                f"resolution: {resolution}",
                f"origin: [{x_min:.4f}, {y_min:.4f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(f"Wrote {image_path} and {yaml_path} ({width} x {height} cells)")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Rosbag2 directory, e.g. /tmp/rgbd_cloud_trial")
    parser.add_argument("--output-dir", type=Path, default=Path("maps/rgbd_trial"))
    parser.add_argument("--points-topic", default="/vif/camera/points")
    parser.add_argument("--odom-topic", default="/diff_drive_controller/odom")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--pixel-stride", type=int, default=16,
                        help="Sample one depth pixel per N x N block.")
    parser.add_argument("--max-range", type=float, default=5.0, help="Metres.")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="PCD voxel size in metres.")
    parser.add_argument("--map-resolution", type=float, default=0.05, help="Nav2 cell size in metres.")
    parser.add_argument("--map-padding", type=float, default=0.50, help="Map border in metres.")
    parser.add_argument("--floor-z", type=float, default=0.0)
    parser.add_argument("--floor-tolerance", type=float, default=0.08)
    parser.add_argument("--obstacle-min-z", type=float, default=0.12)
    parser.add_argument("--obstacle-max-z", type=float, default=1.80)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.bag.is_dir():
        raise SystemExit(f"Bag directory not found: {args.bag}")
    if args.pixel_stride < 1 or args.max_range <= 0 or args.voxel_size <= 0:
        raise SystemExit("Stride, range, and voxel size must be positive.")
    if args.map_resolution <= 0 or args.map_padding < 0:
        raise SystemExit("Map resolution must be positive and padding non-negative.")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    type_by_topic = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    for topic in (args.points_topic, args.odom_topic, args.tf_static_topic):
        if topic not in type_by_topic:
            raise SystemExit(f"Required topic is absent from the bag: {topic}")

    cloud_type = get_message(type_by_topic[args.points_topic])
    odom_type = get_message(type_by_topic[args.odom_topic])
    tf_type = get_message(type_by_topic[args.tf_static_topic])

    static_transforms = {}
    odom_pose = None
    odom_frame = None
    cloud_frames = set()
    point_chunks = []
    color_chunks = []
    cloud_count = 0
    skipped_clouds = 0

    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        if topic == args.tf_static_topic:
            tf_msg = deserialize_message(serialized, tf_type)
            for transform in tf_msg.transforms:
                t = transform.transform.translation
                q = transform.transform.rotation
                static_transforms[transform.child_frame_id] = (
                    transform.header.frame_id,
                    np.array([t.x, t.y, t.z], dtype=np.float64),
                    np.array([q.x, q.y, q.z, q.w], dtype=np.float64),
                )
        elif topic == args.odom_topic:
            msg = deserialize_message(serialized, odom_type)
            pose = msg.pose.pose
            p, q = pose.position, pose.orientation
            odom_pose = (
                np.array([p.x, p.y, p.z], dtype=np.float64),
                np.array([q.x, q.y, q.z, q.w], dtype=np.float64),
            )
            odom_frame = (msg.header.frame_id, msg.child_frame_id)
        elif topic == args.points_topic:
            msg = deserialize_message(serialized, cloud_type)
            cloud_frames.add(msg.header.frame_id)
            if odom_pose is None:
                skipped_clouds += 1
                continue
            frame = msg.header.frame_id
            if frame not in static_transforms:
                skipped_clouds += 1
                continue

            parent_frame, t_base_camera, q_base_camera = static_transforms[frame]
            if odom_frame and parent_frame != odom_frame[1]:
                skipped_clouds += 1
                continue

            xyz_camera, colors = point_arrays(msg, args.pixel_stride, args.max_range)
            if xyz_camera.size == 0:
                skipped_clouds += 1
                continue

            p_odom_base, q_odom_base = odom_pose
            r_odom_base = quaternion_matrix(q_odom_base)
            r_base_camera = quaternion_matrix(q_base_camera)
            xyz_odom = xyz_camera @ (r_odom_base @ r_base_camera).T
            xyz_odom += p_odom_base + r_odom_base @ t_base_camera
            point_chunks.append(xyz_odom.astype(np.float32))
            if colors is not None:
                color_chunks.append(colors.astype(np.float32))
            cloud_count += 1

    if not point_chunks:
        raise SystemExit(
            "No clouds were fused. Check the recorded /tf_static camera transform, "
            "cloud frame_id, and odometry topic."
        )
    points = np.concatenate(point_chunks, axis=0)
    if color_chunks and len(color_chunks) == len(point_chunks):
        colors = np.concatenate(color_chunks, axis=0)
    else:
        colors = np.ones_like(points, dtype=np.float32)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    if args.voxel_size > 0:
        cloud = cloud.voxel_down_sample(args.voxel_size)
    if len(cloud.points) == 0:
        raise SystemExit("Voxel filtering removed every point.")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    pcd_path = args.output_dir / "rgbd_map_odom.pcd"
    if not o3d.io.write_point_cloud(str(pcd_path), cloud, write_ascii=False, compressed=True):
        raise SystemExit(f"Failed to write {pcd_path}")
    print(f"Fused {cloud_count} clouds; skipped {skipped_clouds}; frames={sorted(cloud_frames)}")
    print(f"Odometry frames={odom_frame}; fused points={len(points):,}; voxel points={len(cloud.points):,}")
    print(f"Wrote {pcd_path}")

    write_nav_map(
        np.asarray(cloud.points),
        args.output_dir / "nav_map",
        args.map_resolution,
        args.map_padding,
        args.floor_z,
        args.floor_tolerance,
        args.obstacle_min_z,
        args.obstacle_max_z,
    )


if __name__ == "__main__":
    main()
