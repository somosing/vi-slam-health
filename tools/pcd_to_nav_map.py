#!/usr/bin/env python3
"""Convert a gravity-aligned, fused RGB-D point cloud into a conservative Nav2 map.

A PCD has no visibility rays. This converter therefore marks only observed floor
cells as free, points in the configured obstacle-height band as occupied, and
leaves all other cells unknown. It is intended for dense fused RGB-D clouds,
not ORB-SLAM3's sparse keypoint map.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=4,
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
        required=True,
        help="Map-frame bounds in metres.",
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--floor-z", type=float, default=0.0)
    parser.add_argument("--floor-tolerance", type=float, default=0.06)
    parser.add_argument("--obstacle-min-z", type=float, default=0.12)
    parser.add_argument("--obstacle-max-z", type=float, default=1.80)
    parser.add_argument("--unknown-pixel", type=int, default=205)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    x_min, x_max, y_min, y_max = args.bounds
    if not (x_min < x_max and y_min < y_max):
        raise SystemExit("Bounds must have positive width and height.")
    if args.resolution <= 0:
        raise SystemExit("Resolution must be positive.")
    if args.floor_tolerance < 0 or args.obstacle_min_z >= args.obstacle_max_z:
        raise SystemExit("Invalid floor tolerance or obstacle-height band.")
    if not 0 <= args.unknown_pixel <= 255:
        raise SystemExit("--unknown-pixel must be in [0, 255].")

    cloud = o3d.io.read_point_cloud(str(args.pcd))
    points = np.asarray(cloud.points)
    if points.size == 0:
        raise SystemExit(f"No points found in {args.pcd}")
    points = points[np.isfinite(points).all(axis=1)]
    if points.size == 0:
        raise SystemExit("The PCD contains no finite points.")

    width = int(np.ceil((x_max - x_min) / args.resolution))
    height = int(np.ceil((y_max - y_min) / args.resolution))
    grid = np.full((height, width), args.unknown_pixel, dtype=np.uint8)

    ix = np.floor((points[:, 0] - x_min) / args.resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - y_min) / args.resolution).astype(np.int64)
    inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    points, ix, iy = points[inside], ix[inside], iy[inside]

    floor = np.abs(points[:, 2] - args.floor_z) <= args.floor_tolerance
    obstacle = (
        (points[:, 2] >= args.obstacle_min_z)
        & (points[:, 2] <= args.obstacle_max_z)
    )

    # PGM rows run top-to-bottom while map y increases bottom-to-top.
    row = height - 1 - iy
    grid[row[floor], ix[floor]] = 254
    grid[row[obstacle], ix[obstacle]] = 0  # occupied evidence wins

    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    image_path = args.output_prefix.with_suffix(".pgm")
    yaml_path = args.output_prefix.with_suffix(".yaml")
    with image_path.open("wb") as image:
        image.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        image.write(grid.tobytes())
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {image_path.name}",
                f"resolution: {args.resolution}",
                f"origin: [{x_min}, {y_min}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )

    n_floor = int(floor.sum())
    n_obstacle = int(obstacle.sum())
    print(f"Wrote {image_path} and {yaml_path}")
    print(f"Grid: {width} x {height} at {args.resolution:g} m/cell")
    print(f"Evidence points: floor={n_floor}, obstacle={n_obstacle}")


if __name__ == "__main__":
    main()
