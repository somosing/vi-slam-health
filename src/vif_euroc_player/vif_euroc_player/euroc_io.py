"""Strict readers for EuRoC's ASL stereo-inertial directory format."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class StereoSample:
    timestamp_ns: int
    left_path: Path
    right_path: Path


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]


@dataclass(frozen=True)
class EurocDataset:
    root: Path
    stereo: tuple[StereoSample, ...]
    imu: tuple[ImuSample, ...]

    @property
    def first_timestamp_ns(self) -> int:
        return min(self.stereo[0].timestamp_ns, self.imu[0].timestamp_ns)

    @property
    def last_timestamp_ns(self) -> int:
        return max(self.stereo[-1].timestamp_ns, self.imu[-1].timestamp_ns)


def _rows(path: Path) -> list[list[str]]:
    if not path.is_file():
        raise FileNotFoundError(f"missing EuRoC CSV: {path}")
    rows: list[list[str]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0].lstrip().startswith("#"):
                continue
            rows.append([field.strip() for field in row])
    if not rows:
        raise ValueError(f"EuRoC CSV contains no samples: {path}")
    return rows


def _timestamp(value: str, path: Path, line: int) -> int:
    try:
        timestamp_ns = int(value)
    except ValueError as error:
        raise ValueError(f"invalid timestamp in {path} data row {line}") from error
    if timestamp_ns <= 0:
        raise ValueError(f"non-positive timestamp in {path} data row {line}")
    return timestamp_ns


def _strictly_increasing(timestamps: list[int], path: Path) -> None:
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise ValueError(f"timestamps are not strictly increasing in {path}")


def _camera_samples(csv_path: Path) -> list[tuple[int, Path]]:
    samples: list[tuple[int, Path]] = []
    for line, row in enumerate(_rows(csv_path), start=1):
        if len(row) != 2:
            raise ValueError(f"expected 2 columns in {csv_path} data row {line}")
        filename = Path(row[1])
        if filename.name != row[1] or filename.is_absolute():
            raise ValueError(f"unsafe image filename in {csv_path} data row {line}")
        image_path = csv_path.parent / "data" / filename
        if not image_path.is_file():
            raise FileNotFoundError(f"missing EuRoC image: {image_path}")
        samples.append((_timestamp(row[0], csv_path, line), image_path))
    _strictly_increasing([sample[0] for sample in samples], csv_path)
    return samples


def _stereo_samples(mav0: Path) -> tuple[StereoSample, ...]:
    left_csv = mav0 / "cam0" / "data.csv"
    right_csv = mav0 / "cam1" / "data.csv"
    left = _camera_samples(left_csv)
    right = _camera_samples(right_csv)
    if len(left) != len(right):
        raise ValueError(
            f"EuRoC stereo row counts differ: cam0={len(left)}, cam1={len(right)}"
        )
    pairs: list[StereoSample] = []
    for index, (left_sample, right_sample) in enumerate(zip(left, right)):
        if left_sample[0] != right_sample[0]:
            raise ValueError(
                "EuRoC stereo timestamps differ at row "
                f"{index + 1}: cam0={left_sample[0]}, cam1={right_sample[0]}"
            )
        pairs.append(StereoSample(left_sample[0], left_sample[1], right_sample[1]))
    return tuple(pairs)


def _imu_samples(mav0: Path) -> tuple[ImuSample, ...]:
    imu_csv = mav0 / "imu0" / "data.csv"
    samples: list[ImuSample] = []
    for line, row in enumerate(_rows(imu_csv), start=1):
        if len(row) != 7:
            raise ValueError(f"expected 7 columns in {imu_csv} data row {line}")
        try:
            values = tuple(float(value) for value in row[1:])
        except ValueError as error:
            raise ValueError(f"invalid IMU value in {imu_csv} data row {line}") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"non-finite IMU value in {imu_csv} data row {line}")
        samples.append(
            ImuSample(
                timestamp_ns=_timestamp(row[0], imu_csv, line),
                angular_velocity=(values[0], values[1], values[2]),
                linear_acceleration=(values[3], values[4], values[5]),
            )
        )
    _strictly_increasing([sample.timestamp_ns for sample in samples], imu_csv)
    return tuple(samples)


def load_euroc_dataset(dataset_path: Path) -> EurocDataset:
    """Load metadata and validate an extracted EuRoC sequence.

    ``dataset_path`` may be either the sequence root (containing ``mav0``) or
    the ``mav0`` directory itself. Image bytes are decoded only during playback.
    """

    root = dataset_path.expanduser().resolve()
    mav0 = root / "mav0" if (root / "mav0").is_dir() else root
    if not mav0.is_dir():
        raise FileNotFoundError(f"EuRoC sequence directory does not exist: {root}")
    stereo = _stereo_samples(mav0)
    imu = _imu_samples(mav0)
    return EurocDataset(root=mav0.parent, stereo=stereo, imu=imu)
