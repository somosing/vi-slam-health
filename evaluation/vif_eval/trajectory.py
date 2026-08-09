from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Trajectory:
    timestamps: np.ndarray
    positions: np.ndarray
    quaternions_xyzw: np.ndarray

    def __post_init__(self) -> None:
        n = len(self.timestamps)
        if self.positions.shape != (n, 3) or self.quaternions_xyzw.shape != (n, 4):
            raise ValueError("trajectory arrays have inconsistent shapes")
        if n and np.any(np.diff(self.timestamps) <= 0):
            raise ValueError("trajectory timestamps must be strictly increasing")


def read_tum(path: str | Path) -> Trajectory:
    """Read `timestamp tx ty tz qx qy qz qw`, ignoring blank/comment lines."""
    rows: list[list[float]] = []
    for line_number, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.replace(",", " ").split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 fields, got {len(fields)}")
        rows.append([float(value) for value in fields])
    if not rows:
        raise ValueError(f"trajectory is empty: {path}")
    values = np.asarray(rows, dtype=np.float64)
    order = np.argsort(values[:, 0])
    values = values[order]
    norms = np.linalg.norm(values[:, 4:8], axis=1)
    if np.any(norms < 1e-12):
        raise ValueError("trajectory contains a zero-length quaternion")
    quaternions = values[:, 4:8] / norms[:, None]
    return Trajectory(values[:, 0], values[:, 1:4], quaternions)


def associate(
    estimate: Trajectory, ground_truth: Trajectory, max_difference_sec: float
) -> tuple[np.ndarray, np.ndarray]:
    """Nearest-neighbour timestamp association with one-to-one GT matches."""
    candidates: list[tuple[float, int, int]] = []
    gt_times = ground_truth.timestamps.tolist()
    for est_idx, timestamp in enumerate(estimate.timestamps):
        insertion = bisect_left(gt_times, timestamp)
        for gt_idx in (insertion - 1, insertion):
            if 0 <= gt_idx < len(gt_times):
                difference = abs(gt_times[gt_idx] - timestamp)
                if difference <= max_difference_sec:
                    candidates.append((difference, est_idx, gt_idx))
    used_est: set[int] = set()
    used_gt: set[int] = set()
    pairs: list[tuple[int, int]] = []
    for _, est_idx, gt_idx in sorted(candidates):
        if est_idx not in used_est and gt_idx not in used_gt:
            used_est.add(est_idx)
            used_gt.add(gt_idx)
            pairs.append((est_idx, gt_idx))
    pairs.sort()
    if len(pairs) < 3:
        raise ValueError(f"only {len(pairs)} timestamp pairs found; at least 3 are required")
    return (
        np.asarray([pair[0] for pair in pairs], dtype=np.int64),
        np.asarray([pair[1] for pair in pairs], dtype=np.int64),
    )


def umeyama_alignment(
    source: np.ndarray, target: np.ndarray, with_scale: bool = False
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return R, t, s such that target ~= s * R @ source + t."""
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must both be Nx3")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = target_centered.T @ source_centered / len(source)
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vt
    scale = 1.0
    if with_scale:
        variance = np.mean(np.sum(source_centered**2, axis=1))
        if variance < 1e-15:
            raise ValueError("cannot align a trajectory with zero positional variance")
        scale = float(np.sum(singular_values * np.diag(correction)) / variance)
    translation = target_mean - scale * (rotation @ source_mean)
    return rotation, translation, scale


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion_xyzw
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_matrix(position: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_matrix(quaternion_xyzw)
    transform[:3, 3] = position
    return transform


def _summary(errors: np.ndarray, prefix: str) -> dict[str, float]:
    return {
        f"{prefix}_rmse_m": float(np.sqrt(np.mean(errors**2))),
        f"{prefix}_mean_m": float(np.mean(errors)),
        f"{prefix}_median_m": float(np.median(errors)),
        f"{prefix}_p95_m": float(np.percentile(errors, 95)),
        f"{prefix}_max_m": float(np.max(errors)),
    }


def evaluate_trajectory(
    estimate: Trajectory,
    ground_truth: Trajectory,
    max_difference_sec: float = 0.02,
    rpe_delta_sec: float = 1.0,
    align_scale: bool = False,
) -> dict[str, float | int | bool]:
    est_idx, gt_idx = associate(estimate, ground_truth, max_difference_sec)
    est_positions = estimate.positions[est_idx]
    gt_positions = ground_truth.positions[gt_idx]
    rotation, translation, scale = umeyama_alignment(est_positions, gt_positions, align_scale)
    aligned_positions = (scale * (rotation @ est_positions.T)).T + translation
    ate = np.linalg.norm(aligned_positions - gt_positions, axis=1)

    est_transforms = []
    gt_transforms = []
    for local_idx, (e_idx, g_idx) in enumerate(zip(est_idx, gt_idx, strict=True)):
        est_pose = pose_matrix(aligned_positions[local_idx], estimate.quaternions_xyzw[e_idx])
        est_pose[:3, :3] = rotation @ est_pose[:3, :3]
        est_transforms.append(est_pose)
        gt_transforms.append(
            pose_matrix(ground_truth.positions[g_idx], ground_truth.quaternions_xyzw[g_idx])
        )

    matched_times = ground_truth.timestamps[gt_idx]
    rpe_errors: list[float] = []
    for i, timestamp in enumerate(matched_times):
        j = int(np.searchsorted(matched_times, timestamp + rpe_delta_sec))
        if j >= len(matched_times):
            continue
        delta_est = np.linalg.inv(est_transforms[i]) @ est_transforms[j]
        delta_gt = np.linalg.inv(gt_transforms[i]) @ gt_transforms[j]
        relative_error = np.linalg.inv(delta_gt) @ delta_est
        rpe_errors.append(float(np.linalg.norm(relative_error[:3, 3])))

    result: dict[str, float | int | bool] = {
        "matched_poses": int(len(est_idx)),
        "estimate_poses": int(len(estimate.timestamps)),
        "ground_truth_poses": int(len(ground_truth.timestamps)),
        "trajectory_completeness": float(len(est_idx) / len(ground_truth.timestamps)),
        "duration_sec": float(matched_times[-1] - matched_times[0]),
        "scale_aligned": bool(align_scale),
        "alignment_scale": float(scale),
    }
    result.update(_summary(ate, "ate"))
    if rpe_errors:
        result.update(_summary(np.asarray(rpe_errors), "rpe_translation"))
        result["rpe_pairs"] = len(rpe_errors)
        result["rpe_delta_sec"] = float(rpe_delta_sec)
    return result
