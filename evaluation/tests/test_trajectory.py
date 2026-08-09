from pathlib import Path

import numpy as np

from vif_eval.trajectory import evaluate_trajectory, read_tum


def _write_tum(path: Path, positions: np.ndarray) -> None:
    lines = [
        f"{index:.3f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1"
        for index, (x, y, z) in enumerate(positions)
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_rigid_alignment_removes_global_transform(tmp_path):
    ground_truth = np.asarray([[i, 0.2 * i * i, 0.1 * i] for i in range(8)], dtype=float)
    angle = 0.45
    rotation = np.asarray(
        [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
    )
    estimate = (rotation.T @ (ground_truth - np.asarray([4.0, -2.0, 1.0])).T).T
    est_path = tmp_path / "estimate.tum"
    gt_path = tmp_path / "groundtruth.tum"
    _write_tum(est_path, estimate)
    _write_tum(gt_path, ground_truth)

    result = evaluate_trajectory(read_tum(est_path), read_tum(gt_path), rpe_delta_sec=1.0)
    assert result["matched_poses"] == 8
    assert result["ate_rmse_m"] < 1e-8


def test_reader_rejects_bad_field_count(tmp_path):
    path = tmp_path / "bad.tum"
    path.write_text("0 1 2\n", encoding="utf-8")
    try:
        read_tum(path)
    except ValueError as error:
        assert "expected 8 fields" in str(error)
    else:
        raise AssertionError("read_tum should reject malformed rows")
