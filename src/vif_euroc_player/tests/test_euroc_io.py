from pathlib import Path

import pytest

from vif_euroc_player.euroc_io import load_euroc_dataset


def _write(path: Path, text: str = "") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _dataset(root: Path, right_stamp: int = 1_000_000_000) -> Path:
    for camera in ("cam0", "cam1"):
        _write(root / "mav0" / camera / "data" / "frame.png")
    _write(
        root / "mav0" / "cam0" / "data.csv",
        "#timestamp [ns],filename\n1000000000,frame.png\n",
    )
    _write(
        root / "mav0" / "cam1" / "data.csv",
        f"#timestamp [ns],filename\n{right_stamp},frame.png\n",
    )
    _write(
        root / "mav0" / "imu0" / "data.csv",
        "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n"
        "999000000,0.1,0.2,0.3,1.0,2.0,3.0\n"
        "1001000000,0.4,0.5,0.6,4.0,5.0,6.0\n",
    )
    return root


def test_loads_exact_stereo_pair_and_imu_order(tmp_path: Path) -> None:
    dataset = load_euroc_dataset(_dataset(tmp_path / "MH_01_easy"))
    assert len(dataset.stereo) == 1
    assert len(dataset.imu) == 2
    assert dataset.stereo[0].timestamp_ns == 1_000_000_000
    assert dataset.imu[0].angular_velocity == (0.1, 0.2, 0.3)
    assert dataset.imu[0].linear_acceleration == (1.0, 2.0, 3.0)
    assert dataset.first_timestamp_ns == 999_000_000
    assert dataset.last_timestamp_ns == 1_001_000_000


def test_rejects_mismatched_stereo_timestamps(tmp_path: Path) -> None:
    path = _dataset(tmp_path / "MH_01_easy", right_stamp=1_000_000_001)
    with pytest.raises(ValueError, match="stereo timestamps differ"):
        load_euroc_dataset(path)
