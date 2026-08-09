# Public dataset policy

Dataset files are never committed to this repository. Download from the official host, retain attribution and terms, store checksums in experiment metadata, and keep data under `data/` (gitignored).

| Dataset | Project use | Notes to verify before use |
|---|---|---|
| EuRoC MAV | Development, regression, primary stereo-inertial benchmark | Stereo + IMU + calibration + motion ground truth |
| TUM-VI | Cross-dataset fisheye and long-sequence evaluation | Ground truth coverage differs by sequence |
| UZH-FPV | Aggressive-motion stress testing | Non-commercial/share-alike terms apply to published dataset |
| Hilti SLAM Challenge | Later construction-site stress testing | Large multimodal data and challenge-specific terms |

Official sources: [EuRoC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), [TUM-VI](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset), [UZH-FPV](https://fpv.ifi.uzh.ch/), and [Hilti SLAM Challenge](https://hilti-challenge.com/dataset-2022.html).

“Publicly available” does not automatically mean unrestricted redistribution or commercial use. Record the official dataset page and license/terms snapshot with each benchmark release.

## EuRoC split

- Development: `MH_01_easy`, `V1_01_easy`, `V2_01_easy`
- Validation: `MH_03_medium`, `V1_02_medium`, `V2_02_medium`
- Held-out test: `MH_02_easy`, `MH_04_difficult`, `MH_05_difficult`, `V1_03_difficult`, `V2_03_difficult`

Never tune health thresholds on the held-out sequences. Fault-injected variants inherit the split of their clean source sequence.

## Download

The EuRoC helper accepts one allow-listed sequence and downloads it atomically:

```bash
python3 tools/datasets/download_euroc.py MH_01_easy --destination data/euroc
```

The helper does not accept arbitrary URLs and does not redistribute data.

## EuRoC ROS 2 playback

The `vif_euroc_player` package reads an extracted ASL-format sequence, validates
strictly increasing timestamps and exact left/right pairing, and publishes all
measurements without changing their timestamps. It decodes images on demand, so
the complete sequence is not loaded into memory.

```bash
export EUROC_SEQUENCE=/absolute/path/to/MH_01_easy

ros2 launch vif_bringup euroc_orbslam3.launch.py \
  dataset_path:="$EUROC_SEQUENCE"
```

`playback_rate` controls only wall-clock pacing; message timestamps always remain
the original EuRoC nanoseconds. `start_offset_sec` and `duration_sec` may be used
for smoke tests. A zero duration plays the complete sequence.
