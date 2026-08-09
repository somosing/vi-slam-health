# ORB-SLAM3 integration

`vif_orbslam3` is the real stereo-inertial backend for the VIF interface. It
links to an external ORB-SLAM3 tree so the upstream source, vocabulary, and
datasets are not copied into this repository.

## Pinned backend

The integration is reviewed for this environment:

| Component | Pinned value |
|---|---|
| ORB-SLAM3 | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` |
| Compiler | GCC 13.3.0 |
| C++ mode used by ORB-SLAM3 | C++14 |
| OpenCV | 4.6.0 |
| Eigen | 3.4.0 |
| Pangolin | 0.9.2 |
| ROS | ROS 2 Jazzy on Ubuntu 24.04 |
| Sophus | the copy bundled by the pinned ORB-SLAM3 revision |

The local ORB-SLAM3 tree changes its top-level `CMakeLists.txt` from C++11 to
C++14 for the installed Pangolin/sigslot headers. `build.sh` uses `make -j1` on
the target laptop to avoid memory pressure; that job-count change does not
alter the binary interface.

The CMake configuration rejects a different Git revision unless
`VIF_ALLOW_UNVERIFIED_ORB_SLAM3_REVISION=ON` is explicitly provided. That
override is for reviewed upgrades, not normal use.

## Apply the diagnostics patch

The upstream `System` API provides pose, state, map points, and keypoints, but
not the inlier ratio, accepted-correspondence reprojection residual, or active
map ID needed by `TrackingStatus`. Apply the small read-only patch:

```bash
export VIF_REPOSITORY=/absolute/path/to/vi-slam-health
export ORB_SLAM3_ROOT=/absolute/path/to/ORB_SLAM3

cd "$ORB_SLAM3_ROOT"

git rev-parse HEAD
git apply --check \
  "$VIF_REPOSITORY/third_party/patches/orb_slam3/0001-expose-tracking-diagnostics.patch"
git apply \
  "$VIF_REPOSITORY/third_party/patches/orb_slam3/0001-expose-tracking-diagnostics.patch"

./build.sh
```

The revision command must print the pinned hash. If the local source directory
is elsewhere, replace only the paths.

The adapter also supports an unpatched build by setting
`VIF_ORB_SLAM3_HAS_DIAGNOSTICS=OFF`. In that mode it publishes `NaN` for inlier
ratio and reprojection error. The health monitor excludes those unavailable
values and renormalizes the remaining weights; it never substitutes invented
healthy values.

## Build the ROS 2 adapter

First build the backend-independent workspace normally. Then build the real
adapter with the external tree enabled:

```bash
cd "$VIF_REPOSITORY"
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select vif_interfaces vif_orbslam3 vif_sensor_frontend \
    vif_health_monitor vif_relocalization vif_bringup \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DVIF_BUILD_ORB_SLAM3_ADAPTER=ON \
    -DVIF_ORB_SLAM3_HAS_DIAGNOSTICS=ON \
    -DORB_SLAM3_ROOT="$ORB_SLAM3_ROOT"

source install/setup.bash
```

If the diagnostics patch is intentionally not applied, change only the
diagnostics option to `OFF`.

## Run

The launch file starts the sensor guard, ORB-SLAM3 adapter, health monitor, and
relocalization manager:

```bash
ros2 launch vif_bringup orbslam3.launch.py \
  vocabulary_path:="$ORB_SLAM3_ROOT/Vocabulary/ORBvoc.txt" \
  settings_path:="$ORB_SLAM3_ROOT/Examples/Stereo-Inertial/EuRoC.yaml" \
  use_viewer:=true
```

It expects synchronized, rectified input on:

| Topic | Type |
|---|---|
| `/vif/camera/left/image_raw` | `sensor_msgs/msg/Image` |
| `/vif/camera/right/image_raw` | `sensor_msgs/msg/Image` |
| `/vif/imu` | `sensor_msgs/msg/Imu` |

Camera and IMU messages use sensor-data QoS. The adapter buffers every
monotonic IMU sample in `(previous stereo stamp, current stereo stamp]`, rejects
non-monotonic input, briefly waits for IMU delivery to catch up to each camera
timestamp, and bounds the buffer to prevent unbounded memory growth.

For EuRoC, the included player starts this stack and preserves the original
camera/IMU timestamps and measurement order:

```bash
export EUROC_SEQUENCE=/absolute/path/to/MH_01_easy

ros2 launch vif_bringup euroc_orbslam3.launch.py \
  dataset_path:="$EUROC_SEQUENCE" \
  vocabulary_path:="$ORB_SLAM3_ROOT/Vocabulary/ORBvoc.txt" \
  settings_path:="$ORB_SLAM3_ROOT/Examples/Stereo-Inertial/EuRoC.yaml"
```

The player waits until both the sensor guard and ORB-SLAM3 subscribe to all
three inputs, preventing the vocabulary-loading interval from dropping the
beginning of the sequence.

## Pose and frame semantics

`TrackStereo()` returns `T_camera_world`. The adapter inverts it and publishes:

```text
T_map_base = T_map_orb_world * inverse(T_camera_world) * T_camera_base
```

- `orb_world_to_map` is `T_map_orb_world`. Its default rotates ORB optical axes
  `(right, down, forward)` into REP-103 map axes `(forward, left, up)`.
- `base_to_camera` is `T_camera_base`: it maps coordinates from `base_frame`
  into the left-camera optical frame.

The supplied configuration uses an identity `base_to_camera` and therefore
names the child frame `camera_left_optical_frame`. For a robot `base_link`, set
the calibrated camera/base extrinsic explicitly. Do not copy EuRoC calibration
onto a physical sensor.

Odometry is published when ORB-SLAM3 has a current pose. During loss, the last
pose may remain visible with very large covariance, but `map -> base` TF stops.
Consumers must use `/vif/localization/health.pose_valid`, not odometry arrival,
to decide whether localization is usable.

## State and recovery mapping

| ORB-SLAM3 state | VIF state |
|---|---|
| `SYSTEM_NOT_READY`, `NO_IMAGES_YET` | `NOT_INITIALIZED` |
| `NOT_INITIALIZED` | `INITIALIZING` |
| `OK`, `OK_KLT` | `TRACKING` |
| `RECENTLY_LOST` | `RELOCALIZING` |
| `LOST` | `LOST` |

`REQUEST_RELOCALIZATION` truthfully acknowledges ORB-SLAM3's automatic
place-recognition recovery while frames continue. It does not claim a separate
upstream API call exists. `RESET_ACTIVE_MAP` calls `ResetActiveMap()` and
`START_NEW_MAP` calls `ChangeDataset()`. Command acceptance means scheduled or
executed; later tracking messages determine whether recovery succeeded.

A one-message `relocalization_event` pulse is emitted when tracking returns to
`OK` or `OK_KLT` after a recently-lost/lost interval.

## Controlled fault semantics

Use `visual_occlusion` to test same-map relocalization. The relay replaces both
camera images with black pixels during the fault window but forwards every
original image header and all IMU samples. Camera timestamps therefore remain
continuous, and recovery is valid only if the observer sees one map ID:

```bash
ros2 run vif_fault_injection fault_injector --ros-args \
  -p fault_type:=visual_occlusion \
  -p start_after_sec:=45.0 \
  -p duration_sec:=1.0

python3 tools/benchmark/observe_demo.py \
  --timeout 60 \
  --expected-path same-map-recovery
```

Use `camera_blackout` to test input-loss handling. It intentionally drops image
messages. When tracking is `LOST` and health diagnostics report
`camera_unavailable`, the recovery manager requests an active-map reset instead
of same-map relocalization. The correct outcome is `RESETTING`, at least two
observed map IDs, and no false `RECOVERED` state:

```bash
python3 tools/benchmark/observe_demo.py \
  --timeout 60 \
  --expected-path blackout-reset
```

Do not treat the two scenarios as interchangeable: one validates
place-recognition recovery on an existing map, while the other validates
truthful reset reporting after a sensor-stream outage.

## Validation sequence

1. Run the existing synthetic contract test with `vif_mock_slam`.
2. Build the adapter against the pinned backend and run its unit tests.
3. Play EuRoC `MH_01_easy` into the three VIF input topics and compare the ROS
   odometry trajectory with the already-validated native baseline.
4. Verify the coordinate transform with a known forward/right/up motion before
   using a physical sensor.
5. Run failure sequences and confirm that TF stops, covariance inflates,
   `pose_valid` becomes false, and recovery events match state transitions.

The preferred execution path for steps 3–5 is the
[one-command benchmark](benchmarking.md), which isolates the scenarios, records
fault and state timing, verifies map identity, and preserves all raw logs.

The native baseline for this machine produced 3,678 poses over 183.85 seconds
on `MH_01_easy`, with translational APE RMSE `0.040566 m` after SE(3) alignment.
