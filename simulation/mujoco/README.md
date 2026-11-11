# MuJoCo simulation environment

The MuJoCo scene is a connected, single-floor 60 m × 18 m indoor map for long-range visual navigation. It contains a central corridor, side bays and doorways, furniture, columns, and varied visual landmarks. The floorplan is long and connected; it is not a multi-storey building.

The scene now includes a differential-drive AMR with independently actuated left and right wheels, wheel position/velocity feedback for encoder odometry, a forward-facing RGB-D camera, and an IMU. Lidar is intentionally omitted.

## Pixi workflow

From the repository root, bootstrap the pinned upstream ROS 2 Control simulator sources, resolve Pixi, and build the workspace:

```bash
bash tools/bootstrap_mujoco_workspace.sh
pixi install
pixi run build
```

Open the MuJoCo scene directly:

```bash
pixi run view-world
```

Launch the integrated ROS 2 simulator, camera plugin, IMU broadcaster, and differential-drive controller:

```bash
pixi run mujoco-amr
```

Use `headless:=true` when launching without a display:

```bash
pixi run ros2 launch vif_bringup mujoco_sim.launch.py headless:=true
```

Expected simulator interfaces:

| Interface | Topic |
| --- | --- |
| RGB image | `/vif/camera/color/image_raw` |
| Registered depth image | `/vif/camera/depth/image_raw` |
| Camera calibration | `/vif/camera/color/camera_info` |
| IMU | `/vif/imu` |
| Wheel encoder odometry | `/diff_drive_controller/odom` |
| Drive command | `/diff_drive_controller/cmd_vel_unstamped` |

Check the runtime topic names and message encodings on the target Ubuntu 24.04/Pixi environment before starting ORB-SLAM3. In particular, confirm the camera plugin's depth encoding and ensure the ORB-SLAM3 `DepthMapFactor` in the selected camera settings matches the published depth units.

## Integration plan

1. Run and inspect the Pixi MuJoCo environment.
2. Verify camera, depth, IMU, wheel-state, and odometry streams and their frame transforms.
3. Run RGB-D-inertial ORB-SLAM3 using simulator images and IMU data; keep EuRoC playback as the dataset baseline.
4. Integrate depth observations with poses into an online occupancy map. OctoMap or a 2D ray-integration mapper can produce a 2D Nav2 `OccupancyGrid`; ORB-SLAM3 sparse landmarks are not a navigation map.
5. Configure Nav2 and add frontier-based autonomous exploration over the long map.
6. Evaluate simulator ground truth offline with ATE/RPE; do not feed simulator truth into SLAM or navigation.

The existing PCD converter is an offline static-map utility. A PCD has no visibility rays, so it cannot identify unseen cells by itself. The online mapper must integrate depth observations using sensor poses and preserve unobserved space as unknown.

MuJoCo's camera conventions differ from ROS optical-frame conventions. The camera-to-base transform and depth conversion must be validated against actual published messages before SLAM results are treated as meaningful.
