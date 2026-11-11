# RGB-D ORB-SLAM3 + Nav2 in Pixi (work in progress)

This branch starts the ROS 2 Jazzy simulation path for `vi-slam-health`.
Lidar is intentionally deferred to a separate project.

## Target system

- Pixi provides ROS 2 Jazzy, Nav2, Gazebo integration, and the Python map
  conversion utility without sourcing the host's global Jazzy installation.
- A differential-drive AMR uses an RGB-D camera and IMU in a long, connected
  Gazebo world.
- ORB-SLAM3 RGB-D + IMU estimates visual pose. The existing stereo-inertial
  mode remains available for EuRoC.
- Nav2 plans and controls motion. A frontier exploration node will later select
  goals in reachable unknown areas so the robot can explore autonomously.
- Simulator ground truth is reserved for offline ATE/RPE evaluation.

## Pixi environment

On Ubuntu 24.04, install Pixi and resolve the project environment:

```bash
pixi install
pixi run build
```

The regular task builds the default packages. ORB-SLAM3 remains an external,
version-pinned source tree, matching the repo's existing integration model.
Build that checkout with the diagnostics patch described in the root README,
then build the adapter from Pixi:

```bash
export ORB_SLAM3_ROOT=/absolute/path/to/ORB_SLAM3
pixi run build-orbslam3-adapter
```

This keeps ROS commands inside Pixi; do not source the host's
`/opt/ros/jazzy/setup.bash` for this workflow. The Pixi solve and ORB-SLAM3
build still need to be confirmed on the target Ubuntu 24.04 machine.

## Current adapter and launch entry points

The ORB-SLAM3 node accepts either `sensor_mode: stereo_imu` or
`sensor_mode: rgbd_imu`. RGB and depth images are synchronized and paired with
IMU data. RGB-D input must be RGB8 with registered depth; depth must be
`16UC1` or `32FC1`. Set ORB-SLAM3's `DepthMapFactor` in the camera settings
to match the depth units (commonly 1000 for millimetres and 1 for metres).

For a calibrated RGB-D-inertial ORB-SLAM3 settings file and vocabulary:

```bash
pixi run ros2 launch vif_bringup orbslam3_rgbd.launch.py \
  vocabulary_path:=/path/to/ORBvoc.txt \
  settings_path:=/path/to/rgbd_imu.yaml
```

The RGB-D config disables direct SLAM TF publication. The accompanying
`slam_map_odom` bridge combines the SLAM `map -> base_link` pose with the
simulator's continuous `odom -> base_link` transform and publishes
`map -> odom`. The `slam_tracking_cmd_gate` forwards Nav2 velocity only while
tracking is recent and healthy, and sends zero commands after tracking or command
timeouts.

The Nav2 launch expects a map-server YAML and camera point cloud:

```bash
pixi run ros2 launch vif_bringup nav2_rgbd.launch.py map:=maps/sim_world.yaml
```

The configured local/global obstacle layers consume
`/vif/camera/depth/points`. The Gazebo RGB-D sensor must publish registered
color/depth, camera info, and point cloud topics, and the robot must publish
`/odom` plus `odom -> base_link`. The robot model and sensor bridge that provide
these topics are still to be added.

## PCD to Nav2 occupancy map

`tools/pcd_to_nav_map.py` creates a ROS map-server PGM/YAML pair from a dense,
fused, gravity-aligned RGB-D cloud in the map frame:

```bash
pixi run python tools/pcd_to_nav_map.py data/fused_map.pcd \
  --output-prefix maps/sim_world \
  --bounds -20 40 -15 15 \
  --resolution 0.05 \
  --floor-z 0.0
```

Observed floor returns become free cells, points in the configured
obstacle-height band become occupied, and cells without either kind of evidence
stay unknown. Occupied evidence wins when a cell contains both. Tune floor and
obstacle heights for the robot/world and confirm the cloud uses the intended
gravity-aligned map frame.

A PCD does not encode camera visibility rays. The converter cannot infer free
space behind or around points, so it is intended for dense fused RGB-D surface
clouds, not ORB-SLAM3's sparse keypoint map. A complete mapping path still needs
to fuse camera depth with pose and integrate rays (or use another occupancy
mapping method) before creating the static Nav2 map.

## Next implementation stages

1. Resolve the Pixi environment on Ubuntu 24.04 and build the existing packages
   without sourcing `/opt/ros/jazzy`.
2. Add the AMR URDF/SDF, differential-drive odometry, RGB-D + IMU Gazebo
   sensors, and ROS topic/TF checks.
3. Add registered depth-to-point-cloud processing and dense depth mapping for
   static occupancy maps and rolling Nav2 obstacle observations.
4. Bring up Nav2 in the simulation and verify frame ownership, map alignment,
   tracking-loss stop behavior, and goal navigation.
5. Add frontier exploration, recovery behavior, and offline ATE/RPE against
   simulator truth only after goal navigation works on a known map.

The feature branch contains the RGB-D adapter path, launch/config, map/odom TF
bridge, tracking-gated velocity output, initial Nav2 configuration/launch,
Pixi environment, and PCD conversion utility. Full simulation, dense mapping,
frontier exploration, and validation remain outstanding; the branch is not a
claim that the complete simulator is ready yet.
