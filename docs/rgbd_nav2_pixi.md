# MuJoCo + ORB-SLAM3 + Nav2 in Pixi

This branch integrates the MuJoCo simulator in stages. The environment is a
long, connected, single-floor map; lidar is deferred.

## Stage 1: simulation environment and long map

`simulation/mujoco/worlds/long_indoor_world.xml` defines a 60 m × 18 m indoor
floorplan with a long central corridor, side bays, doorways, furniture, and
visual landmarks. It is one continuous floor, not a multi-storey building.

ROS 2 Jazzy support uses the upstream `mujoco_ros2_control` source and its
RGB-D camera plugin. The pinned source setup is handled by
`tools/bootstrap_mujoco_workspace.sh`.

On Ubuntu 24.04, run from the repository root:

```bash
bash tools/bootstrap_mujoco_workspace.sh
pixi install
pixi run build
```

The bootstrap script clones the pinned sources into ignored
`src/third_party` directories. Use Pixi's ROS 2 Jazzy environment for this
branch.

## Stage 2: AMR and sensor topics

The scene now contains a differential-drive AMR with independently actuated
wheels, wheel position/velocity feedback, a forward-facing RGB-D camera, and an
IMU. The ROS description exposes the wheel joints and IMU sensor to
`ros2_control`. The controller setup uses wheel feedback for odometry and
the standard IMU sensor broadcaster.

Launch the integrated simulation:

```bash
pixi run mujoco-amr
```

For a machine without a graphical display:

```bash
pixi run ros2 launch vif_bringup mujoco_sim.launch.py headless:=true
```

The launch config targets these topics:

| Interface | Topic |
| --- | --- |
| RGB image | `/vif/camera/color/image_raw` |
| Registered depth image | `/vif/camera/depth/image_raw` |
| Camera calibration | `/vif/camera/color/camera_info` |
| IMU | `/vif/imu` |
| Wheel encoder odometry | `/diff_drive_controller/odom` |
| Drive command | `/diff_drive_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`) |

Jazzy's `diff_drive_controller` expects a stamped `Twist` on
`~/cmd_vel`. The `use_stamped_vel` switch used by older releases is not
available in the Jazzy controller.

Before connecting ORB-SLAM3, inspect topic availability, image encoding and
timestamps, camera calibration, optical-frame axes, depth units, and the
`odom -> base_link` transform. The MuJoCo camera publishes `32FC1` depth
values in metres, so ORB-SLAM3 RGB-D settings should use
`DepthMapFactor: 1.0`. These interfaces are configured but still need to be
run on the target Ubuntu 24.04 Pixi setup.

EuRoC playback remains the dataset baseline. Simulator ground truth is reserved
for offline ATE/RPE evaluation and must not be fed to SLAM or navigation.

## Stage 3: online 3D mapping and Nav2 map

The default Pixi environment currently covers the simulator and sensor bringup.
Nav2 and OctoMap packages are deferred until this stage because the available
RoboStack Nav2 bringup pulls ROS-Gazebo packages and mixed ROS distro-mutex
builds into the MuJoCo environment, preventing Pixi from solving it. Add and
resolve those packages when implementing this stage.

The planned online mapper is OctoMap or a 2D ray-integration mapper. It must
integrate RGB-D observations with sensor poses, preserve unknown space, and
produce a projected 2D `OccupancyGrid` for Nav2. The 2D projection height band
will be tuned around the AMR floor and body so overhead structures do not
become ground obstacles.

The existing `tools/pcd_to_nav_map.py` is an offline fallback for dense fused
clouds that include floor returns. A PCD has no visibility rays, so it cannot
recover free space that was never observed. ORB-SLAM3 sparse landmarks are not
a complete occupancy map.

Once mapping works, Nav2 will use the projected grid as its map layer and the
RGB-D cloud for near-field obstacle observations. A frontier goal selector will
then let the robot explore reachable unknown-space boundaries in the long map.

## Next integration steps

1. Run the AMR launch and verify images, IMU, wheel states/odometry, and TF.
2. Connect the verified RGB-D-inertial adapter and validate ORB-SLAM3 tracking.
3. Add online 3D-to-2D occupancy mapping and verify the projected map.
4. Bring up Nav2, then add frontier-based autonomous exploration.
5. Evaluate localization against simulator ground truth offline with ATE/RPE.
