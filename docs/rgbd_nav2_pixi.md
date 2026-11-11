# RGB-D ORB-SLAM3 + Nav2 in Pixi (work in progress)

This branch begins the ROS 2 Jazzy simulation path for `vi-slam-health`.
Lidar is intentionally out of scope here. The later lidar project can be a
separate sensor and mapping comparison.

## Target system

- Pixi environment for ROS 2 Jazzy, Nav2, Gazebo integration, and the Python map
  conversion utility. It is isolated from the host's global Jazzy setup.
- RGB-D camera and IMU on a differential-drive AMR in a long, connected
  Gazebo world.
- ORB-SLAM3 RGB-D + IMU as the visual SLAM estimator.
- Nav2 for planning and control, followed by a frontier-based exploration node
  so the robot can autonomously cover reachable unknown space.
- Simulator ground truth is reserved for offline trajectory/map evaluation.

## Mapping contract

`tools/pcd_to_nav_map.py` creates a ROS map-server PGM/YAML pair from a dense,
fused, gravity-aligned RGB-D cloud in the map frame. Example:

```bash
pixi run python tools/pcd_to_nav_map.py data/fused_map.pcd \
  --output-prefix maps/sim_world \
  --bounds -20 40 -15 15 \
  --resolution 0.05 \
  --floor-z 0.0
```

The converter marks observed floor returns free, returns in the configured
obstacle-height band occupied, and cells without either kind of evidence
unknown. Occupied evidence wins if a cell has both. Tune the floor and obstacle
heights to the robot/world and confirm the PCD is expressed in a gravity-aligned
map frame.

A point cloud does not encode the camera's visibility rays. This conversion
therefore does not infer free space behind or around sparse points. Do not feed
ORB-SLAM3's sparse keypoint/map-point cloud to this converter and treat the
result as a complete navigation map. The intended input is a dense fused RGB-D
surface cloud; use an occupancy mapping method that integrates depth rays if
the map needs reliable free-space carving.

## Integration sequence

1. Resolve the Pixi environment on Ubuntu 24.04; build the existing packages
   without sourcing `/opt/ros/jazzy`.
2. Extend the existing ORB-SLAM3 adapter with synchronized RGB, depth, and IMU
   input and ORB-SLAM3's RGB-D-inertial tracking API. Preserve the existing
   stereo-inertial mode for EuRoC.
3. Add an AMR URDF/SDF, RGB-D + IMU Gazebo sensors, and ROS topic/TF checks.
4. Establish Nav2's TF contract: one continuous `odom -> base_link` source and
   one `map -> odom` authority. The ORB-SLAM camera pose must be transformed
   using calibrated camera/base extrinsics; it must not compete with the
   odometry or Nav2 transform publishers.
5. Provide a dense mapping path and Nav2 map server / planner / controller
   configuration. Nav2 should remain stopped when SLAM tracking is invalid.
6. Add frontier exploration, recovery behavior, and offline ATE/RPE against
   simulator truth only after navigation works on a known map.

## Current scope

This branch adds the Pixi environment and the conservative PCD conversion
utility. It does not yet claim a complete RGB-D adapter, robot simulation,
Nav2 bringup, or autonomous exploration. Those pieces depend on the tracking
and TF interfaces above and are the next implementation stages.
