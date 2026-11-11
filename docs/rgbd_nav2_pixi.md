# MuJoCo + ORB-SLAM3 + Nav2 in Pixi

This branch is being integrated in stages. The active simulator choice is
MuJoCo, the environment is a long single-floor map, and lidar is deferred.

## Stage 1: simulation environment and long map

The first asset is `simulation/mujoco/worlds/long_indoor_world.xml`: a
60 m × 18 m connected indoor floorplan with a long central corridor, side bays,
doorways, furniture, and visual landmarks. It is one continuous floor, not a
multi-storey building. Robot and sensor bodies will be added after the map is
reviewed.

ROS 2 Jazzy has an upstream MuJoCo integration in
`mujoco_ros2_control`. Its plugins include RGB-D camera and IMU support, and
the upstream project documents a Jazzy Pixi/RoboStack workflow. We pin the
upstream MuJoCo ROS packages and vendor source with
`tools/bootstrap_mujoco_workspace.sh`.

On Ubuntu 24.04, first fetch those pinned sources, then resolve the isolated
Pixi workspace:

```bash
bash tools/bootstrap_mujoco_workspace.sh
pixi install
pixi run build
```

The bootstrap script only clones the two upstream sources into the ignored
`src/third_party` directory. The build uses Pixi's ROS 2 Jazzy environment;
do not source the global `/opt/ros/jazzy/setup.bash` for this workflow.

## Stage 2: robot and sensors

The next integration step adds a differential-drive AMR with wheel encoder
interfaces, registered RGB-D camera, and IMU. We will check camera calibration,
optical-frame axes, depth encoding/scale, synchronized timestamps, and the
`odom -> base_link` transform before connecting ORB-SLAM3.

The existing adapter has an RGB-D-inertial mode and dedicated launch/config
files. Its expected inputs are RGB8, registered 16UC1 or 32FC1 depth, and IMU.
ORB-SLAM3's `DepthMapFactor` must match the depth units.

## Stage 3: 3D mapping and Nav2 map

For online mapping, the first candidate is OctoMap's Jazzy `octomap_server`.
It incrementally integrates sensor `PointCloud2` measurements using their
sensor poses, maintains unknown/free/occupied 3D space, and publishes a
projected 2D `OccupancyGrid`. We will set the projection height band around
the AMR's floor and body so overhead structures do not become ground obstacles.
The depth camera plugin's point cloud will be the mapping input.

The existing `tools/pcd_to_nav_map.py` is an offline fallback for dense fused
clouds that include floor returns. It marks observed floor as free, obstacle
height as occupied, and unsupported cells as unknown. A PCD alone has no
visibility rays, so it cannot recover free space that was never observed. It
also must not be fed ORB-SLAM3's sparse keypoint map as if that were a complete
occupancy map.

Nav2 will consume the projected grid as its map layer and use the same RGB-D
cloud for near-field obstacle observations. We will then add a frontier
explorer so Nav2 can select reachable unknown-space boundaries and explore the
long environment on its own.

## Integration order

1. Resolve Pixi + MuJoCo ROS 2 sources and inspect the long world.
2. Add the robot, wheel/odom interfaces, RGB-D camera, and IMU.
3. Verify the ORB-SLAM3 RGB-D-inertial path and its pose/TF conventions.
4. Integrate OctoMap and tune its 2D height projection.
5. Bring up Nav2, verify goals on the occupancy map, then add frontier
   exploration and offline ATE/RPE against simulator ground truth.

The long-world asset and Pixi bootstrap are now in the branch. The robot,
sensor wiring, OctoMap launch, Nav2 integration, exploration, and runtime
validation are subsequent stages.
