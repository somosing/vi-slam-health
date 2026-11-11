# MuJoCo simulation environment

The first simulation asset is a single-floor, connected indoor environment for
long-range visual navigation. It is a 60 m × 18 m floorplan: a 60 m central
corridor, ten side bays, doorways between bays, furniture/columns as visual
landmarks, and non-repeating wall colors. It is one continuous floor, not a
multi-storey building.

The scene is in `worlds/long_indoor_world.xml`. The robot is intentionally
added in the next integration step so the environment geometry can be reviewed
independently. The planned robot is a differential-drive AMR with RGB-D, IMU,
wheel encoders, and camera-to-base calibration.

## Why MuJoCo

MuJoCo is compatible with ROS 2 Jazzy through the ROS 2 Control
`mujoco_ros2_control` project. Its Jazzy plugins can publish RGB, depth,
camera info, and IMU-related data from MJCF models. The upstream project also
documents a Pixi + RoboStack development workflow. We will build that source
inside this repository's Pixi workspace rather than add Gazebo packages.

The camera and sensor plugins use MuJoCo's rendering conventions, so optical
frame orientation, registered depth, camera info, and image encoding must be
checked against ORB-SLAM3 before Nav2 is connected.

## Planned sequence

1. Resolve the Pixi/Jazzy/MuJoCo source workspace and open this environment.
2. Add the differential-drive robot and wheel encoder interfaces.
3. Add RGB-D camera and IMU topics and wire them to the RGB-D-inertial
   ORB-SLAM3 adapter.
4. Fuse depth observations with tracked poses into an occupancy map. Evaluate
   OctoMap or a 2D ray-integration mapper; ORB-SLAM3's sparse landmarks are not
   an occupancy grid.
5. Convert the 3D map to a Nav2 `OccupancyGrid`, configure Nav2, and add a
   frontier goal selector for autonomous exploration.

The existing PCD converter is an offline static-map utility. It preserves
unobserved cells as unknown and is not the online mapper for this world.
