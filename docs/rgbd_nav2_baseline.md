# Verified RGB-D mapping and Nav2 baseline

Verified locally on 2026-11-13 with ROS 2 Jazzy, Pixi and MuJoCo.

## Start from the repository root

Terminal 1 — simulator and axial XYZRGB converter:

    pixi run bash -c 'source install/setup.bash && ros2 launch "$PWD/rgbd_sim_fixed.launch.py"'

Terminal 2 — Nav2 in its separate environment:

    pixi run --manifest-path envs/nav2/pixi.toml ros2 launch "$PWD/nav2_odom_baseline.launch.py"

Terminal 3 — velocity adapter with command and odometry timeouts:

    pixi run --manifest-path envs/nav2/pixi.toml python tools/nav2_twist_stamper.py

Wait for cloud rates, “Managed nodes are active”, and “Command adapter ready”.

Terminal 4 — first navigation goal:

    pixi run --manifest-path envs/nav2/pixi.toml ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' --feedback

Keep the first three terminals running during navigation.

## Verified result

- 480 clouds fused with zero skips.
- Saved map: maps/rgbd_verified_scan_20261113_144153/nav_map.yaml
- Nav2 goal (1.0, 0.0): SUCCEEDED.
- Final wheel-odometry position: (0.8343, 0.0028) m.
- Position error: approximately 0.166 m; configured tolerance: 0.20 m.

## Corrections required

- MuJoCo publishes axial Z-depth: use PointCloudXyzrgbNode.
- The installed radial executable has incorrect component registration.
  The RGB-D launch creates a temporary registration for the selected component.
- Fixed caster spheres use frictionless support contacts.
  Wheel/IMU turn disagreement improved from 11.61°/2.43° to 38.00°/36.70°.
- Nav2 uses /vif/camera/points_rgb and /diff_drive_controller/odom.
- The drive controller receives TwistStamped through the velocity adapter.

## Scope

This is an offline RGB-D map fused using wheel odometry.
Identity map-to-odom is valid only while the map and running odometry
share the same origin and alignment. It does not provide relocalization.
ORB-SLAM3 integration, online mapping and long-route validation remain pending.
