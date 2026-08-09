# ROS interface contract

## Backend obligations

A real backend replaces `vif_mock_slam` and must provide:

1. `/vif/slam/tracking` at the image processing rate.
2. `/vif/slam/odometry` whenever a pose estimate exists.
3. `/vif/slam/manage` with truthful acceptance responses.
4. `map -> base_link` TF only while the pose is usable, or another documented frame chain.

### TrackingStatus semantics

| Field | Required meaning |
|---|---|
| `tracked_features` | Features participating in the current tracking solution, not all detected keypoints |
| `inlier_ratio` | Geometric inliers divided by tested correspondences, in `[0,1]` |
| `mean_reprojection_error_px` | Mean error over accepted correspondences in pixels |
| `processing_time_ms` | End-to-end backend time for the current camera update |
| `state` | Backend-native state mapped to the closest interface constant |
| `map_id` | Changes whenever the active global map identity changes |
| `relocalization_event` | One-message pulse after the backend accepts a relocalized pose |

If a backend cannot expose a floating-point signal, publish `NaN`. The health
monitor excludes unavailable values and renormalizes the active weights. Do
not fabricate healthy values. A backend-specific configuration may also set a
known-unavailable signal's static weight to zero.

### ManageSlam semantics

- `REQUEST_RELOCALIZATION`: enable/continue place-recognition recovery without destroying the active map.
- `RESET_ACTIVE_MAP`: clear the active tracking context. This may invalidate the global frame.
- `START_NEW_MAP`: preserve the atlas if supported and create a new active map.

`accepted=true` means the command was scheduled or executed, not that recovery succeeded. Success is verified from subsequent tracking and health messages.

## QoS

Camera and IMU inputs use sensor-data QoS. Health, tracking status, odometry, events, and the management service use reliable QoS. Tune history depth based on measured processing latency; do not use an unbounded queue.

## Fault-event evidence

`/vif/fault/events` publishes `vif_interfaces/msg/FaultEvent` at the exact start
and end of an injected fault window. The event includes the fault type and
injector-relative elapsed time. It is benchmark evidence only: neither the
health monitor nor the recovery manager subscribes to it, so labelled fault
timing cannot leak into online failure detection.
