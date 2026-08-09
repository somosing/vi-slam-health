# Changelog

## 0.3.0 — 2026-08-09

- Added a one-command real-backend benchmark for clean tracking, timestamp-continuous
  visual occlusion, and camera-blackout reset behavior.
- Added machine-readable fault start/end events without connecting labels to the
  online health or recovery path.
- Added explicit acceptance checks for state order, pose invalidation, map identity,
  forbidden false-recovery states, and stable clean tracking.
- Added per-scenario raw logs plus JSON, CSV, and Markdown benchmark reports.
- Added pure-Python unit tests for benchmark acceptance and report generation.
- Preserved reset map generations across backend diagnostics and enforced the
  blackout path `HEALTHY → LOST → RESETTING → INITIALIZING → HEALTHY`.
- Reframed the README around the verified engineering result and added a case study,
  reproducibility protocol, reference evidence, and a portfolio demo script.

## 0.2.0 — 2026-08-07

- Added the pinned `vif_orbslam3` stereo-inertial ROS 2 backend.
- Added bounded IMU buffering, stereo synchronization, explicit REP-103 pose conversion,
  odometry/TF loss behavior, and truthful management-service handling.
- Added the version-controlled ORB-SLAM3 diagnostics patch and unpatched `NaN` fallback.
- Updated the health core to exclude unavailable signals and renormalize active weights.
- Added real-backend launch/configuration, tests, and integration documentation.
- Added timestamp-faithful EuRoC stereo-inertial playback and a complete
  EuRoC/ORB-SLAM3 launch path.
- Preserved recovery context across transient backend initialization, exposed
  active-map changes as `RESETTING`, and prevented reset/reinitialization from
  being reported as successful relocalization.
- Added timestamp-continuous visual occlusion for genuine same-map recovery
  tests and ordered observers for recovery versus reset validation paths.

## 0.1.0 — 2026-08-06

- Added backend-independent ROS 2 health and recovery interfaces.
- Added sensor timing guard, explainable health state machine, and recovery manager.
- Added deterministic mock SLAM, synthetic stereo/IMU traffic, and fault injection.
- Added trajectory and failure/recovery evaluation metrics.
- Added Jazzy Docker build, CI, documentation, tests, and EuRoC download helper.
