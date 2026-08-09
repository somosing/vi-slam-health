# 90-second portfolio demo script

## 0:00–0:15 — The problem

Show the ORB-SLAM3 trajectory and the localization-health topic side by side.

> A SLAM system may still publish odometry while tracking is unusable. I built a
> ROS 2 health and recovery layer that tells downstream software when the pose is
> valid and distinguishes same-map recovery from a map reset.

## 0:15–0:35 — Architecture

Show the README architecture diagram. Point to the sensor guard, ORB-SLAM3
adapter, health monitor, and recovery manager. Mention that ground truth is used
only for offline evaluation.

## 0:35–0:58 — Same-map recovery

Run the `visual_occlusion` scenario. Show the transition line from the benchmark:

```text
HEALTHY → RELOCALIZING → RECOVERED → HEALTHY
map IDs: [0]
```

> The images are black for one second, but timestamps and IMU remain continuous.
> The system detects the failure, ORB-SLAM3 relocalizes against the existing map,
> and downstream pose validity returns only after stable tracking.

## 0:58–1:15 — Why blackout is different

Show the blackout result:

```text
HEALTHY → LOST → RESETTING → INITIALIZING → HEALTHY
map IDs: [0, 1]
```

> If camera messages disappear, resuming them creates a timestamp discontinuity.
> ORB-SLAM3 starts a new map. My state machine reports a reset instead of falsely
> claiming relocalization success.

## 1:15–1:30 — Reproducibility

Show `report.md`, `summary.csv`, and one scenario log directory.

> One command runs the clean, occlusion, and blackout scenarios and saves the
> configuration, environment, state transitions, map IDs, timings, raw logs, and
> pass/fail verdict. The same acceptance logic is unit-tested in CI.

Finish on the short result table in the README. Keep terminal text large enough
to read and avoid showing home-directory details or unrelated windows.
