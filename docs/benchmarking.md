# Reproducible ORB-SLAM3 benchmark

The portfolio benchmark replaces the manual four-terminal experiment with one
command. It starts and stops only the processes it owns, runs each scenario
against a fresh ORB-SLAM3 instance, records ROS state/map/fault events, and
writes machine-readable evidence.

## Prerequisites

Build the workspace with the real adapter and the diagnostics patch enabled:

```bash
export VIF_REPOSITORY=/absolute/path/to/vi-slam-health
export ORB_SLAM3_ROOT=/absolute/path/to/ORB_SLAM3

cd "$VIF_REPOSITORY"
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DVIF_BUILD_ORB_SLAM3_ADAPTER=ON \
    -DVIF_ORB_SLAM3_HAS_DIAGNOSTICS=ON \
    -DORB_SLAM3_ROOT="$ORB_SLAM3_ROOT"
```

The benchmark wrapper sources ROS and the workspace itself. By default it uses
ROS domain 42 to avoid interfering with nodes on the normal domain. Override it
by setting `ROS_DOMAIN_ID` before the command.

## Run all scenarios

```bash
export EUROC_SEQUENCE=/absolute/path/to/MH_01_easy

./tools/benchmark/run_portfolio_benchmark.sh \
  --dataset "$EUROC_SEQUENCE" \
  --vocabulary "$ORB_SLAM3_ROOT/Vocabulary/ORBvoc.txt" \
  --settings "$ORB_SLAM3_ROOT/Examples/Stereo-Inertial/EuRoC.yaml"
```

The default suite runs:

| Scenario | Injected condition | Required outcome |
|---|---|---|
| `clean` | Relay enabled, no fault | Stable `HEALTHY` for 10 s; one map; no loss/reset |
| `visual_occlusion` | Black pixels with continuous image timestamps and IMU | `HEALTHY → RELOCALIZING → RECOVERED → HEALTHY`; one map |
| `camera_blackout` | Image messages dropped while IMU continues | `HEALTHY → LOST → RESETTING → INITIALIZING → HEALTHY`; map ID changes; no false `RECOVERED` |

Every scenario has a fresh backend process. The runner waits for the ORB-SLAM3
node only after vocabulary loading is complete, then starts the fault relay and
timestamp-faithful EuRoC player. It ends the scenario as soon as the acceptance
path is complete, preventing end-of-playback sensor staleness from being
mistaken for an injected failure.

## Outputs

Each run is stored under `results/<UTC-run-id>/`:

```text
report.md                         human-readable verdict and metrics
summary.json                      complete configuration, evidence, and environment
summary.csv                       one row per scenario
clean/result.json                 scenario evidence and acceptance errors
clean/{backend,injector,player}.log
visual_occlusion/result.json
camera_blackout/result.json
```

The command exits `0` only when every selected scenario passes. A failed run is
still preserved with its logs and explicit acceptance errors.

## Useful variants

Run only same-map recovery:

```bash
./tools/benchmark/run_portfolio_benchmark.sh \
  --dataset "$EUROC_SEQUENCE" \
  --vocabulary "$ORB_SLAM3_ROOT/Vocabulary/ORBvoc.txt" \
  --settings "$ORB_SLAM3_ROOT/Examples/Stereo-Inertial/EuRoC.yaml" \
  --scenarios visual_occlusion
```

Write to an explicit directory:

```bash
... --output results/MH_01_easy-final
```

Do not shorten the 75-second playback below the configured fault window. For a
different sequence, keep the scenario settings fixed until a development run
shows that the fault begins after stable initialization.

## Metric semantics

- **Detection delay:** published fault-start event to the first observed
  `LOST`, `RELOCALIZING`, or `RESETTING` transition.
- **Same-map recovery:** fault end to `RECOVERED`, reported only when exactly
  one map ID was observed.
- **Healthy after fault:** fault end to the subsequent `HEALTHY` transition.
- **Map change:** more than one unique `TrackingStatus.map_id`.

The health score is an online diagnostic, not a calibrated probability. Ground
truth remains offline-only and is not subscribed to by the runner or detector.
