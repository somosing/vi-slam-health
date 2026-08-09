# Engineering case study

## Problem

A visual-inertial SLAM backend can keep publishing a pose even when tracking is
no longer trustworthy. Downstream software therefore needs more than odometry:
it needs an explicit validity contract, an explanation of why localization is
degrading, and a recovery policy that does not confuse a new map with successful
relocalization.

## What I built

I designed and implemented a ROS 2 Jazzy framework around stereo-inertial
ORB-SLAM3 with five separable responsibilities:

1. A sensor guard checks stereo synchronization, timestamps, rates, staleness,
   and IMU gaps.
2. A version-pinned adapter converts ORB-SLAM3 diagnostics, poses, map identity,
   and recovery commands into a backend-independent ROS contract.
3. A health monitor fuses sensor and tracker evidence into an explainable state
   machine and a downstream `pose_valid` signal.
4. A recovery manager requests same-map relocalization first and escalates to an
   active-map reset only after a timeout.
5. Deterministic fault injection and EuRoC playback make the behavior repeatable.

The implementation combines C++17 ROS nodes for the runtime path with Python
for dataset playback, fault injection, orchestration, and offline evaluation.

## The important failure I found

My first blackout experiment looked like a relocalization test, but it was not.
Dropping camera messages while IMU continued caused a discontinuity when images
resumed. Stereo-inertial ORB-SLAM3 correctly treated that as a timestamp jump,
abandoned map 0, and initialized map 1.

That observation changed both the experiment and the product contract:

- `camera_blackout` now validates truthful reset/reinitialization behavior;
- `visual_occlusion` publishes black images while preserving every camera
  timestamp and IMU sample, which isolates genuine same-map recovery;
- map-ID changes are surfaced as `RESETTING`, never as `RECOVERED`;
- the benchmark rejects a blackout run that reports successful relocalization.

This distinction is the central engineering result: recovery is not just
"tracking became good again." The system must also prove whether the global map
identity was preserved.

## Verified result

On EuRoC `MH_01_easy`, the final automated validation run produced:

```text
INITIALIZING → HEALTHY → RELOCALIZING → RECOVERED → HEALTHY
map IDs: [0]
```

`RECOVERED` followed approximately 395 ms after visibility returned, and
`HEALTHY` followed approximately 895 ms after visibility returned. A separate
camera-blackout run changed the public map generation from 0 to 1 and followed
`HEALTHY → LOST → RESETTING → INITIALIZING → HEALTHY` without a false
`RECOVERED` state.

These are single-sequence engineering validation results, not a claim of
dataset-wide statistical performance. The automated suite exists to repeat the
same acceptance criteria across additional sequences.

## Design choices I can defend

- **Ground truth is offline-only.** The online detector cannot leak reference
  trajectory information.
- **Hard failures override the score.** A smoothed score cannot hide backend
  loss, stale sensors, or a map change.
- **Unavailable diagnostics are `NaN`.** The monitor renormalizes remaining
  weights instead of inventing healthy values.
- **Pose validity is explicit.** Consumers do not infer safety from the mere
  arrival of odometry messages.
- **Recovery commands are truthful.** Command acceptance is not recovery
  success; later state and map evidence determine the result.
- **The backend boundary is version-pinned.** The adapter validates the reviewed
  ORB-SLAM3 commit and keeps the upstream GPL source external.

## Limitations and next steps

- Current real-backend evidence is centered on EuRoC `MH_01_easy`; the next
  evaluation should run the fixed configuration across easy, medium, and
  difficult sequences.
- The score thresholds are engineering settings, not calibrated probabilities.
- Physical-camera deployment requires real stereo/IMU calibration and a checked
  base-to-camera transform.
- The framework is not a certified safety system; `pose_valid` should be one
  input to a broader safety architecture.

## Skills demonstrated

ROS 2 Jazzy, C++17, Python, ORB-SLAM3 integration, stereo-inertial timing, QoS,
TF and frame semantics, state-machine design, fault injection, reproducible
benchmarking, unit/integration testing, Docker, CI, and technical documentation.
