# Evaluation protocol

## Rules

1. Ground truth is offline-only.
2. Tune weights and thresholds on development data, choose them on validation data, then freeze them.
3. Report clean and fault-injected results separately.
4. Preserve the exact configuration, repository commit, dataset checksum, host metadata, and random seed.
5. A successful recovery requires stable online health plus acceptable post-recovery offline pose error.

## Trajectory metrics

- Absolute trajectory error (ATE), SE(3)-aligned for stereo-inertial estimates
- Translational relative pose error (RPE) at a declared time delta
- Trajectory completeness
- Post-relocalization pose error in a fixed window
- Runtime, real-time factor, CPU, and peak memory

Do not enable scale alignment for metric stereo-inertial trajectories unless the experiment explicitly studies scale drift.

## Detection metrics

- Precision, recall, F1
- False alarms per minute
- Detection delay from labelled fault onset
- Fraction of failures for which invalid pose publication stopped within the deadline

## Recovery metrics

- Relocalization success rate
- Time from loss detection to stable recovery
- Reset/new-map rate
- Map-frame discontinuity
- Error during the first 1, 3, and 5 seconds after recovery

Every aggregate table should include the number of sequences and recovery attempts, not only percentages.
