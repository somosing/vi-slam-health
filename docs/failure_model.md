# Failure model

Failures are defined by observable consequences, not only injected causes.

| Category | Examples | Online evidence |
|---|---|---|
| Visual input | blackout, occlusion, blur, underexposure, frame loss | camera age/rate, feature count, inlier ratio |
| Inertial input | dropout, gap, saturation, bias step | IMU age/gap, range checks, motion disagreement |
| Timing | jitter, reordering, camera/IMU offset | monotonicity, stereo skew, synchronization residual |
| Geometry | low texture, repeated patterns, dynamic objects | low inliers, high residual, unstable pose increments |
| Compute | CPU contention, processing overrun | processing time, queue age, real-time factor |
| Recovery | false place match, map jump, reset loop | discontinuity, repeated attempts, post-recovery error offline |

The supplied injector covers transport-level faults. Photometric and inertial-value corruption should be added with clearly versioned parameters and deterministic seeds before making detector claims about them.

Labels describe the injected interval and the expected observable failure. Some faults may not cause tracking loss; those cases must remain in evaluation rather than being silently removed.
