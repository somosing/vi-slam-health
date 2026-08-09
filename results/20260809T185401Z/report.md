# VIF ORB-SLAM3 benchmark report

- Run ID: `20260809T185401Z`
- Generated (UTC): `2026-08-09T18:56:00.181250+00:00`
- Overall result: **PASS**
- Dataset: `EuRoC MH_01_easy` (local path omitted)

| Scenario | Result | Detection delay (s) | Same-map recovery after fault (s) | Healthy after fault (s) | Map IDs |
|---|---:|---:|---:|---:|---|
| clean | PASS | — | — | — | 0 |
| visual_occlusion | PASS | — | 0.395 | 0.895 | 0 |
| camera_blackout | PASS | 0.502 | — | 0.268 | 0, 1 |

Detection delay is measured from the published fault-start event to the first failure-state transition. Same-map recovery is reported only for visual occlusion; a camera blackout is expected to reset and reinitialize instead.

### clean — PASS

- State sequence: `INITIALIZING → HEALTHY`
- Map IDs: `[0]`
- Acceptance errors: None
### visual_occlusion — PASS

- State sequence: `INITIALIZING → HEALTHY → RELOCALIZING → RECOVERED → HEALTHY`
- Map IDs: `[0]`
- Acceptance errors: None
### camera_blackout — PASS

- State sequence: `INITIALIZING → HEALTHY → LOST → RESETTING → INITIALIZING → HEALTHY`
- Map IDs: `[0, 1]`
- Acceptance errors: None
