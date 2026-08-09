# vif-eval

Pure Python offline evaluation. Ground truth belongs here, never in the online health monitor.

Trajectory inputs use the TUM format:

```text
timestamp tx ty tz qx qy qz qw
```

Failure labels use:

```csv
start_sec,end_sec,label
8.0,11.0,camera_blackout
```

Recovery events use:

```csv
timestamp_sec,event,attempt_id
8.4,REQUESTED,1
9.1,STARTED,1
12.2,SUCCEEDED,1
```

The ROS message logger is intentionally separate from metric computation so experiment artifacts remain inspectable and metrics are reproducible.
