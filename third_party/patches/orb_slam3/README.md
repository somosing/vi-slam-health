# Pinned ORB-SLAM3 patch

Target revision:

```text
4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
```

The patch exposes a read-only `TrackingDiagnostics` snapshot through
`ORB_SLAM3::System`. It does not alter tracking, optimization, map creation, or
relocalization behavior.

This patch was generated and apply-checked against the exact target revision.
Local changes limited to the top-level `CMakeLists.txt` and `build.sh` do not
change its source-file contexts.

Apply it from the ORB-SLAM3 repository:

```bash
git rev-parse HEAD
git apply --check \
  /absolute/path/to/vi-slam-health/third_party/patches/orb_slam3/0001-expose-tracking-diagnostics.patch
git apply \
  /absolute/path/to/vi-slam-health/third_party/patches/orb_slam3/0001-expose-tracking-diagnostics.patch
./build.sh
```

The revision check must print the target hash. If `git apply --check` fails,
do not force the patch onto a different source version. Build the adapter with
`VIF_ORB_SLAM3_HAS_DIAGNOSTICS=OFF` until the patch has been reviewed for that
version; the health monitor will exclude the unavailable metrics.
