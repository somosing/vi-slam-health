# Third-party SLAM backends

No backend is fetched during the default build. The `vif_orbslam3` package is
present, but its external executable is built only when
`VIF_BUILD_ORB_SLAM3_ADAPTER=ON` and `ORB_SLAM3_ROOT` identifies a complete
pinned build tree.

For an ORB-SLAM3 integration, record an exact upstream commit and its complete dependency versions, keep any instrumentation as a small patch series, and do not add vocabulary files or dataset content to Git. See `docs/orbslam3_integration.md`.

The reviewed patch for commit `4452a3c4...` is stored under
`third_party/patches/orb_slam3/`. Candidate upgrades must be reviewed for
license, Jazzy compatibility, supported sensor mode, coordinate conversion,
QoS, and whether they preserve every observability signal required by
`vif_interfaces/TrackingStatus`.
