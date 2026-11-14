# Recorded IMU/GNSS replay experiment

This experiment borrows the public generic replay packages from `yongkyuns/imu_gnss_fusion` at **8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6**. It leaves that repository and the frozen RouteFusion synthetic snapshot unchanged.

## What this actually tests

The original RouteFusion `PlanarFilter` and `RouteTracker` consume recorded IMU/PVT through a new reduced planar adapter. An independent OpenStreetMap graph supplies road geometry, identity and connectivity. Each navigation branch sees the original adapted inertial increments and PVT, not a snapped coordinate or another branch's posterior.

This is **not** a full integration with the source project's Rust 3-D ESKF, not an M9/LSM6DSO hardware qualification, and not a surveyed accuracy benchmark. The iOS recordings contain no independent reference track. The short-loop package includes `reference_position.csv`, documented upstream as a **fused reference trajectory**, so its comparison is agreement, not ground-truth accuracy. Neither reference position, attitude, mount nor motion enters calibration, estimator inputs, candidate selection, or road construction.

## Fixed experiment configuration

Four datasets are selected before replay: `urban-short-turn-loop-nominal-001`, `ios-mixed-drive-reverse-parking-001`, `ios-drive-20260521-130835`, and `ios-drive-20260521-131420`. File hashes and row counts are verified against each pinned manifest. The first is a curated generic source trace with unspecified hardware; the other three are iOS recordings. The source repository's generic format does not by itself establish the sensor models or independent GNSS-only provenance.

The first `min(90 seconds, 25% of duration)` is calibration only. Stationary samples establish an up direction and gyro offset; speed/course changes and corresponding IMU interval means fit a proper horizontal mounting rotation. The evaluated suffix begins at the first later GNSS sample with horizontal speed at least 2 m/s. That seed is used for initialization once, not reapplied as an independent update. Insufficient stationarity or excitation causes an explicit failure rather than an invented mount.

After calibration, an **IMU-only** complementary tilt adapter rotates gravity out of horizontal acceleration. It propagates the body-frame up vector with the gyro and applies a slow 60-second accelerometer correction near the calibrated force norm. This approximation can confuse sustained acceleration with tilt and assumes the physical mounting does not change. It is a research adapter, not a replacement for the existing full INS's attitude/mount/covariance handling. No suffix GNSS enters this preprocessing, including in withheld-GNSS tests.

Inputs are consumed causally using zero-order hold, packed into at most 50 ms increments while preserving time, acceleration and angular-rate integrals. Epochs are aligned to recorded measurement time, not file order/arrival time. IMU gaps over 0.2 seconds fail explicitly. NED velocity/covariance axes become local EN; units remain SI. Heading derived from velocity is not added as another observation. Missing PVT cross-covariance/inter-epoch correlation is not solved by the adapter; covariance floors (3 m position, 0.5 m/s velocity) and the existing colored error states are approximations.

Fixed recorded-data process settings are acceleration density 0.2 m/s²/√Hz, gyro density 0.3 degrees/s/√Hz, acceleration bias random walk 0.01, gyro bias random walk 0.01 degrees/s, initial map-offset sigma 5 m, and boundary sigma 1.5 m. These are declared engineering settings, not identified hardware specifications or held-out performance-optimal tuning. Original synthetic tunings and evidence are untouched.

## Independent road maps

Overpass supplies OSM motor-road ways and restriction relations for a padded trip bounding box. The full trip chooses **cache coverage only**; geometry is never constructed from recorded GNSS or references. This is an offline, preloaded-cache experiment, not a measured bandwidth/prefetch algorithm.

Canonical OSM node identity supplies connectivity. Ways are split at shared graph nodes; one-way travel and simple via-node turn restrictions are preserved. Geometric crossings do not create connections. Immediate reverse edges are excluded. Via-way and conditional restrictions, reverse-driving modes, lane boundaries, grade-separated height measurements and a complete production routing compiler are not implemented. Each directed edge has its own confidence group; a street name does not merge distant segments.

Width tags are used when numeric, otherwise conservative full-width priors of 12 m (ordinary road) or 8 m (service road). These are not surveyed road boundaries. Current OSM geometry may differ from the recording date. The published query, response checksum, OSM database timestamp, compressed response, generated tiles, and attribution make the map input reproducible.

Map data and derived graph tiles: **© OpenStreetMap contributors**, available under **ODbL 1.0**. https://www.openstreetmap.org/copyright. This data license does not replace or assign a software license to the surrounding repository.

## Runs and interpretation

Every dataset gets an unmodified-input replay. The two shorter datasets also get a **30-second injected GNSS withholding** window and a **30-second injected map-cache loss** window, starting 30 seconds after initialization. Missing-map epochs must return exactly the map-free state/covariance. Withheld observations are available to the scorer only; they are not sensor truth and are not fed to the navigation filter in that window.

Reports measure finite/PSD state behavior, association availability/ambiguity, branch pruning, map-induced displacement and output discontinuities, receiver agreement, and fused-reference agreement when present. They do **not** label distance to a road as accuracy, count a high branch weight as correct-road probability, or claim a wrong-road rate without independent road labels. A poor result remains in the report.

## Reproduce

From the repository root after installing the RouteFusion package and NumPy/SciPy/pytest/matplotlib:

```sh
python -m pytest -q experiments/recorded/test_recorded.py
python experiments/recorded/run.py --fetch \
  --input runs/recorded-inputs --output runs/my-recorded-replay
```

To reuse the exact published OSM responses, copy each dataset's `results/<dataset>/map/` directory into the corresponding output dataset directory before running. Otherwise a new query may return changed map geometry. Source sensor packages are downloaded at the pinned commit and verified; they are not redundantly copied into this repository.

The initial inventory and subsequent executions live in Git history. `results/summary.json` records the executed code hashes, input hashes, calibration diagnostics, map source, failures, and every run. `results/REPORT.md` is the compact human-readable output. Per-epoch CSVs and plots are diagnostics; no reference trajectory is used as a navigation input.
