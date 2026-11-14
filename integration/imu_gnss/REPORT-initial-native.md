# Native IMU/GNSS integration and recorded-data evaluation

## What was integrated

The actual `sensor_fusion` Rust crate from `yongkyuns/imu_gnss_fusion`, pinned to `8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6`, runs through a small C ABI. Its 18-error-state 3-D propagation, auto mount alignment, GNSS/NHC updates, gating, timing and health policies are retained. No reference attitude, mount, position or motion is fed into the estimator.

An isolated build adds only clone support, an f64 anchor accessor, and an error-moment injection hook using the existing nominal injection and covariance reset. The source repository is unchanged. Road branches clone the entire runtime and ingest original IMU/GNSS once per alternative. The original planar prototype and frozen evidence are unchanged.

## Evidence boundary

**This is a recorded-data replay and integration evaluation, not demonstrated urban-canyon accuracy.** Package references do not establish an independent surveyed reference device. Discrepancies are agreement, not ground-truth error. iOS recordings have no reference trajectory. No independent road labels exist, so latch fraction is not road-association accuracy. These recordings are not established here as the target M9/LSM6DSO hardware.

The package directory contains 36 datasets. 35 passed strict input checks and replayed: 3,587,141 IMU samples and 48,920 GNSS samples over 10.004 hours. 1 dataset was rejected rather than silently retimed. 0 non-finite outputs were found at the 71,761 inspected output epochs.

Initialization succeeded in 33 replays; the two short stationary smoke recordings did not initialize automatic vehicle alignment. Output was available at 72.08% of sampled epochs, including initialization time rather than selecting only successful tails.

### Per-recording baseline

| Dataset | Duration (s) | First initialized epoch (s) | Output availability | Reference discrepancy RMSE (m) |
|---|---:|---:|---:|---:|
| covered-parking-urban-drive-gnss-outage-001 | 2148.5 | 382.580 | 82.2% | 5.066 |
| ios-drive-20260521-121635 | 1869.4 | 274.821 | 85.3% | — |
| ios-drive-20260521-130835 | 246.0 | 167.043 | 32.2% | — |
| ios-drive-20260521-131420 | 1105.2 | 337.081 | 69.5% | — |
| ios-drive-20260521-164415 | 541.2 | 520.228 | 3.9% | — |
| ios-drive-20260521-180601 | 901.4 | 511.161 | 43.3% | — |
| ios-drive-20260526-174206 | 3955.6 | 831.634 | 79.0% | — |
| ios-drive-20260527-123747 | 2629.4 | 183.035 | 93.0% | — |
| ios-drive-20260528-183859 | 1095.7 | 112.837 | 89.7% | — |
| ios-drive-20260530-110409 | 530.0 | 68.350 | 87.1% | — |
| ios-drive-20260530-164847 | 2213.7 | 115.362 | 94.8% | — |
| ios-evening-mixed-road-drive-001 | 707.1 | 136.891 | 80.6% | — |
| ios-evening-mixed-road-drive-002 | 567.8 | 192.536 | 66.1% | — |
| ios-mixed-drive-reverse-parking-001 | 2585.7 | 138.388 | 94.6% | — |
| ios-mixed-road-drive-001 | 673.0 | 64.687 | 90.4% | — |
| ios-mixed-road-drive-002 | 906.6 | 192.057 | 78.8% | — |
| ios-mixed-road-loop-001 | 1678.5 | 83.243 | 95.0% | — |
| ios-stationary-smoke-001 | 3.4 | — | 0.0% | — |
| ios-stationary-smoke-002 | 25.3 | — | 0.0% | — |
| mixed-road-highway-low-signal-001 | 1290.5 | 1045.959 | 19.0% | 1.499 |
| mixed-road-long-drive-large-mount-001 | 1278.8 | 106.407 | 91.7% | 0.853 |
| mixed-road-long-drive-large-mount-002 | 2608.1 | 2478.752 | 5.0% | 0.418 |
| mixed-road-long-drive-nominal-001 | 1421.0 | 882.333 | 37.9% | 0.724 |
| parking-lot-circle-turns-nominal-001 | 95.7 | 20.090 | 79.1% | 1.064 |
| parking-lot-figure8-nominal-001 | 151.0 | 29.622 | 80.4% | 2.491 |
| parkinglot-maneuvers | 662.3 | 146.717 | 77.8% | 1.498 |
| urban-long-turn-loop-nominal-001 | 399.3 | 38.439 | 90.4% | 0.947 |
| urban-low-speed-low-signal-001 | 1256.0 | 509.045 | 59.5% | 0.362 |
| urban-medium-turn-loop-nominal-001 | 309.7 | 23.607 | 92.4% | 0.656 |
| urban-medium-turn-loop-nominal-002 | 245.3 | 41.699 | 83.0% | 1.249 |
| urban-mixed-turns-nominal-001 | 521.7 | 107.007 | 79.5% | 1.973 |
| urban-short-turn-loop-nominal-001 | 177.0 | 39.668 | 77.6% | 3.682 |
| urban-short-turn-loop-nominal-002 | 190.0 | 22.100 | 88.4% | 0.609 |
| urban-stop-go-large-mount-001 | 490.5 | 188.338 | 61.6% | 1.036 |
| urban-stop-go-tight-turns-nominal-001 | 532.6 | 40.192 | 92.5% | 0.747 |

Availability is sampled at approximately 2 Hz. The first initialized epoch uses original log time. Reference comparisons interpolate only within 0.2-second reference gaps. GNSS agreement uses time-aligned fixes, not stale held positions.

### Rejected input

`covered-parking-urban-drive-data-gap-001` contains 87,003 duplicate adjacent IMU timestamps, including differing measurements at the same timestamp. It needs a documented timestamp-repair or aggregation policy. This evaluator does not sort, discard, or invent sub-sample times to make it pass.

## Road-map experiment

An independent cached OpenStreetMap extract is compiled to canonical directed graph tiles using shared node connectivity and one-way tags. It is not generated from the reference track. The current map may differ from the recording date; default widths are unsurveyed. Turn restriction relations, conditional access, lane geometry and surveyed elevation are not implemented.

The bounded bank applies finite-width corridor moments in the full 18-state covariance and five persistent map-translation alternatives. No snapped coordinate is returned as a fresh GNSS observation. The published shadow output remains map-free; only the separately named experimental candidate is corrected.

| Recording | Latch fraction of usable/map-covered epochs | Map-free discrepancy RMSE (m) | Candidate discrepancy RMSE (m) |
|---|---:|---:|---:|
| urban-short-turn-loop-nominal-001 | 54.0% | 3.668 | 3.668 |
| urban-short-turn-loop-nominal-002 | 69.0% | 0.608 | 0.607 |
| urban-stop-go-tight-turns-nominal-001 | 72.4% | 0.744 | 0.744 |
| ios-mixed-road-drive-001 | 63.8% | — | — |

The clean reference-bearing runs show essentially no coordinate benefit. This null result is retained, not used to tune widths until plots improve. The iOS candidate can move by metres, but no truth stream establishes whether that helps.

At every sampled map epoch the map-free filter matched a separate native replay exactly: True. No negative horizontal candidate covariance eigenvalues occurred: True.

### Controlled perturbations

The second short-loop recording was replayed with either all GNSS fixes removed at seconds 90–120 or a 20 m north position bias ramped in/out over five seconds in that interval. IMU, velocity and stated GNSS uncertainty were unchanged. These are injected stresses, not natural receiver errors, independent trials or held-out hardware tests. The original reference remains evaluation-only.

| Stress (90–120 s) | Map-free reference discrepancy RMSE (m) | Candidate reference discrepancy RMSE (m) |
|---|---:|---:|
| outage30 | 2.927 | 2.450 |
| north_bias20 | 17.506 | 17.445 |

Geometry reduced discrepancy in this outage example, but barely helped the position-bias case. The pinned upstream EKF does not have the planar prototype’s augmented colored GNSS position/velocity bias states. Its original covariance and rejection behavior remain intact; persistent urban multipath is not solved by this integration.

## Computation and memory

A complete native clone occupies 14,880 bytes (`size_of::<Bridge>()`). At 24 alternatives plus fallback, native instances alone use 372,000 bytes, excluding graph tiles, Python state, histories, allocation and stack. The evaluation also runs an independent control. This is exact runtime reuse for research, not a RAM-optimized firmware design.

Baseline replay loops took 68.01 s on this host, excluding decompression/audit. Four map runs took 7.96–41.27 s each. One-Hz map-step p95 ranged from 27.28 to 32.36 ms. These are desktop measurements, not MCU latency, stack, RAM or power qualification.

## Remaining limits

Inference is approximate: sequential Gaussian interval moments, locally linearized polylines, bounded hypotheses, finite static map-offset prior, and broad-mixture branch scoring. The score is not the exact marginal likelihood of upstream’s gated GNSS/NHC update. No calibrated free-motion mode is implemented. Map histories grow with travel and lookup is not MCU optimized.

Peak discarded conditional branch mass was 55.3%. The policy abstains at that epoch, but does not bound cumulative loss of a correct alternative. A 97% retained-group weight is not a calibrated 97% probability of the correct road. OSM-way groups do not certify a unique directed segment or lane.

The integration inherits the pinned runtime’s reset convention, including its attitude-only reset implementation; it does not independently requalify every upstream covariance approximation. Automatic alignment delays, persistent GNSS error modeling, branch recovery, independently labelled routes, surveyed reference data, map validity and MCU qualification remain explicit work.

## Recommendation

Keep this integration in shadow/output-only mode. Prioritize a calibrated or warm-start mount path for a permanently installed device, then evaluate persistent GNSS error modeling and safe hypothesis recovery on independently labelled urban-canyon recordings. Do not promote geometric feedback based on the clean-run null result or one injected-outage improvement.

See `README.md` for reproduction, `evidence/dataset-audit.json` for input findings, per-run CSV/JSON for every result, and `map_cache/provenance.json` for the exact OSM snapshot. Road data © OpenStreetMap contributors, ODbL 1.0; map terms are separate from code licensing.
