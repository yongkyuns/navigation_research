# Executed validation and limits

## Evidence

The final configuration ran 24 automated tests and six synthetic scenarios with ten held-out random seeds each (100–109), 120 seconds per run, totaling 7,200 one-second map epochs. Development used other seeds and exposed failures; the final velocity-bias model was added before this held-out evaluation. This is not a blind external benchmark or a real sensor trial. The replay adapter reproduced all 120 navigation outputs in the supplied parallel-bias example.

Artifacts: `results/tests.txt`, `results/environment.json`, `results/metrics.json`, `results/summary.json`, per-run CSVs, and input JSONL/tile examples. The source and evaluation configuration are included for independent reruns.

## Scenario assumptions

The reduced inertial input runs at 5 Hz and PVT/map epochs at 1 Hz. This is a synthetic computation convenience, not a recommended physical IMU sampling rate. The simulated vehicle travels at 8 m/s. Most scenarios begin with a known-road/open-sky-type initialization; the symmetric parallel-road case intentionally starts ambiguous. The tracker receives no truth labels.

Parallel roads are 26 m apart. The biased case ramps the GNSS position bias to [4,26] m and adds 0.35 m/s cross-track velocity bias; it also includes white PVT noise and occasional large outliers. The turn case has a curved turn and GNSS outage during seconds 25–70. The negative-map case moves the map 6 m north while GNSS is otherwise good. The off-road case leaves the road after 40 seconds. The cache case removes all map coverage during epochs 45–70. Noise/offset choices are invented stress-test parameters, not measured M9 or LSM6DSO performance.

## Results

Numbers below are **means of per-run horizontal RMSE**, not pooled RMS and not hardware accuracy claims. Baseline is the same augmented robust navigation model with map updates disabled. Proposed output uses a selected branch only while latched, otherwise the baseline. Availability denominator includes only mapped, map-available epochs; off-road and unavailable epochs are analyzed separately.

| Scenario | Map-free RMSE (m) | Proposed-output RMSE (m) | Route-latch availability |
|---|---:|---:|---:|
| Persistent GNSS bias / nearby road | 17.70 | 5.82 | 95.3% |
| Turn and GNSS outage | 8.86 | 2.30 | 89.5% |
| Symmetric parallel roads | 13.05 | 13.05 | 0.0% |
| Map shifted 6 m | 2.12 | 4.01 | 95.4% |
| Departure from mapped road | 3.84 | 3.75 | 95.0% |
| Cache unavailable | 2.12 | 1.99 | 44.9% |

### Positive cases

The persistent-bias case reduced mean RMSE from 17.70 to 5.82 m. Mean per-run 95th-percentile horizontal error decreased from 25.86 to 6.66 m. The true route remained available through motion/topology rather than a nearest-point choice. In the turn/outage case RMSE decreased from 8.86 to 2.30 m, with a 4.11 m mean per-run 95th-percentile output error versus 17.18 m baseline.

These show that the implementation can exploit the intended information under its simulation assumptions. They do not establish general urban performance, a specific outage duration for LSM6DSO, or superiority to a tuned deployed production filter.

### Negative and abstention cases

**Map error:** A 6 m map shift increased position RMSE from 2.12 to 4.01 m while the road association remained correct. Persistent map-offset uncertainty prevented pretending the map was exact, but did not solve the absolute-source ambiguity. A confidently correct road ID is not evidence of accurately corrected coordinates.

**Parallel ambiguity:** Symmetric observations produced no route latch. The 13.05 m position error remained; that is preferable to inventing a certain road. A raw nearest-road tie happened to choose the ground-truth road in this artificial case, which is not valid evidence that nearest-road matching solved it.

**Off-road detection:** The old road latch persisted for 3 seconds after departure in every test run, i.e. 3.75% of the 80 off-road seconds, before abstention/fallback. Therefore the correct statement is no wrong-road latches on the sampled *mapped-road portions*, not no false latches overall. The detector needs independent field qualification.

**Missing cache:** The system fell back rather than treating absence of geometry as evidence. Latch availability over map-available epochs was only 44.9%; after restoration it was often cautious/ambiguous. A short fixed-lag recovery buffer could help, but is not implemented.

### Confidence and computation

The evaluated position covariances covered truth inside their nominal 95% ellipses in 100% of these synthetic samples. This indicates conservatism under the chosen model, not calibrated 95% coverage. All seeds/scenarios share related assumptions and provide no certified failure-probability bound.

At most eight branches were observed, with a configured cap of twelve. Some intersection epochs pruned about 20% of conditional branch mass. The latch policy suppresses commitment on a high-pruning epoch, but does not account for cumulative lost alternatives. No target-MCU timing, stack, RAM or energy test was performed. Python desktop runtimes in the metrics are execution evidence only.

## Tests included

The 24 tests cover scalar interval moments against an independent numerical reference, Monte Carlo conditional moments, lack of artificial along-road information, persistent map-offset covariance, mixture between-mode variance, covariance positive semidefiniteness, duplicate PVT/epoch rejection, graph connectivity, canonical tile IDs and round trips, missing coverage, symmetric ambiguity, turning with GNSS outage, off-road abstention and local coordinate conversion.

Tests validate selected invariants, not all finite-precision behavior. The model is planar, and candidate gating/beam pruning are approximate. Extremely long logs, road reversals, aggressive driving, geometry-version replacement and dense multi-level urban networks need additional cases.

## Before deployment

Validate actual M9 epoch alignment/covariance and installed LSM6DSO bias/temperature behavior. Compare current production filter, robust map-free variant, output-only HMM, single-road feedback and proposed bank with identical data. Keep separate measures for route identity, position/velocity error, abstention, false-latch duration, map-error sensitivity, coverage gaps and route reacquisition. Test cold starts inside ambiguous canyons, not only entry with a good prior.

Use held-out roads/days/hardware units, an independently checked reference trajectory and map-alignment labels. Public UrbanNav data is a possible additional replay corpus, but it was not executed here and does not replace the user's hardware validation. Check release-specific usage terms. Surveyed and high-grade reference data also need uncertainty bounds in urban obstructions.
