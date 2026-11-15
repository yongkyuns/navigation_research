# Map-only position recovery

## Scope

This opt-in extension addresses the failure isolated by PR #5: a displaced EKF position with very small covariance rejects a correct road corridor. It does not add GNSS-bias states, change receiver covariance, narrow road widths, or snap a displayed coordinate. The original 18-state Rust `SensorFusion` runtime and the default `RoadBank` remain unchanged.

The corrected quantity is a candidate's internal native EKF state. The original map-free state is maintained independently. The research output contract remains shadow/map-free by default; candidate position and route association are not independently certified physical truth.

## Explicit recovery model

`TrustedRouteConditioner` supports a caller-supplied trusted road identity. `RelocalizingRoadBank` instead evaluates nearby directed roads without receiving the supplied evaluation itinerary. Both use the existing finite-width interval update for the actual correction.

A local Gaussian with a tiny covariance can be confidently wrong. Merely raising the map compatibility threshold would not make its Kalman gain large enough to recover. The new supervisor starts a separate localization-loss hypothesis, rather than quietly changing the existing Gaussian or rejecting every distant road forever.

Activation requires three consecutive map-inconsistency observations spanning at least 1.8 seconds, at least five metres of displacement, speed above 2 m/s, and horizontal position sigma at most 2 m. The same candidate road must persist across the observations; gaps above 1.5 seconds restart confirmation. The original interval log-probability threshold remains -12. A plausible competing road prevents automatic loss inference. Candidates must lie on finite segments, agree with velocity direction (cosine at least 0.5), and be within 30 m of a corridor. Missing geometry is not negative evidence.

After confirmation, a fresh clone of the already-updated map-free runtime is given an independent horizontal position-translation prior:

```
mean_new = mean_old
P_new = P_old + E (sigma_recovery^2 I_2) E^T
sigma_recovery = 10 m
```

`E` maps map-frame horizontal translations into the native position error block. The added matrix is positive semidefinite. Existing attitude, velocity, sensor biases, mounting uncertainty, and cross-covariance are retained. No new state dimensions are introduced. The mean is then corrected by the original interval conditioning and native error-state injection/reset, not by the prior expansion itself.

**This deliberately increases candidate position uncertainty.** It is an explicit recovery-mode assumption, not unchanged covariance tuning or a rigorously calibrated fault probability. The uncorrected control retains its original covariance throughout. The three-way comparison starts from identical original means/covariances; the recovery intervention occurs only after confirmation.

The old map-conditioned candidate information is dropped when rebuilding from the map-free checkpoint. All original samples through that epoch are already represented in the clone and are not fused again. Subsequent original samples update each alternative once. At most one expansion occurs per recovery adapter instance; there is no repeated inflation at every GNSS/map epoch. Map replacement invalidates the recovery episode.

The automatic adapter retains multiple roads and the existing map-offset alternatives, branch budget, and junction phases. It disables the separate rollback mechanism in this experiment so two recovery policies cannot repeatedly reset uncertainty. Existing branch pruning and confidence scores remain approximate. Full automatic recovery on a coherent wrong parallel road is not established.

## Controlled native evaluation

The policy was recorded before the experiment and was not adjusted after examining the results. It reuses the three explicitly supplied connected OSM itineraries and recorded inputs from PR #5. These are development fixtures, not new held-out field data or surveyed truth.

The final local run executes 27 paired cases: three clean controls and 24 disturbed cases (three routes, four signed initial offsets, with original GNSS continuing or the first ten seconds withheld equally). Every pair runs the old and new implementation afresh. All use the original native initial covariance; the additional-uncertainty diagnostic from PR #5 is not used here.

Cross-track targets are +/-10 or +/-20 m from the road centerline, whose half-width is 6 m. Thus the imposed positions are 4 or 14 m outside the corridor. A supplied-route success is actual native-state entry followed by five sampled seconds inside, not a road label. Automatic success additionally requires a selected native hypothesis on the supplied itinerary; a good fallback coordinate does not count.

| Initial cross-track | Cases | Old supplied route: entry before GNSS return | New supplied route | Old automatic accepted route | New automatic accepted route |
|---|---:|---:|---:|---:|---:|
| +/-10 m | 6 | 6/6 | 6/6 | 2/6 | 2/6 |
| +/-20 m | 6 | 0/6 | 6/6 | 0/6 | 5/6 |

The six larger-offset supplied-route cases enter at about **2.01 s** and remain within the supplied corridor through the rest of the evaluated window, including subsequent GNSS updates. In all six, the five-second hold completes before fixes resume. The original map-only estimator enters only after GNSS returns, at approximately 10.05–10.55 s.

Representative short-loop-002 case: at recovery activation, actual internal EKF cross-track position changes **20.187 -> 2.482 m**, about 17.705 m of position correction. Before activation, all arms have the original identical initialization and the fallback remains outside. No reference coordinates or receiver-error model produce this change.

The five successful automatic cases start their accepted five-second interval at about **5.03 s**. Their five-second hold extends slightly past GNSS resumption at 10 s; do not report this as five full seconds of pre-GNSS automatic retention. The sixth case briefly accepts the supplied road but junction ambiguity interrupts the continuous hold, so it remains a failure under the declared metric. The +/-10 m automatic limitation is not fixed by this targeted large-inconsistency recovery.

## Regression and negative checks

Local validation passed **70 Python tests**: 39 existing integration/intersection checks, 14 previous route-latching checks, and 17 recovery checks. Four full unmodified mapped recordings retain exactly the same sampled baseline/candidate positions, covariances, directed-edge decisions and latch states at **1,569 epochs**. All paired controls remain bit-identical and inputs are unchanged. No negative known-route covariance eigenvalue occurred at the inspected epochs.

The original native library's provenance was checked against all 37 recorded source hashes. A fresh Rust build and independent replay are performed by `.github/workflows/qualify-route-recovery.yml`; check that workflow for exact-head CI status rather than treating the local run as a runner execution.

Negative tests include stationary observations, out-of-range roads, incompatible travel direction, finite-segment bounds, missing maps, changed map versions, repeated activation prevention, and equal parallel-road alternatives. In the symmetric parallel-road test both hypotheses survive and no road is published. A plausible alternate road prevents the automatic loss declaration.

## Remaining limits

A supplied but incorrect parallel road can still pull a trusted-route candidate onto that incorrect corridor. This cannot be detected merely from that trusted identity and indistinguishable relative motion. A coherent wrong road already supporting the map-free position can prevent automatic recovery. The data does not establish lane accuracy, true-road probability, natural urban-canyon performance, or behavior under sustained receiver faults.

The policy is intentionally local and single-attempt. It is not a global relocalizer, always-on covariance floor, route-command enforcement, GNSS-fault detector, or full Bayesian change-point model. Its distance, uncertainty and time thresholds are explicit engineering assumptions, not measured sensor distributions. The native reset and unsurveyed-map limitations from `INTERSECTIONS.md` remain.

## Reproduce

Build with `prepare_native.py` and upstream revision `8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6`, never `prepare_bias_native.py`:

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD/routefusion_prototype:$PWD/integration/imu_gnss"
export ROUTEFUSION_NATIVE_LIBRARY="$PWD/.cache/native/target/release/libroutefusion_native_bridge.so"
export ROUTEFUSION_DATASETS="$PWD/.cache/upstream/web/datasets"
export ROUTE_LATCH_MAP_CACHE="$PWD/integration/imu_gnss/map_cache"
mkdir -p runs/recovery-tests
python -m pytest -q integration/imu_gnss/tests integration/imu_gnss/route_latch/test_route_latch.py integration/imu_gnss/route_recovery_tests --junitxml=runs/recovery-tests/tests.xml
python integration/imu_gnss/evaluate_relocalization.py --library "$ROUTEFUSION_NATIVE_LIBRARY" --datasets "$ROUTEFUSION_DATASETS" --map-cache "$ROUTE_LATCH_MAP_CACHE" --output runs/recovery-comparison --tests runs/recovery-tests/tests.xml
```

Output directories must be new. Results preserve both versions, every native trajectory, per-factor acceptance diagnostics, activation times, policy/source/library/map hashes, clean comparisons, and failure denominators. Successful CI qualifies execution integrity; it does not convert every automatic association into success.

Map data: OpenStreetMap contributors, ODbL 1.0. The local native binary hash is `5ad7955f748154e3ae66ed6fd5f04c0519991aac3ca3298dbcd772d4cd93affd`; the exact CI rebuild can have a different binary hash. Policy-record SHA-256 is `f720d2529948fa4460084d614a4d8a5ac568586cf14283b2a6e0495d1b8a0d2e`.
