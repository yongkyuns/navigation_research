# Map-only route latching: native EKF comparison

## Answer and scope

**Map interval updates can move the internal EKF position into a supplied route corridor. The current defaults do not make that behavior reliable for large initial errors or automatic road association. No GNSS-bias model was used.**

This is a comparison, not a production estimator change. `native.Library` loads the original unaugmented 18-state Rust runtime; the evaluator refuses a shared library exposing the bias extension. Sensor tuning, map widths, map factor, map-cell reuse, intersection policy and default publication are unchanged. Supplied-route corrections call the existing `RoadBank.condition()` and native moment injection. Automatic association runs the full current `RoadBank`.

## Experimental contract

Three explicit itineraries use the independent cached OSM graph and unmodified GNSS tracks from `urban-short-turn-loop-nominal-001`, `urban-short-turn-loop-nominal-002`, and `urban-medium-turn-loop-nominal-001`. They are supplied-route fixtures, not independently surveyed or labelled ground truth. Segment IDs and time labels are frozen in `cases.json`. The known-route arm receives those IDs, not reference positions, attitudes or mounting. The automatic arm never receives the itinerary.

Each route has one clean control and sixteen disturbed cases: starting cross-track coordinate -20, -10, +10 or +20 m; original native covariance or a separately declared 10 m cross-track uncertainty added equally to every arm; original GNSS continuing or its first ten seconds withheld equally. IMU is never withheld. There is no artificial GNSS position bias and no receiver-error estimator. The uncertainty diagnostic is not a result with unchanged initialization uncertainty.

The same original native runtime, sample chronology, initial mean and covariance are cloned into map-free, supplied-route and automatic-map arms. A separate control verifies fallback isolation bit-for-bit. Initial mean displacement is applied once before cloning. References are not loaded. The actual native EKF state is sampled; no displayed marker is snapped and no mixture mean is mistaken for an internal state.

All selected road half-widths are 6 m: initial 10 m and 20 m cross-track coordinates are 4 m and 14 m outside the corridor. The injected displacement is not necessarily exactly 10/20 m because the clean checkpoint already has a lateral offset.

## Measures

Inside means distance to the finite supplied-route corridor <=1 mm numerical tolerance. Convergence is the first entry followed by five sampled seconds inside. Time-weighted residence includes every output epoch, not only successful latches. Two pre/post-map samples at the same timestamp have zero intervening duration.

Automatic accepted latching additionally requires a selected native branch on the supplied itinerary. An accurate fallback position, a broad road-group ID or a mixture coordinate is not sufficient. A known-route factor rejected by the existing compatibility gate leaves its native state unchanged; the supplied-route test retains that state and records subsequent attempts instead of substituting another road.

## Local executed results

51 paired cases completed. The 14 new tests and 39 pre-existing native integration tests passed. Inputs remained unchanged and all map-free/control snapshots matched exactly. No negative eigenvalue was found in inspected known-route covariance; the minimum was approximately 1.49e-8. Repository CI independently rebuilds the original Rust library and repeats the tests and all 51 cases. CI completion status is separate from this local evidence and must be checked on the exact commit.

| Initial cross-track | Initial uncertainty | GNSS | Before: entry before 10 s | Supplied route: entry before 10 s | Accepted automatic route: entry before 10 s |
|---|---|---|---:|---:|---:|
| +/-10 m | Native | Original fixes | 6/6 | 6/6 | 6/6 |
| +/-20 m | Native | Original fixes | 5/6 | 5/6 | 5/6 |
| +/-10 m | Native | First 10 s withheld | 0/6 | 6/6 | 2/6 |
| +/-20 m | Native | First 10 s withheld | 0/6 | 0/6 | 0/6 |
| +/-10 m | +10 m sigma to all arms | Original fixes | 6/6 | 6/6 | 6/6 |
| +/-20 m | +10 m sigma to all arms | Original fixes | 6/6 | 6/6 | 6/6 |
| +/-10 m | +10 m sigma to all arms | First 10 s withheld | 0/6 | 6/6 | 6/6 |
| +/-20 m | +10 m sigma to all arms | First 10 s withheld | 0/6 | 6/6 | 6/6 |

An entry before ten seconds can have its five-second hold extend past GNSS resumption. The traces retain both entry and subsequent retention. Do not reinterpret every entry as five seconds of map-only retention before fixes return.

### What the map actually does

With native covariance and GNSS initially withheld, all six +/-10 m supplied-route cases entered in **4.82-7.54 s**, versus **10.35-10.55 s** without maps, after fixes returned. The six +/-20 m cases did not enter before GNSS returned: the original map compatibility gate rejected updates.

On short-loop-002, native lateral sigma was about **0.261 m**. An imposed 20 m cross-track mean is deliberately inconsistent with that very confident prior. At the first map update the mean stays at 20.00 m because the constraint is rejected. The 10 m case is accepted but moves initially only to about **9.87 m**, then gradually enters the +/-6 m corridor.

Giving every arm the same explicitly uncertain prior changes the outcome. All twelve withheld-GNSS supplied-route cases enter at the first map update and remain in the supplied corridor for the evaluated remainder. For the same short-loop 20 m case, the **internal EKF** cross-track position changes **20.00 -> 2.46 m**. The corresponding map-free filter stays outside until fixes resume. This demonstrates a real state correction, not a selected label or postprocessed marker. It does not justify indiscriminate covariance inflation in production.

Automatic accepted-route latching remains more difficult. With native covariance, only two of six +/-10 m outage cases obtain a five-second accepted correct-itinerary latch beginning before ten seconds; none of the six +/-20 m cases do. With the explicitly uncertain prior, all twelve start an accepted native correct-itinerary latch in **3.02-5.03 s**, but junction abstention may interrupt publication later.

With original GNSS continuing, the baseline generally returns to the corridor by itself. Map-only timing gains with native covariance are small. Those recoveries must not all be credited to the map.

## Interpretation

The correction mechanism is demonstrated; universal route latching is not. The remaining issue is recovery from an overconfident displaced state without rejecting the correct map hypothesis, while distinguishing road-association confidence from positional convergence. The GNSS-bias extension is not involved.

This is a controlled route-relative test, not natural urban-canyon or surveyed absolute accuracy. Initial states are intentionally perturbed. Map widths/alignment are unsurveyed. Supplied segment-time labels can be approximate around junctions. The original covariance/reset, branch-scoring and static map-offset approximations remain. The additional-uncertainty arm is explicitly a sensitivity diagnostic, not a hidden retuning of the primary experiment.

## Reproduce

Build the original library using `prepare_native.py`, not `prepare_bias_native.py`, with upstream revision `8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6`. From the repository root:

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD/routefusion_prototype:$PWD/integration/imu_gnss"
export ROUTEFUSION_NATIVE_LIBRARY="$PWD/.cache/native/target/release/libroutefusion_native_bridge.so"
export ROUTEFUSION_DATASETS="$PWD/.cache/upstream/web/datasets"
export ROUTE_LATCH_MAP_CACHE="$PWD/integration/imu_gnss/map_cache"
python -m pytest -q integration/imu_gnss/route_latch/test_route_latch.py --junitxml=route-latch-tests.xml
python integration/imu_gnss/route_latch/evaluate.py --library "$ROUTEFUSION_NATIVE_LIBRARY" --datasets "$ROUTEFUSION_DATASETS" --map-cache "$ROUTE_LATCH_MAP_CACHE" --output runs/route-latch
python integration/imu_gnss/route_latch/qualify.py --results runs/route-latch --tests route-latch-tests.xml
```

Every output directory must be new. The artifacts contain per-case native trajectories, per-factor acceptance/rejection diagnostics, time-weighted residence, source/configuration/map/library hashes, and the complete observed-outcome table. Successful CI means this comparison executed faithfully; it does not turn failed latching cases into successes.

Original map-only source revision: `4ff67b909fd2ac025f06c550eedbdc6ef21c52a5`.
Configuration SHA-256: `ccc39432759c9a891e6b8310b22e2fcce27b073273e86a8866ba2bc81d3a7806`.
Local original-native library SHA-256: `5ad7955f748154e3ae66ed6fd5f04c0519991aac3ca3298dbcd772d4cd93affd`.
OSM response SHA-256: `b47a3bf3e88fb37bce8f0cb3c96e0041b4eaf04b63008e276c005fcb0c8fec16`.

OSM data: OpenStreetMap contributors, ODbL 1.0. Raw recordings are reused from the pinned upstream repository, not copied here. No independent reference source or new field collection is claimed.
