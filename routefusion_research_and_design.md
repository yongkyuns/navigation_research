# Tiled-road-aided loosely coupled INS/GNSS: research and design

## 1. Decision and scope

Use a bounded **Gaussian-sum road-hypothesis filter bank** around the existing loosely coupled navigation filter. Each branch represents an alternative road/path and continuous navigation state. The original IMU and GNSS PVT observations update every alternative once. A map-free branch remains available at all times. Road geometry supplies conditional constraints, not a second absolute position measurement computed by snapping the first estimate.

This retains loose GNSS coupling: no per-satellite pseudorange solver, RAWX, NTRIP stream, or new receiver is required. It targets a road vehicle with a fixed IMU mounting and a usable road graph. It is not a lane-level guarantee, and an intended route is a soft prior rather than proof that a driver followed it.

The delivered prototype verifies the core mathematics and failure behavior using synthetic planar data. A full production integration requires the existing filter's prediction/update interfaces, real recordings, map-error calibration, and MCU profiling. It is intentionally described as approximate inference, not an exact Rao–Blackwellized particle filter or certified integrity monitor.

## 2. What the literature contributes

**Sequence matching instead of isolated snapping.** Newson and Krumm's HMM map matcher combines noisy position observations with network structure [S1]. Valhalla Meili documents a practical HMM implementation [S9]. These are useful output-only baselines; their road posterior is not automatically an independent sensor that can be fed back into the original navigation filter.

**Discrete routes with conditional continuous estimates.** Heirich combines loosely coupled GNSS, IMU, and a rail map using a Rao–Blackwellized particle filter [S2]. The transferable lesson is to retain discrete alternatives while estimating continuous variables within each alternative. A train's exact track constraint is stronger than a vehicle's road corridor; the paper's accuracy statistics must not be transferred to cars. The proposed bounded Gaussian bank borrows the decomposition, not the paper's full estimator.

**Map constraints are compatible with loose coupling.** Falco et al. study constrained GPS/INS integration with road-boundary information [S3]. The proposal adopts the geometric idea, but not old measurement-reuse or noise assumptions without scrutiny.

**Inequalities instead of fictitious centerline observations.** Density-function truncation provides a principled way to condition a Gaussian on a feasible interval [S4]. A road means the antenna may lie within an uncertain corridor, not that it is exactly at the centerline.

**Feedback needs validation.** Wang et al.'s 2025 work concerns low-quality sensors and map-matching feedback in urban canyons [S5]. Only its publisher abstract/highlights and institutional record were inspected here. They support examining feedback validity and motion-dependent ambiguity, not importing their reported gains or claiming their complete method was reproduced.

**Tiled topology is established infrastructure.** Valhalla's Baldr and Overture's explicit segments/connectors provide useful map-organization references [S10,S11]. The estimator needs a small directed graph, not a rendering stack or a full routing service on the device.

## 3. Why ordinary feedback snapping is unsound

Let a navigation filter produce q(x | z), and let a map matcher derive a snapped coordinate s = F(q,M). Feeding s into the same filter as a fresh position measurement uses information derived from z twice. A small arbitrarily assigned covariance makes this particularly dangerous: the wrong road can become self-confirming, after which genuine GNSS evidence is rejected as an outlier.

The same problem occurs when a trajectory matcher treats consecutive fused positions as independent samples, or separately multiplies GNSS velocity and course likelihoods even though the course was derived from that velocity. Increasing a heuristic covariance can reduce the damage but does not establish the missing joint covariance.

The clean interface is either (a) output-only matching, with no navigation feedback; or (b) conditional state branches with the original observations and map factors inside the estimator. The prototype implements (b). Alternative branch posteriors are never fused as independent estimates. This solves that specific double-use problem; temporal correlation in the receiver outputs still needs its own model.

## 4. Architecture

```text
SERVER / OFFLINE                                    DEVICE

OSM / Overture / licensed road graph                 LSM6DSO samples + timestamps
  -> resolve directed connectivity                         |
  -> widths, quality, local coordinate frames              v
  -> canonical IDs + map versions                    existing INS propagation
  -> compact graph tiles                                  |
              |                                            +------------------+
              | on demand / cache refill                   |                  |
              v                                            v                  v
        versioned tile cache                         map-free filter    route branch bank
              |                                      original PVT      original PVT once
              +--> candidates + legal transitions -------------------> per alternative
              |                                                         |
              +--> corridor / geometry / map error -------------------->|
                                                                        v
M9 NAV-PVT + NAV-COV -> epoch/covariance adapter --------> evidence + pruning
                                                                        |
                                             ambiguity / map-validity supervisor
                                                       /                \
                                            fallback solution      selected route branch
                                                       \                /
                                                        output contract
                                            position, covariance, road ID or unknown,
                                            conditional weights, map status and age
```

No branch receives a position exported by another branch as a new measurement. Tiling is a storage/delivery concern and must not cause filter resets at tile boundaries. The map-free filter is a fallback, not a calibrated Bayesian off-road mode in this implementation.

## 5. Probabilistic model

Let x_k contain continuous navigation variables, r_k a directed road/path hypothesis, u_k the inertial increment, z_k the original receiver PVT, and M the road map. Add b^p_k and b^v_k for slowly varying GNSS position and velocity errors, and m for a persistent local map translation error.

A conceptual path posterior is

    p(X,R,b,m | U,Z,M) proportional to
      p(initial state, route, biases, map error)
      product_k [f(x_k | x_(k-1),u_k)
                 g(z_k | x_k,b_k)
                 T(r_k | r_(k-1),motion,graph)
                 psi(x_k,r_k,m,M)].

This is conditioned on the relevant mapped-road motion assumptions. A production model can add an explicit free-motion/off-road mode, but doing so requires a normalized generative transition model and realistic on/off-road priors. Assigning an arbitrary constant 'off-road likelihood' is not a calibrated solution. The reference instead makes abstention explicit and labels all road weights as conditional.

The map term psi is a feasible-corridor/path factor. It must not be interpreted as a new independent map survey at every IMU tick. The implementation approximates it with local scalar conditioning, spatial event throttling, and a persistent map-error state. State-dependent candidate gating, route transitions, Gaussianization and pruning make the overall posterior approximate.

### 5.1 Continuous state in the reference

The 14-state planar vector is

    [p_E,p_N,v_E,v_N,yaw,b_ax,b_ay,b_g,
     b^p_E,b^p_N,m_E,m_N,b^v_E,b^v_N].

The production equivalent should use the existing three-dimensional error-state filter, including its quaternion conventions, mounting states, lever arms and calibration states. Adding states alone cannot make unobservable variables identifiable.

The reference uses continuous-time-inspired first-order Gauss–Markov GNSS biases:

    b_(k+1) = exp(-dt/tau) b_k + eta,
    Cov(eta) = sigma_b^2 [1-exp(-2dt/tau)].

Persistent urban position bias is not well represented as independent white noise. Velocity bias matters too: repeated slightly biased receiver velocity can falsely justify a cross-track drift. However, this simple colored model is only an approximation to a receiver's internally filtered output. The synthetic tau/sigma settings are illustrative and must be estimated from installed hardware recordings.

### 5.2 Robust PVT likelihood

The prototype observes position plus position bias and velocity plus velocity bias:

    z = [p + b^p, v + b^v] + noise.

Use a normalized two-component measurement-noise mixture:

    g(z | x) = (1-epsilon) N(z;h(x),R)
               + epsilon N(z;h(x),R + R_outlier).

Both components perform Gaussian updates. Their posterior weights use the full evidence, including covariance determinants. Their moments are combined with between-component covariance. This avoids using only a Mahalanobis residual to favor an arbitrarily diffuse hypothesis. It does not make all multipath errors independent or detectable, and moment matching loses multimodality within a branch.

A real M9 integration should use valid NAV-COV blocks and observed residual statistics. The documented separate 3-by-3 position and velocity blocks do not provide position–velocity cross-covariance or inter-epoch correlation [S6]. Zeroing the missing blocks is an engineering approximation, not a claim of independence. Course derived from the same velocity is not another observation.

## 6. Road corridors and uncertainty

For road centerline gamma_r(s), unit local normal n, and map translation m, define

    d = n^T [p - m - gamma_r(s)].

The constraint is

    -w <= d + epsilon_boundary <= w,

where w is a conservative road half-width and epsilon_boundary captures residual local boundary uncertainty. A finite-width corridor avoids treating a legitimate lane offset as navigation error. Width, geometry quality, antenna lever arm and divided-carriageway semantics must be handled by the compiler and estimator. At intersections, several geometries should remain possible.

The map offset m is persistent. Its uncertainty is not refreshed independently at every epoch, so repeated use of the same road cannot magically average away a single survey offset. More realistic production models may use spatially correlated offsets shared across tiles; resetting independent offsets on every tile boundary would be wrong.

### 6.1 Interval-conditioning equations

Locally linearize y = Hx + c + epsilon, with mean mu and variance S = HPH^T + R. For interval [l,u], set

    a = (l-mu)/sqrt(S), b = (u-mu)/sqrt(S)
    Z = Phi(b)-Phi(a)
    lambda = [phi(a)-phi(b)]/Z
    y_bar = mu + sqrt(S)*lambda
    V_y = S * [1 + (a*phi(a)-b*phi(b))/Z - lambda^2].

With cross-covariance c_xy = P H^T,

    x_plus = x + c_xy*(y_bar-mu)/S
    P_plus = P + c_xy*c_xy^T*(V_y-S)/S^2.

These are the exact first two moments for a single linear Gaussian interval event. The conditioned density itself is not Gaussian. Sequentially replacing it with a Gaussian, changing the projection on a polyline, and applying more factors is assumed-density filtering, not an exact multivariate truncated posterior. The implementation uses stable CDF differencing and refuses numerically remote intervals instead of forcing an extreme projection.

Road normal constraints primarily affect cross-track position. On a straight infinite road, translating the vehicle along that road preserves the same corridor geometry; no algorithm can infer absolute along-track position from that corridor alone. Turns, known junction sequence, speed/distance information and GNSS supply additional constraints. The test suite includes an along-track non-invention check.

### 6.2 Repeated geometry and heading

Applying the same Gaussianized interval repeatedly while stationary would artificially shrink uncertainty. The reference applies a new map factor only on entering a previously unused 8 m road cell, identified by map version and canonical edge ID. This is a practical approximation, not a proof of correct temporal map correlation. Cell history is bounded and shared through branch splits. The persistent map-error state and spatial throttling address different failure modes; neither replaces field calibration.

A broad road-heading interval is also used when moving, away from intersections. It is deliberately not an exact yaw sensor. It is disabled near nodes and at low speed. A production classifier must distinguish reversing, parking maneuvers, slip and non-road use. Turning is not by itself a violation of ideal non-holonomic motion, but transient sideslip and mounting error require guards. Do not also add an independent turn score derived from the same gyro increments unless the joint likelihood is modeled.

## 7. Route transitions, evidence and output policy

A branch carries navigation state/covariance, road ID, short path history, map-event history and log weight. Seed candidates from uncertainty around the map-free solution; fork only to explicit successors. Update normalized log weights using PVT evidence and geometry-event probabilities. Avoid joining roads merely because polylines intersect.

The prototype caps the bank at 12 branches and three per edge. It exposes discarded probability mass at each pruning step. These are computational limits, not a guarantee that the correct branch survives. Cumulative model error and candidate omission are not bounded by a small current-step discarded mass.

The illustrative latch rule requires top route-group weight at least 0.97, top-to-second odds at least 30, three consecutive eligible epochs, and current pruned mass below 0.05. None of these numbers is a calibrated field threshold. Group IDs must represent the appropriate physical route, not just a street name that happens to recur. Production output also needs along-route spread and directed-edge confidence; a high broad-group weight does not prove a unique segment or lane.

Possible states are LATCHED, AMBIGUOUS, MAP_UNAVAILABLE and OFF_MAP. In this reference, OFF_MAP means no enumerated route remains consistent; it does not prove physical off-road motion. A missing tile is not negative geometric evidence. When ambiguous or unavailable, output the map-free position and uncertainty rather than a fake snapped fix.

For diagnostic mixture moments,

    P_mix = sum_j w_j [P_j + (mu_j-mu_mix)(mu_j-mu_mix)^T].

Omitting the between-mode term falsely shrinks uncertainty. Even a correctly computed mixture mean may lie between roads and must not be labeled a routable position. The demo publishes the best branch only after latching; the mixture is diagnostic.

## 8. Two confidence questions must remain separate

**Which road?** This is a topological association question. A turn sequence may identify it strongly even with poor absolute positioning.

**How accurately is the road aligned with the world?** This is a map/geometric calibration question. Correct road identification does not prove meter-level coordinate accuracy.

The shifted-map experiment makes the distinction concrete: the road ID is correct, yet geometric feedback worsens position RMSE. For first production deployment, expose road IDs in shadow/output-only mode. Enable physical-state map correction only in regions where map alignment and corridor semantics have been validated. A navigation-versus-map consistency monitor is necessary, but a flexible GNSS bias state can absorb a bad map; consistency alone cannot always identify which source is wrong.

Maintain the independent fallback and consider delayed branch recovery from a short raw-observation buffer. A full fixed-lag smoother/replay recovery is a recommended extension, not implemented in this reference. Never reset the map-free filter from the selected map branch, which would destroy the escape path.

## 9. Tile design and low-bandwidth operation

Prepare a **routing graph**, not an image tile. Each directed edge needs a canonical ID, geometry, allowed successors, width/uncertainty and quality flags. Keep map version, coordinate frame, coverage state and edge identity separate from tile membership. An edge crossing two tiles must remain one logical edge and one map event source.

Overture explicitly represents physical connectivity using connectors; coincident geometries without shared connectors are not connected [S11]. Resolve interior connectors and turn/access restrictions on the server. Its level/layer-like metadata is not a measured elevation. Two different levels do not alone establish a GNSS height constraint, and geometric crossings must not create a shortcut through an overpass.

The included compiler accepts already-directed GeoJSON and converts it to a local EN frame before writing 256 m JSON+gzip tiles. It intentionally does not infer OSM topology. The demonstration's local-frame radius is limited to 10 km. A production frame change must transform states and covariances coherently rather than abruptly replacing coordinates.

Cache by version/content hash. Prefetch the reachable route fanout, not only the currently selected road, and include the uncertainty envelope. Deliver replacements atomically; never compare a filter state using one geometry version against half of another. Distinguish known-empty coverage from not-downloaded data. Delta updates and a compact quantized binary format are production optimizations after correctness.

No continuous cloud correction stream is needed for this design. GNSS satellite diagnostics over UART consume local serial bandwidth, not cellular bandwidth. Cellular use consists of map cache fills, changed tiles and chosen telemetry. Illustrative arithmetic, not measured city payloads: a 32-byte report every minute is 46,080 bytes/day, about 1.38 MB per 30 days before transport, security, retries and connection overhead. One hundred hypothetical 20 kB new tiles are 2 MB. Actual tile density must be measured on the target cities; toy example sizes are not representative.

## 10. Resource design and 3DMA extension

A bounded bank makes allocation predictable. As a storage-only illustration, twelve 21-state branches with full float32 covariance require 12*(21*21+21)*4 = 22,176 bytes for those arrays, excluding graph, history, base filter, buffers, runtime and stack. This is not a total RAM or CPU estimate. The Python reference uses float64 and has not been benchmarked on an MCU. Reuse buffers, propagate with the existing optimized ESKF, and profile branch expansion at dense junctions.

Full GNSS raw measurements are product dependent on M9 [S7], but irrelevant to the first route-aiding release. A later skyline/shadow-matching likelihood may help distinguish nearby streets using per-satellite signal observations and building geometry [S14]. That likelihood must not be returned as an independent position fix if it already incorporates the same GNSS/INS prior. Reflection-aware pseudorange processing is a separate, more demanding extension. It is not required to benefit from road topology.

## 11. Validation and rollout

Use four levels: mathematical tests; adversarial synthetic scenarios; recorded-data replay; physical deployment. The present delivery completes the first two and provides a replay contract. It does not complete real-world qualification.

Compare at least the current deployed filter, the same robust augmented filter without maps, output-only HMM matching, single-road pseudo-measurement feedback, and the proposed multi-hypothesis system. Use the same epochs, maps and ground-truth intervals. Report road identity separately from position accuracy, wrong-road dwell, off-road detection delay, abstention, reacquisition time, covariance consistency, data volume and worst-case compute. Do not report only accuracy on successful latched samples.

UrbanNav is one potential public replay resource [S12], but sensor and antenna differences remain; no UrbanNav run is claimed here. Record actual M9/LSM6DSO hardware with installation, temperature and timestamp metadata. Label map errors and grade separations independently; urban reference trajectories also need their own uncertainty bounds.

Freeze tunings before held-out routes, days, temperatures and map regions. Prefer a higher unknown-route rate to a confidently wrong road. A release gate should be an explicitly chosen maximum wrong-road dwell/frequency on the intended use cases, not a visually smooth trajectory.

## 12. Bottom line

The clean first implementation is **route-conditioned loose coupling with bounded alternatives and an independent escape path**. It can exploit existing hardware and cached graph tiles without a continuous correction link. Its value comes from topology and motion consistency, not from pretending a road map is an exact second GNSS receiver. The prototype's positive cases support further engineering; its ambiguous, stale-map and off-road cases define the limits that must remain visible in the product.

## Primary sources and access scope

Original URLs are recorded in `sources.json` in the accompanying prototype archive. Sources informed the design; the code is an original reduced reference, not a port of any cited implementation.

- **[S1]** Newson and Krumm (2009), Hidden Markov Map Matching Through Noise and Sparseness. 10.1145/1653771.1653818. Official author page and paper; topology-aware sequence matching, not a consistent navigation-feedback interface.
- **[S2]** Heirich (2016), Bayesian Train Localization with Particle Filter, Loosely Coupled GNSS, IMU, and a Track Map. 10.1155/2016/2672640. Full publisher text. Discrete route and conditional continuous estimation; rail constraints are stronger than road constraints.
- **[S3]** Falco, Einicke, Malos and Dovis (2012), Performance Analysis of Constrained Loosely Coupled GPS/INS Integration Solutions. 10.3390/s121115983. Full publisher text. Precedent for road-boundary constraints with loose coupling; not adopted wholesale.
- **[S4]** Simon and Simon (2006), Constrained Kalman Filtering via Density Function Truncation. NASA/TM-2006-214129. NASA full report and page images. Basis for inequality conditioning rather than centerline projection.
- **[S5]** Wang, Chen and Weng (2025), GNSS/IMU/map-matching feedback integration with adaptive GNSS accuracy estimation by using low-quality sensors for vehicle localization in urban canyon. 10.1016/j.measurement.2024.115541. Publisher abstract/highlights and institutional record only; full article not inspected. Incorrect feedback and motion-dependent validation are relevant cautions.
- **[S6]** u-blox M9 SPG 4.04 Interface Description, UBX-21022436 R02. UBX-21022436 R02. Official interface document, including NAV-PVT and NAV-COV page images. Separate position and velocity covariance blocks, not complete PVT temporal correlation.
- **[S7]** u-blox GNSS raw data capability table. Official capability table; full RAWX support is product/firmware dependent. Not needed for this road-map approach.
- **[S8]** STMicroelectronics LSM6DSO product documentation. Official product information; no sensor-noise or outage-duration performance claims are inferred.
- **[S9]** Valhalla Meili map-matching algorithms. Official project documentation; HMM baseline and implementation reference.
- **[S10]** Valhalla Baldr tiled graph architecture. Official project documentation; server preprocessing and tiled graph design reference, not an MCU dependency.
- **[S11]** Overture Maps transportation: Segments and Connectors. Official schema guide. Connectivity comes from explicit connectors, not geometric intersections; level is not measured altitude.
- **[S12]** UrbanNav dataset, PolyU IPN Lab. Official dataset page. Candidate future validation; no UrbanNav traces were run in this delivery. Check release-specific usage terms.
- **[S13]** OpenStreetMap copyright and licensing. Official attribution/licensing information; check applicable obligations when building distributed map products.
- **[S14]** UCL 3D mapping-aided GNSS research project. Primary research overview. Optional independent environmental likelihood, not part of this prototype.
- **[S15]** OSRM HTTP API, Match service. Official project documentation; useful offline route-matching baseline, not navigation-covariance truth.

---

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


---

# Integrating with an existing navigation filter

## Preferred integration boundary

Retain the existing full 3-D INS/error-state mechanics. Supply cloneable state/covariance, chronological prediction, original receiver PVT update, and a local map-factor update. The branch bank should own alternate conditional navigation solutions. It must not send its final position into the original filter as an independent sensor.

Conceptual adapter:

```python
class NavigationAdapter:
    def clone(self): ...
    def predict_imu(self, timed_increment): ...
    def update_original_pvt(self, observation) -> float:
        # Return normalized log marginal likelihood, not just NIS.
        ...
    def local_pose_and_covariance(self): ...
    def condition_road_interval(self, geometry, map_error_model) -> float: ...
```

ESKF map update: linearize the corridor distance with respect to error coordinates; include position, map offset and applicable antenna attitude/lever-arm terms. Update all states through cross-covariance. Inject the resulting error into the nominal state and apply the correct error-state reset Jacobian. Preserve the filter's quaternion convention; the planar demo uses no quaternion. The map-free state never receives a correction copied from a map-conditioned branch.

State history must include any colored GNSS bias and map-error variables, not just pose. Clone after a common update or before it consistently. The prototype seeds from an already-PVT-updated fallback and does not consume that PVT again.

If the existing filter is a black box exposing only PVT-like posterior snapshots, use a route matcher for **output labels only**. Consecutive snapshots are correlated and contain the same GNSS history. Full feedback needs the observation interface, joint cross-covariance, or a different explicitly conservative information-fusion formulation. Do not simply declare the snapped coordinate independent.

## Timing, mounting and observations

Align every inertial increment and GNSS observation to measurement time. Use receiver epoch/time fields and hardware timestamp/PPS support where available; serial arrival time is not the measurement epoch. A late PVT update requires time alignment or buffered repropagation; the demo rejects out-of-order input rather than implementing late-data replay.

Choose one state reference point. Model the GNSS antenna lever arm relative to IMU/vehicle and rotational velocity effects. Resolve the sensor-to-car mounting before applying non-holonomic constraints. The standalone planar example expects horizontal, gravity-compensated body acceleration with x forward and y left, and yaw rate positive counterclockwise. Raw accelerometer samples include gravity and cannot be fed into that model unchanged.

Prototype local coordinates: east/north in meters; velocity m/s; angles radians; timestamps seconds. WGS84 builder input is longitude, latitude, optional ellipsoidal height. The full INS may remain ECEF/NED internally; transform map residuals and covariance Jacobians coherently.

For M9 SPG4.04 verify exact receiver/firmware with MON-VER, and decode NAV-PVT plus valid NAV-COV when available. NAV-PVT velocity fields and NAV-VELNED have different documented unit scales; use the exact message specification, not a shared guessed scaling. For horizontal EN from NED use [east,north] = [NED_y,NED_x] and transform both covariance blocks. Preserve measurement validity flags. NAV-COV gives separate position/velocity blocks, not all cross-block or temporal correlations [S6 in sources.json]. Treat missing information explicitly, validate conservative assumptions on replay, and never treat hAcc as a guaranteed integrity bound.

Optional NAV-SAT/NAV-SIG can diagnose environment changes. They are not required for the reference road matcher. Standard M9 raw-measurement restrictions do not block PVT-level route aiding. If a dead-reckoning receiver already fuses the same IMU, its PVT is not independent of those inertial samples; that requires a different correlation treatment.

## Event-log format

An example is in `results/parallel_bias_example/`. No truth is read by `replay.py`.

`initial.json` contains `x` (length 14) and `P` (14x14) in the planar reference ordering. `tiles/` contains the map cache. `events.jsonl` contains chronological records:

```json
{"kind":"imu","accel_body":[0.0,0.0],"gyro_z":0.0,"dt":0.2}
{"kind":"epoch","t":1.0,"map_available":true,"pvt":{"epoch":1,"t":1.0,"position":[8.0,0.0],"velocity":[8.0,0.0],"covariance":[[6.25,0,0,0],[0,6.25,0,0],[0,0,0.0225,0],[0,0,0,0.0225]]}}
```

Propagate to the epoch before the epoch record. `pvt: null` represents missing GNSS while inertial/map processing continues. An absent tile cache is modeled separately from missing GNSS. The standalone planar record is deliberately not a UBX decoder; convert hardware recordings through a verified adapter.

## Tile compiler contract

The input is a GeoJSON FeatureCollection containing directed LineStrings. Required properties:

```json
{"edge_id":"road-segment-123-forward","successors":["road-segment-124-forward"],"half_width_m":6.0,"layer":0,"group":"physical-route-A"}
```

Group related contiguous segments carefully; never use a broad street name as a uniqueness guarantee. Supply reverse-direction edges explicitly for bidirectional roads. Resolve access, turn restrictions and interior connectors before compilation. The demo does not implement a reverse-driving classifier or arbitrary raw OSM extraction.

```sh
python -m routefusion.build_tiles directed_roads.geojson tiles \
  --origin -79.75 43.70 180 --version example-v1
```

The coordinates above are only an example frame origin. No map near that location was downloaded or validated. The compiler limits local patches to 10 km; use smaller coherent patches or implement frame rebasing. JSON+gzip is a readable demonstrator, not an optimized firmware encoding. The compiler currently covers the input bounding rectangle as known coverage: use it only for a complete graph extract, or replace that policy with an explicit coverage manifest. A partial graph is not proof that omitted roads do not exist.

Cross-tile references use canonical edge IDs and compatible map versions/frames. Missing successor data triggers unavailable-map fallback; it must not favor the other branch as evidence of a closed road. Version updates must be atomic with respect to active branch geometry.

## Production output contract

Export physical navigation and association separately:

- Timestamp, frame, physical pose/velocity and covariance.
- Candidate directed edges and along-edge coordinates; optional selected route ID.
- Status: mapped/ambiguous/unavailable/no-consistent-candidate; map version/age.
- Conditional branch weights, discarded mass and relevant uncertainty diagnostics.
- Map-free fallback estimate, at least in engineering logs.

Do not publish the weighted mean between parallel roads as a confident road coordinate. Do not interpret a group weight as lane confidence or a calibrated wrong-road probability. A future explicit off-road mode should have its own validated transition/observation model; it is not present here.

## Suggested integration stages

1. Log and replay original M9/IMU epochs with the existing estimator unchanged. Verify mounting, timing, covariance units and ground truth.
2. Add output-only route labels and compare against a standard HMM baseline. Measure false latch, abstention and turn recovery.
3. Introduce cloned conditional ESKF branches and finite-width factors in shadow mode, preserving map-free output.
4. Enable geometric correction only for validated map regions and operating modes. Keep immediate fallback and raw-log recovery.
5. Profile RAM, worst-case junction compute, allocation, tile fetch and energy on the target MCU. Consider fixed-lag replay and skyline likelihood only after core failure behavior is understood.

The production implementation should add explicit stop/reverse/off-road modes, robust map-quality gating, late-data replay, long-distance frame management, and a validated ambiguity-aware null model as needed. These are listed as work, not claimed as implemented features.
