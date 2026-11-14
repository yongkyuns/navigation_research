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

Original URLs are recorded in `../sources.json`. Sources informed the design; the code is an original reduced reference, not a port of any cited implementation.

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
