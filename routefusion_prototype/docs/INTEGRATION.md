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
