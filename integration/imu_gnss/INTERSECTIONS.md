# Maneuver-aware intersection handling

## Scope and output contract

This revision changes the road-hypothesis manager, not the pinned Rust
`SensorFusion` equations. Published navigation remains **shadow/map-free**.
Road-conditioned positions remain separately named experimental outputs. The
original `evidence/` and `REPORT-initial-native.md` describe the pre-intersection
implementation at `bc8cbb91b6efc82cadd0ce2c67769b93a07b9352`; they are retained,
not overwritten.

The old manager generated outgoing segments and immediately applied their
corridor and direction gates. Before a right-angle turn, `t_exit dot v` is near
zero, so the correct turn could be discarded while still approaching. It also
split transition priors repeatedly and used OSM-way grouping for publication.

## State and measurement ownership

A branch now carries `(incoming edge, connected exit path, phase, map offset,
native state/covariance, log weight, used map cells)`. A maneuver is a **future
association alternative**, not a commanded or simulated turn. Every clone uses
the same original IMU/GNSS samples once. No additional gyro-turn likelihood is
multiplied into the posterior, and no snapped coordinate is fed to GNSS.

The three phases are:

1. **Approach.** Retain all enumerated connected exits. Apply only the incoming
   road's existing finite-width interval factor. A confident current incoming
   edge may still be reported; `future_exit` remains unknown.
2. **Junction.** Retain the maneuver alternatives inside a common broad support
   envelope. Apply neither the outgoing-road direction gate nor its corridor
   update. This is a neutral map factor inside the envelope, not an assumed turn
   centerline, radius, or independent position observation. Report ambiguity.
3. **Exit pending / road.** After clearing the envelope, require outgoing
   position support and velocity alignment at two consecutive map epochs.
   Only then apply that exit's normal road-corridor update. Normal publication
   also requires three eligible epochs and >97% retained directed-edge weight.

For incoming endpoint `c`, each path follows only explicit successor IDs with
matching endpoints. Short connectors are traversed in the topology search up to
three edges. The common radius is at least 12 m and covers connector start
points plus twice their road half-widths. A maximum 50 m radius and bounded
path search prevent unbounded expansion. A margin `min(15 m, 2 sigma_position)`
softens the envelope boundary. Outgoing velocity alignment uses cosine >=0.5
only after clearing it; at speed <=2 m/s the exit remains unresolved.

These are research policies, not surveyed intersection polygons or calibrated
probabilities. Missing successor data is `MAP_UNAVAILABLE`, not evidence for a
different exit. Unresolved short-link chains or excessive fanout abstain with
`HYPOTHESIS_OVERFLOW`; they are not silently truncated to a preferred exit.
Turn restrictions, conditional access, long roundabouts and lane-level geometry
remain outside the current OSM compiler's contract.

## Diversity and bounded work

The retained bank still has a configurable cap (default 24). Selection reserves
one representative for every distinct `(incoming, exit path)` maneuver, then
uses remaining capacity for additional map-offset/state alternatives. Distinct
maneuvers beyond the cap cause abstention, rather than renormalizing the few
survivors into false certainty. All evaluated candidates, including discarded
ones, contribute to the discarded-mass diagnostic.

Candidates are constructed and evaluated incrementally: at most cap+1 child
clones coexist with the preceding <=cap parents. Map-offset alternatives are
still approximate, and pruning them can change route weights. No cumulative
integrity bound is claimed. Publication uses a **directed edge**, not an OSM way
or street name; hypotheses inside a junction never certify an exit merely
because their way IDs agree.

## Bounded rollback recovery

On first approach, retain one map-free native checkpoint and a single shared
chronological event buffer (20 s / 4,096 events by default). The checkpoint has
already consumed all original sensor data through its epoch, but contains no
map-conditioned posterior. Only subsequent events are recorded; map epochs are
recorded alongside IMU and GNSS delivery order.

If no route survives, make at most one replay attempt from that checkpoint with
the same bounded policy. The candidate bank is replaced only when the replayed
map-free snapshot is bit-identical to the live fallback, and viable alternatives
remain. The live fallback is never reset or updated by the replay. Exit latching
is held off on the recovery epoch and must qualify again afterwards.

Expiration, event overflow, reordered inputs, unavailable geometry, or map
version replacement invalidates the buffer rather than replaying a partial
history. This is rollback/re-filtering from a map-free checkpoint, **not** a
fixed-lag smoother and not guaranteed recovery. It intentionally loses map
information before the checkpoint. Tests demonstrate recovery after a deliberately
dropped turn; the four unmodified recording experiments did not demonstrate a
successful natural recovery.

At the evaluated host's 14,880 bytes per full native runtime, the conservative
transient bound is `(2*cap+4)` native instances, including recovery but excluding
the independent evaluation control, Python objects, raw-event storage, maps and
stack. For cap=24 this is 773,760 native bytes. Steady retained-state storage is
smaller. These bounds are not a claim of suitability for a small MCU. Packed
branch state and an explicitly scheduled replay budget remain firmware work.

## Validation and observed trade-off

Local validation passed **39 Python integration/policy tests** (10 existing and
29 new) and the unchanged 24 original planar tests. New cases cover left/right/
straight exits without GNSS, stationary ambiguity, short connectors, missing
successors, non-connected crossings, mode-budget pressure, one-time branching,
shared way IDs, delayed exits, replay expiration/order, cache/version changes,
clone lifetime bounds, and injected loss of a true turn. Two tests exercise
native Rust cloning and chronological replay using actual recorded inputs.
The deterministic navigation double and its synthetic junctions are unit fixtures,
not field positioning evidence.

All 35 accepted recordings were replayed again, with the same rejected duplicate-
timestamp package. Baseline navigation metrics, initialization, input hashes and
coverage were unchanged. In four paired map runs and two input stresses, the
map-free output was bit-identical to an independent native replay at every map
epoch; no negative horizontal candidate covariance eigenvalues were found.

The revised policy intentionally reports less certainty:

| Recording | Previous latch fraction | Revised latch fraction |
|---|---:|---:|
| Short turn loop 001 | 54.0% | 40.1% |
| Short turn loop 002 | 69.0% | 42.3% |
| Stop/go tight turns | 72.4% | 24.1% |
| iOS mixed-road drive 001 | 63.8% | 42.5% |

The definition is stricter (directed edge and unresolved-junction abstention), so
these fractions are not correct-road accuracy comparisons. The iOS run included
33 overflow-abstention epochs. There are no independently labelled intersection
exits in the provided recordings. Clean-run reference discrepancies remained
essentially unchanged. Reference independence is still unverified.

The prior single-outage coordinate benefit is **not retained**: during the
injected 90-120 s outage the revised candidate discrepancy was 2.928 m versus
2.927 m map-free (previous candidate: 2.450 m). During the injected north-bias
stress it was 17.478 m versus 17.506 m map-free. These results do not establish
urban-canyon accuracy gains. They are retained rather than narrowing junction
supports to improve the plot. Merge this as intersection-policy and robustness
work in shadow mode, not a production accuracy upgrade.

`intersection-local-validation.json` contains the local summary and source
hashes. The read-only native qualification workflow rebuilds the pinned Rust
runtime, executes its tests, reruns all recordings and stresses, verifies unchanged
baseline navigation and fallback isolation, and uploads exact-head evidence. It
does not rewrite historical results or commit generated data to the PR branch.

## Reproduction

Use the native build instructions in `README.md`, then:

```sh
python integration/imu_gnss/run_evaluation.py \
  --upstream .cache/fusion-upstream \
  --library .cache/fusion-native/target/release/libroutefusion_native_bridge.so \
  --map-cache integration/imu_gnss/map_cache \
  --output runs/intersections --report runs/intersections-report.md
python integration/imu_gnss/qualify_intersections.py --results runs/intersections
```

Each run directory must be new. Compare road association, availability and physical
position separately. Calibrated GNSS-error models, independently labelled junction
routes, mount covariance reset qualification and embedded profiling remain needed
before enabling geometric feedback in a deployed device.
