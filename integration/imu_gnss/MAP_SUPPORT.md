# Connected map support: first implementation stage

## Scope

This is an opt-in continuation of the persistent map-pseudomeasurement EKF
experiment. The default map-free navigation, original RoadBank, recovery policy,
GNSS-bias models, existing checks, Mergify and the separately authorized Pages
publication are untouched. No surveyed accuracy or production qualification is
claimed. The corrected filter never changes its state source at association gaps.

This stage improves **support availability and geometric handoffs**, not the
entire road matcher. Accepted seeds still come from the original causal RoadBank.
Between seeds, bounded continuation is calculated online from the independent
raw EKF and the map. The corrected state is not recycled as fresh independent
association evidence. This is not a new HMM or lane-level localization system.

## Implemented behavior

`path_support.py` builds a geometric path from explicit successors only when the
connection is unbranched in both directions, its endpoints coincide, width/layer
agree, and the turn angle is at most 60 degrees. It does not infer connectivity
from 2-D crossings. Forks, merges, missing links, different layers, gaps and sharp
connections are not stitched. Cycles remain unstitched pending a progress model.

Collinear forward vertices are removed without changing the road shape. A path
identity comes from geometry, width and layer, not graph edge names. Changing
which constituent edge is selected does not reset the support history or create
new measurement cells. Identical geometry split into different edges produces
identical states and covariance in the regression suite.

Ordinary internal vertices use analytic circular fillets that are tangent to
both incident line segments. Departure from the original polyline is bounded by
0.5 m; the maximum trim is 5 m and less than a quarter of either incident segment.
The construction uses `r*(1-cos(turn/2))` for its departure bound. It is a geometric
regularization with uncertainty, not evidence of a surveyed turning curve.
Unsupported sharp corners and open path ends receive a smooth ten-metre taper;
no fillet is created across a junction or between divided carriageways.

A previously accepted path can provide support for at most eight seconds after
its last accepted seed. Continuation must remain inside its finite geometry,
agree with motion and uncertainty, and pass a complete neighboring-tile/extract
coverage check. A plausible competing corridor prevents continuation. It never
averages two roads into a median position, and it never chooses a fork exit
without a new independently accepted seed. An explicit original association
failure clears support; it does not erase the corrected EKF. These bounds are
conservative engineering settings, not calibrated probabilities.

`supported_map.py` retains one native 18-state instance, processing every original
IMU/GNSS event. A support rejection skips only the next map factor. The module has
no raw-filter object from which to substitute a navigation state. Native numerical
failures terminate explicitly. Map version changes suspend measurements without
silently undoing past information or allowing the old version to reactivate.

Two measurement models are evaluated with the same support schedule:

* **Connected centerline:** the diagnostic strong lateral Gaussian factor; the
  sigma knob and curve-departure uncertainty determine its noise. This isolates
  a useful comparison with the previous aggressive experiment, but geometry and
  acceptance are also different, so it is not a pure sigma-only ablation.
* **Soft corridor:** interval road-occupancy conditioning with the map half-width,
  a 1.5 m geometry-noise floor, curve-departure uncertainty and boundary taper.
  It allows legitimate positions inside the road rather than attracting every
  vehicle to the centerline. Widths remain the original unsurveyed map defaults.

For the corridor arm the effective variance is

```
R = (sigma_knob^2 + sigma_map^2 + curve_departure^2) / quality
```

where `quality` fades toward zero at unsupported boundaries. The map factor is
lateral only; no along-road fix is invented by projecting the estimator onto its
own road. The full state/covariance updates use the native error-state injection.
Gaussian centerline factors use Joseph covariance; corridor factors use the
existing interval first/second moments. The native attitude/mount increment
guards remain. No output smoothing, correction multiplier or covariance inflation
is added.

## Information reuse remains deliberately conservative

Each canonical geometric 10 m cell is consumed once per persistent estimator
history. A graph split or association gap cannot create a fresh copy of that
information. A static measurement queried 100 more times adds no information in
the tests. This does **not** establish general update-rate invariance on moving
trajectories, or model all spatial/temporal map correlations. A proper correlated
map-error/Schmidt treatment is a later stage, not covertly approximated by sending
more high-confidence updates. No sampling-rate increase is introduced here.

## Executed local qualification

The existing pinned original Rust library was reused, SHA-256
`5ad7955f748154e3ae66ed6fd5f04c0519991aac3ca3298dbcd772d4cd93affd`.
All 37 pinned native source hashes were verified. This is fresh sensor replay,
**not a fresh local Rust build or local upstream Rust test run**.

All four unmodified mapped recordings were replayed at all seven sigma settings
(0.05, 0.1, 0.25, 0.5, 1, 2, 5 m) for both new measurement models: **56 corrected
native replays**. All were independently repeated, with byte-identical numerical
trajectory and factor-diagnostic files. The original automatic associations were
also rebuilt in four fresh native RoadBank runs; raw coordinates, association
states, edge identities and epochs match the retained original evidence.

All **101 local tests pass**: 48 new support tests plus all 53 unchanged earlier
artifact tests. Existing repository checks are not relabeled as local passes;
the added CI workflow rebuilds the pinned runtime and runs the repository's
original integration/recovery tests alongside the new support suite.

For each sigma/model there are **1,402 initialized outputs** across **1,569
exported epochs**. No corrected instance is replaced, no raw-EKF substitution
occurs, all skipped factors preserve the complete native snapshot, and the
independent raw/control/map-disabled states are bit-identical. Every arm consumes
all **156,603 IMU and 2,472 GNSS events**. Covariance checks pass.

### Results at sigma = 0.25 m

| Recording | Original selected epochs | New supported epochs | Previous map updates | Connected centerline updates | Soft-corridor updates |
|---|---:|---:|---:|---:|---:|
| Mixed road | 258 | 326 | 160 | 219 | 220 |
| Short loop 001 | 55 | 67 | 40 | 48 | 48 |
| Short loop 002 | 71 | 84 | 46 | 55 | 55 |
| Stop/go turns | 118 | 143 | 77 | 91 | 91 |
| Total | 502 | 620 | 323 | 413 | 414 |

There are 119 newly supported previously unaccepted epochs and one formerly
accepted epoch now rejected by the path compatibility check: net gain 118. These are
**support/update counts, not correctness or accuracy success rates**. Original
association confidence thresholds were not lowered.

| Recording | Previous maximum individual map step | Connected centerline | Soft corridor |
|---|---:|---:|---:|
| Mixed road | 6.642 m | 6.545 m | 0.725 m |
| Short loop 001 | 3.283 m | 2.712 m | 0.043 m |
| Short loop 002 | 2.205 m | 2.177 m | 0.013 m |
| Stop/go turns | 3.156 m | 2.098 m | 0.036 m |

The aggressive centerline still produces metre-scale steps. The softer corridor
reduces them because it asserts less precise information, **not because we have
proved better physical accuracy**. Broad corridor support can yield little mean
correction when the vehicle is already plausibly within the road. There is no
post-hoc snapping or cosmetic smoothing of the plotted estimates.

Machine-readable local evidence: `support-local-validation.json`. Fresh exact-head
CI must pass before any merge; prior build results do not qualify changed code.

## Reproduction

Use the existing pinned original native build procedure in this directory, then:

```sh
export PYTHONPATH="$PWD/routefusion_prototype:$PWD/integration/imu_gnss"
python -m pytest -q integration/imu_gnss/support_tests
python integration/imu_gnss/prepare_support_inputs.py \
  --library .cache/native/target/release/libroutefusion_native_bridge.so \
  --datasets .cache/upstream/web/datasets \
  --map-cache integration/imu_gnss/map_cache --output runs/support-inputs
python integration/imu_gnss/evaluate_support.py \
  --playback runs/support-inputs/playback-data.json \
  --library .cache/native/target/release/libroutefusion_native_bridge.so \
  --datasets .cache/upstream/web/datasets --source-root .cache/upstream \
  --sigma-m .05 .1 .25 .5 1 2 5 --output runs/support-results
```

Output directories must be new. Sensor inputs and the original matcher are never
modified. The added `qualify-map-support.yml` does a fresh pinned Rust build/test,
rebuilds causal associations, executes both seven-sigma models twice, requires
byte-identical numerical repetition, and uploads evidence. Existing workflows
are unchanged. Neither this workflow nor this PR deploys a website.

## Explicit remaining work

Online hypothesis weighting with shared approach support is not implemented by
this bounded continuation adapter. Truly ambiguous exits remain unsupported.
Lane-specific widths and surveyed map calibration are unavailable. Persistent
wrong map information still needs an explicit integrity/recovery policy. Exact
correlated map-error treatment and general measurement-rate invariance remain
separate requirements. Do not claim universal continuous map measurements,
production qualification, or measured accuracy improvement from these results.
