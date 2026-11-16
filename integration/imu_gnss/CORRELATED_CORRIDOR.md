# Correlated native road corridors: matched research replay

## Scope and status

This opt-in stage connects the finite-width road corridor from PR #9 to the
native event-tracked map cross covariance from PR #12, using the reference
principle in PR #11. Baseline is main `9084af21850424b1a825cb8992fbf0b8eba501b9`.
It adds a separate module, evaluator, tests and qualification workflow. Existing
native equations, defaults, road matcher, support gates, workflows, Mergify and
the published playback are unchanged.

Local validation: **178 focused Python tests passed**, including **44 new tests**,
38 unchanged native-adapter/verifier tests, 27 reference tests and 69 support
tests. The new tests use independent scalar numerical integration, full
augmented Joseph algebra and fake-native lifecycle fixtures. **They do not
constitute a native recorded-road replay.** No Rust compiler is available in
this local environment. The added CI workflow must build and test the pinned
original and traced runtimes, regenerate causal road associations, replay all
four mapped recordings and independently repeat the numerical outputs before
any integration result is claimed. This change is not deployed.

## Model and approximation

Let x be the 18-dimensional navigation error, m a fixed-frame 2-D map
translation error, and C=Cov(x,m). For one scalar cross-track constraint,

```
Q = [[P, C], [C^T, U]]       y = d + h*x + j*m + epsilon
j = -n_ENU                  h[position] = n_ENU * R_ENU_from_native_NED
S = [h j] Q [h j]^T + R_local
```

Here d is the predicted distance from the proposed map line. The observation
is an interval event `-w <= y <= w`, not a zero-distance centerline fix. Let
`mu=E[y|interval]`, `v=Var(y|interval)`, `a=Q*[h j]^T`, and `beta=1-v/S`.
For the local linear/Gaussian model the navigation conditional moments are

```
dx = a_x * (mu-d)/S
P_new = P - beta*a_x*a_x^T/S
C_new = C - beta*a_x*a_m^T/S
U_new = U                     map nominal mean remains zero
```

The navigation first/second moments equal those of one linear Gaussian interval
event. **Holding the map mean and U fixed is a Schmidt-style moment projection,
not the exact joint interval posterior.** In particular, it is not proof of
conditional Bayesian consistency under arbitrary map bias, nonlinear road
association, successive truncations or incorrect road widths.

For `beta>0`, the same moments are obtained from a scalar Gaussian Schmidt
update with

```
S_eff = S/beta
R_eff = R_local + S*(1/beta-1)
residual_eff = (mu-d)/beta
K_aug = [beta*a_x/S; 0]
```

The full Joseph covariance matches the implemented P/C update and preserves
positive semidefiniteness in exact arithmetic. Tests compare that independent
construction and numerical integration over the interval. A fully uninformative
interval is a no-op. An interval with log probability below -12 is rejected,
as in the existing corridor implementation. No threshold is relaxed.

Native `rf_moments` applies the 18-state correction and covariance. The exact
native reset operator transforms C afterward. All IMU and GNSS events pass
through the existing native transition adapter even when no road is supported.
A native failure or invalid transition permanently invalidates the affected
instance; there is no silent continuation, covariance reset or raw-EKF fallback.

## Stable map identity and frame

The experiment uses **one shared translation for the entire downloaded extract
within each independent recording**. Its ID hashes the map response SHA-256
and the fixed local frame origin. U is expressed in that fixed ENU frame and
retained through edge changes and native reanchoring; native reanchor operators
transform only the navigation side of C. It is never replaced with a fresh
independent prior when the path or graph-edge identity changes.

This is deliberately a limited correlation model. A single translation does
not describe spatially varying surveying errors, wrong lane geometry or
junction shape. No multi-patch retirement/reintroduction policy is claimed.
The default shared map sigma, 1.5 m per axis, is an experimental setting, not a
calibration of OSM or evidence of lane accuracy.

## Matched comparison

The runner rebuilds the original causal RoadBank associations using the unchanged
native baseline. The existing bounded path-support selector then uses the raw
EKF only. It proposes **the same world-frame normal, point, width and spatial
cell** to both corrected arms. Each arm still checks its native availability,
finite path support and increment guards. Accepted counts can consequently
differ; improved road selection must not be inferred from this experiment.

The two arms differ only in the treatment of shared map uncertainty:

- **Correlated interval:** U is fixed, C is retained, and only local noise enters R.
- **Independent-noise control:** U=C=0, and `j*U_shared*j^T` is added independently
  to each local factor variance. This is an intentional comparison model, not
  a recommended way to treat repeated map error.

Both use the same `R_local=(sigma_knob^2+curve_departure^2)/quality`. The boundary
quality affects local measurement strength, not the persistent map prior. This
makes the first factor's navigation moments identical when the priors match;
a regression explicitly checks that map uncertainty is not counted twice.

The proposed spatial key uses the canonical path and the raw support's 10 m
progress cell, not the corrected position. Graph segmentation therefore cannot
provide extra information. Each accepted cell is consumed once per arm, and the
budget is retained through gaps. This stage **does not increase measurement
rate** or claim general rate invariance or independence of all local residuals.

A third traced-native arm receives no map factors and must remain bit-identical
to the original native baseline in all 357 navigation/state ABI slots at every
sensor event. The object allocation-size field is validated separately, exactly
as in PR #12. All initialized outputs remain available; map validity gates the
next measurement, not the position output.

## Qualification and reproduction

Use the pinned original native source and the existing build preparers:

```sh
export PYTHONPATH="$PWD/routefusion_prototype:$PWD/integration/imu_gnss"
python -m pytest -q integration/imu_gnss/correlated_corridor_tests
python integration/imu_gnss/prepare_native.py \
  --upstream .cache/upstream --output .cache/native-control
python integration/imu_gnss/prepare_native_transition.py \
  --upstream .cache/upstream --output .cache/native-trace
# Build/test each prepared workspace with the existing locked Rust procedure.
python integration/imu_gnss/prepare_support_inputs.py \
  --library .cache/native-control/target/release/libroutefusion_native_bridge.so \
  --datasets .cache/upstream/web/datasets --map-cache integration/imu_gnss/map_cache \
  --output runs/corridor-seeds
python integration/imu_gnss/evaluate_correlated_corridor.py \
  --playback runs/corridor-seeds/playback-data.json \
  --datasets .cache/upstream/web/datasets --source-root .cache/upstream \
  --control-library .cache/native-control/target/release/libroutefusion_native_bridge.so \
  --traced-library .cache/native-trace/target/release/libroutefusion_native_bridge.so \
  --sigma-m .25 --map-sigma-m 1.5 --output runs/corridor-first
```

Output directories must be new. The focused CI repeats the complete four-recording
comparison and requires byte-identical trajectory/diagnostic JSONs. It checks all
37 upstream source hashes, all 159,075 sensor events, 1,569 exported epochs and
1,402 initialized outputs; unchanged inputs, preserved native instances, unchanged
moments on skipped map updates, and native joint covariance checks. These are
**qualification requirements**, not claims of a completed run until the workflow
actually passes. Failure writes a diagnostic and terminates instead of substituting
another trajectory. The artifact includes pinned build provenance and both native
libraries for independent inspection; it is not a release or deployment.

## Interpretation boundaries

Report map-update step size, displacement from the independent EKF, factor
availability and uncertainty separately. Smoother motion or smaller displacement
is not a positioning accuracy result. The baseline supplies association, not
surveyed truth. Do not report NEES or accuracy from agreement with the same map.

Remaining work includes meaningful independent truth, sensitivity to incorrect
map geometry and shared-error assumptions, native integrity/reseed recovery,
map-patch transitions, and efficient in-Rust cross-covariance storage. A successful
replay is a real-data integration test, not production navigation qualification.
