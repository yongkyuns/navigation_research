# Correlated map error: Schmidt reference and native integration gate

This is **a reference implementation and controlled synthetic comparison**, not
an extension of the currently published native estimator or a new on-road accuracy
result. It follows opt-in path/corridor support merged in PR #9 (commit
`820ef41037fdf35fe82329f41f04efd028f561e8`). The existing native 18-state
map-free filter, persistent map-corrected replay, thresholds, geometry, defaults,
GitHub Pages, and Mergify are unchanged.

## Why an additive map sigma per update is insufficient

The same downloaded road geometry contains shared errors. A path may be shifted
three metres for many hundreds of metres. Sampling it repeatedly does not create
many independent surveyed observations. In particular, using
`R_k = sigma_map^2 + sigma_local^2` independently at each map epoch makes the
common shift average away as `1/N`, even though repeated road lookup does not
improve the map.

PR #9 already uses **one map update per canonical 10 m geometric cell**. That is
valuable protection against simply increasing the query frequency, but it does
not capture shared error **between different cells** on one local map patch.
Removing the cell budget without modelling cross-correlation would be unsound.

## Proposed local model

For a single connected, locally consistent road-map patch, denote navigation
error `delta x` and an uncertain but static 2-D road displacement `delta m`:

```
             [ Pxx  Pxm ]
P_aug     =  [          ]
             [ Pxmᵀ Pmm ]

r = z − Hx*x_hat − Hm*m_hat
S = [Hx Hm] P_aug [Hx Hm]ᵀ + R_local
Kx = (Pxx Hxᵀ + Pxm Hmᵀ) S⁻¹
Km = 0    (Schmidt: do not adjust the pre-existing map state)
```

The navigation mean receives `Kx*r`; the map mean stays fixed. A Joseph-form
update with `K_aug=[Kx;0]` updates **both** the navigation covariance and its
navigation/map cross covariance. `Pmm` remains fixed, but `Pxm` becomes nonzero.
A map factor therefore cannot be treated as fresh independent information about
`x` when the next cell refers to the same uncertain map patch.

Normal inertial propagation must also update `Pxm <- F_x * Pxm`; a normal GNSS
update must apply its actual EKF update Jacobian to `Pxm`. A native error-state
injection or frame reanchor must apply the correct reset/frame Jacobian to
**both** `Pxx` and `Pxm`. Ignoring ANY of those terms destroys the claimed
consistency, even if `Pxx` stays positive semidefinite.

`map_correlation.py` implements this exactly for user-supplied **linear**
transition, sensor, map-factor and error-reset Jacobians. It accepts an
arbitrary navigation dimension, including 18, and 1/2-D map error. It
requires a stable map-patch key and a unique cell evidence key. It fails closed
on an unrecognised map patch, non-PSD covariance, invalid measurement, and
explicit innovation gate. Duplicate cell updates leave mean/covariance
bit-identical. It neither chooses a road nor creates map measurements.

### The non-negotiable native integration boundary

The current bridge (see `integration/imu_gnss/native.py`) exposes native
`rf_imu`, `rf_gnss`, `rf_moments`, `rf_snapshot` and `rf_clone`. It provides the
marginal **18x18** covariance, but **does not export the linearization/update
Jacobians used internally by IMU propagation, GNSS updates, native injection,
or reanchoring**. Therefore it is NOT possible to correctly advance the
Schmidt cross covariance across recorded events using only the existing bridge.

**This reference must not be pasted into `supported_map.py` with its cross
covariance left unchanged between 1 Hz map epochs.** Doing that would look
plausible while providing inconsistent feedback to the native EKF. The next
native integration change should expose an internal 2-D local map-error
Schmidt block with the relevant Jacobians and reset semantics inside the
shared Rust runtime/bridge, or explicitly expose validated transition/update
operations to the wrapper. It must survive unambiguous road transitions,
branch rejection, map-version changes and map-frame reanchoring without
reinitializing the same uncertain map as independent evidence.

Multiple map patches require a defined correlation/retirement policy. The
reference intentionally permits ONE map patch only. It never creates a new
independent nuisance state simply because the edge ID changes; doing so can
again create fake information. This is also distinct from GNSS-receiver error
states and does not modify the existing GNSS-bias-free navigation default.

## Synthetic controlled demonstration, **not** empirical road accuracy

`evaluate_map_correlation.py` evaluates a simple lateral position/velocity
linear Kalman model over 80 independently sampled 45-second trials. The ground
truth starts from the declared prior and follows the same linear process and
noise model as the filter. Each trial draws **one fixed map shift** with 4 m
standard deviation, plus independent **0.3 m** local geometric residuals per
unique spatial cell. Independent GNSS observations have **2 m** noise. Four
estimators process identical observations:

1. GNSS only;
2. Incorrectly independent 0.3 m centerline samples, ignoring shared error;
3. Independently inflating every map sample by the 4 m map variance;
4. A Schmidt map-error state, with fixed 4 m map uncertainty and tracked
   navigation/map cross covariance.

Reproduce with `python integration/imu_gnss/evaluate_map_correlation.py
--trials 80 --seconds 45 --output /tmp/map-correlation.json` and compare to
`integration/imu_gnss/map_correlation_evidence.json`.

| Approach | Lateral RMSE, m | Mean reported 1σ, m | Mean 1-D NEES | 95th percentile map update step, m |
|---|---:|---:|---:|---:|
| GNSS only | 1.057 | 1.066 | 0.98 | – |
| Naively independent map | **4.469** | **0.226** | **391** | 0.722 |
| Per-update inflated map noise | 1.124 | 0.977 | 1.32 | 0.613 |
| **Schmidt shared map offset** | **0.997** | **1.061** | **0.88** | **0.196** |

*These are only seeded toy-model results.* They demonstrate that pretending
shared map shift is new independent centerline evidence can make uncertainty
spectacularly overconfident. They **do not prove Schmidt improves real INS/GNSS
accuracy**: no real OSM or native sensor data are used, the geometry is linear,
the simulated map relationship is correctly specified and known to the
evaluator, the Monte Carlo sample count is modest, and the values depend on the
chosen model and noise.

The map model here is Gaussian on lateral position, not the finite-width
**interval occupancy** used by the opt-in realistic road corridor. Extending
correlation handling to the interval model requires the augmented-state
truncated-Gaussian moment formulas and approximation analysis; the Gaussian
reference is deliberately not marketed as that complete solution.

## Regressions and boundaries

- 26 local tests verify full augmented Joseph equivalence (18+2 example),
  cross-covariance propagation through linear dynamics, GNSS-like update and
  error reset, PSD, stable map identity, duplicate-cell no-op, invalid map
  changes and innovation-gate rejection. The native EKF is not invoked.
- Identical map geometry with 100 independent local residual cells cannot
  average away the fixed map-offset uncertainty; independently inflating each
  factor **does** collapse variance in the known analytical negative control.
- The simulator is deterministic for its fixed seed and runs without network.
  NEES values above are empirical across synthetic draws, not certified
  confidence intervals or guarantees.
- No claim of a smooth output under arbitrary wrong geometry, no route-exit
  association resolution, no lane-level information or independent road truth.

## Integration gates for a future native patch

Before enabling Schmidt map factors on the native 18-state estimator:

1. Expose/qualify actual IMU transition, GNSS update and error-reset Jacobians,
   including reanchor handling; test against an augmented reference.
2. Define stable map-patch identity and spatial correlation length using map
   provenance rather than edge IDs. Preserve map `Pmm`/`Pxm` through reuse and
   handoffs; fail closed on inconsistent map versions.
3. Use same IMU/GNSS epochs as the independent raw control and run paired
   native replays with/without map observations, including map bias sign tests,
   repeated stationary map lookups, sharp junctions, divided roads, missing
   map tiles and wrong-map injected offsets.
4. Compare displacement/step sizes, covariance, normalized innovations, NEES
   (only with independent ground truth or sound simulation truth), and factor
   availability at differing sample rates. Verify no raw-filter substitution.
5. Keep existing default map-free estimator and the published playback
   unchanged until exact-head native CI and replay qualification pass.

### Relevant literature

- Patrick Geneva and Guoquan Huang, *Map-based Visual-Inertial Localization:
  A Numerical Study*, ICRA 2022, https://pgeneva.com/downloads/papers/Geneva2022ICRA.pdf
- Zhuqing Zhang et al., *Toward Consistent and Efficient Map-Based Visual-
  Inertial Localization: Theory Framework and Filter Design*, IEEE T-RO 2023,
  https://arxiv.org/abs/2204.12108 . The observability cautions matter for
  real nonlinear/error-state navigation; the toy model is not an observability
  proof for the actual strapdown filter.
