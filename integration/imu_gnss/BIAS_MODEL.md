# Selective receiver-position fault modelling

## Status and scope

This is an **opt-in research extension**, not a change to production/default navigation. The original `prepare_native.py`, `native.Library`, native source repository and historical evidence are unchanged. `prepare_bias_native.py` builds a separate, hash-checked copy of the pinned upstream runtime with three receiver-position nuisance states. `BiasLibrary` supplies the optional mode detector and complete clone/replay state. The road corridors, intersection rules and latch thresholds are unchanged.

The target failure is a new position offset that disagrees with both observed receiver velocity and the inertial/navigation prediction. It is **not a general solution to all urban GNSS errors**. In particular, a bias present from startup, sufficiently slow drift, or compatible position/velocity corruption may not be distinguishable with this policy.

## Why not simply tighten road constraints?

The preceding map-only implementation retained almost all of a persistent 20 m GNSS position fault. Tightening the road to a centerline would pretend that the map is a surveyed lane and could reinforce a wrong association. Instead this experiment permits receiver error to persist separately from physical position. Original sensor observations are consumed once per alternative; derived residuals select a model but are never fused as additional sensor measurements.

An always-on bias model was tested first. On the development short-loop-002 drive, its clean full-trip discrepancy worsened from about 0.608 m to 1.903 m; its injected-outage window worsened from 2.927 m to 4.320 m, despite improving the injected position-bias window from 17.506 m to 2.444 m. These negative trials motivated selective activation, not hidden removal of unfavourable results. No always-on prior is enabled by default.

## Joint model

Retain the native 18-error-state vector `delta x`, and append the three-dimensional NED receiver-position error `b`:

```
z_position = p + b + noise
z_velocity = v + noise
P_joint = [[P_x, C_xb], [C_xb^T, B]]
```

The receiver error has first-order Gauss–Markov dynamics:

```
phi = exp(-dt/tau)
b_next = phi b + eta
Q_b = sigma_stationary^2 * (1 - exp(-2 dt/tau)) I
C_next = F C phi
B_next = phi^2 B + Q_b
```

The native generated inertial transition `F`, raw-body IMU mechanization, mounting state, and health logic are retained. All relevant GNSS, NHC, zero-velocity and scalar updates propagate the cross-covariance, not just the three new diagonal variances. Nuisance mean/covariance and both sides of the cross-covariance rotate when the NED anchor changes. Disabled mode uses the original observation implementation.

For a scalar row with native Jacobian `h` and nuisance Jacobian `h_b`:

```
u_x = P_x h^T + C h_b^T
u_b = C^T h^T + B h_b^T
S = h u_x + h_b u_b + R
K = [u_x; u_b] / S
```

Only GNSS position rows directly observe `b`. Native gain masking is handled with the Joseph-equivalent joint covariance update. The code retains the inherited attitude-only reset convention; this work does not resolve the existing mount-reset limitation.

### Road interval update

The road still supplies its finite-width interval, not a snapped GNSS fix. If its scalar predicted variance is `S`, conditional interval mean is `mu_T`, and conditional interval variance is `V_T`, define

```
u = [P_x h^T; C^T h^T]
alpha = (mu_T - mu) / S
beta = (V_T - S) / S^2
error_mean = alpha u
P_joint_plus = P_joint + beta u u^T
```

The map observes physical position, not receiver error directly. Cross-covariance transfers map information into the bias estimate. Overwriting only the original 18-state marginal would discard that correlation, so arbitrary marginal injection is rejected while bias mode is active. Native attitude injection/reset follows the joint update. Clone ownership and rollback comparisons include nuisance states, detector history and pending observations.

## Selective activation

The detector uses at most 32 recent original GNSS rows within two seconds. Its diagnostic is

```
d = (p_last - p_first) - integral(v_GNSS dt).
```

Positions and velocities are expressed in one local frame. The trapezoidal integral is a diagnostic only. Its engineering variance doubles endpoint position variances and treats short-window velocity uncertainty conservatively as fully correlated; a small motion/discretization allowance is added. This is not a calibrated receiver correlation model.

Activation requires all of:

- usable initialized navigation, at least two previous fixes, and at least 0.9 s diagnostic span;
- displacement disagreement at least 1.5 m and squared normalized disagreement above 16;
- horizontal navigation position innovation above 9, while horizontal velocity innovation remains below 16.

A GNSS gap above 1.1 s clears the diagnostic window. No reference trajectory, injected-fault timestamp, known fault magnitude, road centerline or future observation enters the detector. A map-conditioned branch's native prediction can nevertheless influence its navigation-innovation test; the detector is not independent of its branch history.

When selected, create a zero-mean receiver-error prior with 20 m standard deviation and zero cross-covariance, **without moving or shrinking the physical navigation prior**. Thereafter the joint model uses `tau=90 s` and stationary sigma 15 m. These settings are research assumptions, not measured M9 specifications. The always-on alternative uses a 3 m initial bias prior with the appropriate negative position/bias correlation, because initialization places physical position at the GNSS fix.

Selection uses the same data whose likelihood is subsequently evaluated. Therefore the combined mode-selection/road-weighting system is approximate, not an exact Bayesian change-point filter or integrity solution. The bias mode stays active after onset; it is not erased when the offset stops changing. Automatic return to the original noise model remains future work.

## Frozen evaluation and local findings

`bias_config.json` was frozen after development on short-loop-002 and before running other drives or amplitude/rate sensitivity cases. Configuration SHA-256:

`b3b00ebbb82b513f629c692e68b67096b3018eb5b25a6ec9cf60978c4e617a55`

These are additional experiments on previously available recordings, not independent new field collections. All distances are discrepancies from supplied reference streams of unverified independence. They are not surveyed accuracy or M9/LSM6DSO hardware guarantees. Reference files are loaded only after navigation replay.

Local execution completed 68 Rust/upstream tests, 53 Python tests, 35 unmodified recording replays, three development cases, 21 holdout-drive/ablation cases, and four amplitude/rate sensitivity cases. Native CI independently rebuilds and replays all of these; its complete logs, source hashes, trajectories, covariance checks and reports are retained in a commit-specific artifact.

### Unmodified recordings

All 35 accepted recordings produced no map-free fault-mode activations. Their sampled native outputs and reference discrepancies stayed identical to the unaugmented runtime. This includes 3,587,141 IMU and 48,920 GNSS rows. The original duplicate-timestamp package is still rejected, and the same two stationary smoke clips still do not initialize. No false-alarm probability is inferred from this finite collection.

### New position-only faults

The held-out fault family consists of 20 m north/east/south offsets, each ramped over five seconds, on three other recordings. During the configured fault windows, the **mean of nine per-case RMSE values** was:

| Variant | Mean reference discrepancy |
|---|---:|
| Original INS/GNSS | 17.603 m |
| Bias-aware INS/GNSS, without maps | 1.159 m |
| Bias-aware navigation plus unchanged map constraints | 1.155 m |

Every one of the nine cases improved, with no change in paired initialized-epoch coverage. Detection occurred about 1.3–1.5 s after fault onset. The important ablation is that **nearly all of this improvement comes from receiver-fault modelling, not road geometry**.

On the development 20 m north fault, the corresponding fault-window results were 17.506 m, 1.764 m and 1.733 m. Removing map coverage retained the 1.764 m result, confirming that the receiver-error improvement does not depend on a map snap.

### Sensitivity and failure cases

With unchanged settings on a different part of the medium loop, a 10 m / two-second-ramp fault improved from 9.159 m to 1.601 m, and a 35 m / ten-second-ramp east fault from 30.569 m to 1.374 m. A 5 m fault did not activate (about 4.22 m remains), and a 20 m / twenty-second-ramp fault did not activate (about 14.53 m remains).

Cold-start translation and three coordinated position/velocity faults did not activate the map-free bias model. Their substantial errors remain. Outage-only trials did not activate it either, preserving the original map-free state; inherited map feedback can still make those discrepancies worse. After a detected fault ends, recovery is not uniformly better: one stop/go north-fault recovery window worsened from about 1.775 m to 2.176 m. Full-trip and recovery-window metrics must be read alongside the fault-window gains.

No negative joint-covariance eigenvalues were found at the sampled local evaluation epochs. The map-free bias-aware base matched a separately replayed bias-aware filter exactly, including receiver-error states and detector history. This is a consistency check, not calibration of the posterior covariance or correct-road probability.

## Resource and deployment limits

The augmented native instance occupies 15,160 bytes on the evaluated host versus 14,880 bytes previously: 280 additional native bytes per clone. This excludes the detector's bounded history, Python objects, maps, replay buffers, allocation and stacks. Full-runtime branching remains a research design, not a RAM-optimized MCU implementation. No additional cellular correction stream is required by this change; target timing, energy and total-memory qualification have not been performed.

Keep the default raw/native path and existing shadow-output policy. Opt into the new adapter only for engineering replay. Natural urban-canyon benefit, slow/coupled faults, confidence calibration, automatic fault-mode recovery, independent route labels and map alignment still need separate validation.

## Reproduce

From the repository root, with the pinned upstream checkout at `.cache/upstream`:

```sh
python -m pip install -r requirements-reproduce.txt
python integration/imu_gnss/prepare_bias_native.py --upstream .cache/upstream --output .cache/bias-native
cargo test --manifest-path .cache/bias-native/Cargo.toml --workspace --locked
cargo build --manifest-path .cache/bias-native/Cargo.toml --release --locked -p routefusion_native_bridge
export PYTHONPATH="$PWD/routefusion_prototype:$PWD/integration/imu_gnss"
export ROUTEFUSION_NATIVE_LIBRARY="$PWD/.cache/bias-native/target/release/libroutefusion_native_bridge.so"
export ROUTEFUSION_BIAS_LIBRARY="$ROUTEFUSION_NATIVE_LIBRARY"
export ROUTEFUSION_DATASETS="$PWD/.cache/upstream/web/datasets"
mkdir -p runs/bias
python -m pytest -q integration/imu_gnss/tests integration/imu_gnss/bias_tests --junitxml=runs/bias/python-tests.xml
python integration/imu_gnss/evaluate_bias_clean.py --library "$ROUTEFUSION_NATIVE_LIBRARY" --datasets "$ROUTEFUSION_DATASETS" --output runs/bias/clean
for suite in development holdout sensitivity; do
  python integration/imu_gnss/evaluate_bias.py --library "$ROUTEFUSION_NATIVE_LIBRARY" --datasets "$ROUTEFUSION_DATASETS" --map-cache integration/imu_gnss/map_cache --cases "integration/imu_gnss/bias_cases_${suite}.json" --output "runs/bias/$suite"
done
python integration/imu_gnss/qualify_bias.py --results runs/bias
```

Use Rust 1.90.0 for the captured build. The shared-library filename above is Linux-specific. Python/native interfaces and cases are portable research code, but no other target binary is qualified here. Output directories must be new; historical evidence is not overwritten.

### Context

The use of temporally correlated GNSS errors as nuisance states is consistent with the modelling problem discussed by Crespillo, Joerger and Langel, *Overbounding GNSS/INS Integration with Uncertain GNSS Gauss-Markov Error Parameters*, ION 2020: https://www.ion.org/publications/abstract.cfm?articleID=17391. This implementation does not reproduce that paper's integrity bounds or claim that its empirical mode selector is an overbounding solution.
