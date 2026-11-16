# Native map-error correlation: event tracing stage (research only)

## Why this stage exists

PR #9 provides persistent map-aided native navigation and conservative 10 m
per-geometric-cell updates. PR #11 supplies a mathematically exact linear
Schmidt reference, but its map/nav cross covariance `C` cannot be carried across
ordinary native IMU and GNSS updates unless the real native error-coordinate
operators are used. A stale `C` can be numerically plausible yet unjustifiably
confident. Native GNSS fusion sometimes happens inside `process_imu`, and the
5 km navigation reanchor rotates the attitude, velocity, and position error
blocks. A Python-derived approximate transition is not an acceptable shortcut.

This change adds an **isolated, opt-in** native bridge experiment that records
those internal operators without changing the original `prepare_native.py` or
its default binary, or changing `supported_map.py`, `road_bank.py`, main EKF,
recovery, or any Pages output.

## Architecture

The standard pinned native runtime builds with `prepare_native.py` as before.
The new `prepare_native_transition.py` calls that same verified source-copy
procedure, then patches **only that temporary copied source**, checking each
source anchor exactly. It adds `ResearchTrace` to the copied Rust `Filter` and
exports the new `rf_transition` C ABI from the copied research bridge.

The trace computes a left-ordered product `J_event` of the actual operators:

- IMU prediction: `F` from `compute_error_transition_with_noise`.
- Accepted scalar and batch sensor rows: `I - K H`, from that row's native
  Jacobian and actual Kalman gain. GNSS rows fused during an IMU event are
  included rather than misattributed to the next GNSS event.
- Error-state injection: native attitude reset `G(dx)`.
- Reanchoring: rotation of attitude, velocity and position blocks using the
  very matrix `r_n1_n0` that rotates the pinned native covariance.
- Mount covariance freeze: project frozen rows to zero.

Special scaled-mount GNSS batch updates are marked **invalid**, because the
pinned routine can update covariance with an unscaled gain while injecting a
scaled mean. Such an epoch is not silently treated as an exact state-response
Jacobian. A filter reset/reseed also emits an invalid transition. Callers must
consume one trace after **each** native IMU or GNSS sample and immediately after
research moment injection. A repeated call returns an identity trace with zero
operators, not the last event's Jacobian.

`native_schmidt.py` is an opt-in bridge-side experimental adapter. It owns one
fixed 2-D map error covariance `U` and the 18x2 navigation/map cross block `C`.
Only Gaussian signed lateral map factors are supported. Its scalar update uses

```
S  = H P Hᵀ + H C Jᵀ + J Cᵀ Hᵀ + J U Jᵀ + R_local
Kx = (P Hᵀ + C Jᵀ) / S
P+ = P - (P Hᵀ + C Jᵀ)(P Hᵀ + C Jᵀ)ᵀ / S
C+ = C - Kx (H C + J U)
U+ = U
```

The native `rf_moments` applies the actual correction to its original 18-state
mean and covariance and performs the native error reset. The adapter consumes
the reset transition and transforms `C` accordingly. Future IMU/GNSS events
use the recorded `J_event` to propagate `C` at **each sensor epoch**, not at
1 Hz. An untraced reset or invalid operator aborts the map-enabled replay;
there is no fallback to the uncorrected estimator masquerading as successful
correlated navigation.

A canonical map-patch identity and unique geometric-cell key are mandatory.
A different patch cannot be introduced as an independent source of information
by simply renaming an edge. Rejected factors, duplicate-cell calls, numerical
failures and unknown patch IDs cannot silently mutate the cross covariance.

## Current evidence and qualification boundary

The local adapter/mathematical suite passes **51 tests**: 25 new adapter/guard
cases, plus PR #11's existing 26 mathematical tests. The tests check 18+2
augmented Joseph equivalence, cross covariance through sensor/IMU/reset
operators, map-patch identity, duplicate-cell no-op, covariance PSD, and
fail-closed behavior on reseed. These tests use **a simulated native transition
provider**. They do **not** prove the Rust patch compiles or that native
instrumentation sees every production update path.

An added exact-head GitHub Actions workflow must still:

1. Check out the original upstream revision and verify every pinned source hash.
2. Build both the uninstrumented and instrumented copies of the *same* Rust
   runtime with pinned Rust 1.90, and run the original Rust test suite and
   Rust `ResearchTrace` operator-order/reset/reanchor tests.
3. Replay an unmodified recorded drive through both copies, compare every
   native snapshot byte-for-byte when **no map update** is applied, and validate
   that the new trace contains actual native operators.
4. Inject one controlled Gaussian map factor into an independent native arm,
   then continue feeding every original IMU/GNSS event and verify the complete
   joint covariance, including GNSS updates that occur during IMU events.
5. Refuse, rather than conceal, a reset, a missing transition, an unsupported
   scaled gain, an indefinite joint covariance, or a version mismatch.

The previous binary is intentionally missing `rf_transition`. Asking the new
adapter to use it raises an explicit error; there is no fallback to guessed
Jacobians. The checked-in code is **not** yet an independently qualified
real-road filter, and the existing playback is not changed.

## Remaining engineering before enabling correlated map aiding

- **Native CI build and event audit are a hard gate.** Fix any actual compile or
  untraced native path before declaring completeness. Instrumenting only some
  EKF update paths would be worse than refusing the map factor.
- Decide whether to keep a research-only full 18x18 per-event trace, or move
  the much cheaper 18x2 cross-covariance update into common native Rust once
  its equivalence is demonstrated. The latter is the preferred production
  architecture; the former is primarily a transparent audit instrument.
- Add causal association of actual road/corridor geometry and stable map-patch
  provenance. This change injects **one controlled Gaussian factor** in CI and
  explicitly does not claim OSM accuracy or corridor-likelihood integration.
- Treat finite-width truncated-Gaussian corridor factors with an augmented
  correlated-moment model, not by substituting a Gaussian centerline factor and
  claiming equivalent road-occupancy semantics.
- Add independent truth or sound synthetic truth to assess NEES; compare
  different update rates, junctions, divided roads, map shifts and sensor gaps.
- Keep default map-free native behavior, existing source revisions, all
  workflows, recovery policy, and the published Pages site unchanged until
  exact-head CI demonstrates the intended opt-in behavior.

## Reproduction

```bash
export PYTHONPATH="$PWD/integration/imu_gnss"
python -m pytest -q integration/imu_gnss/schmidt_native_tests
python integration/imu_gnss/prepare_native.py \
  --upstream .cache/upstream --output .cache/native-control
python integration/imu_gnss/prepare_native_transition.py \
  --upstream .cache/upstream --output .cache/native-transition
cargo test --manifest-path .cache/native-transition/Cargo.toml --workspace --locked
cargo build --manifest-path .cache/native-transition/Cargo.toml --release --locked -p routefusion_native_bridge
python integration/imu_gnss/verify_native_transition.py \
  --control-library .cache/native-control/target/release/libroutefusion_native_bridge.so \
  --traced-library .cache/native-transition/target/release/libroutefusion_native_bridge.so \
  --dataset .cache/upstream/web/datasets/urban-short-turn-loop-nominal-001 \
  --output runs/native-transition/replay.json
```

This stage is isolated from PR #11 until both independent exact-head CI gates
are qualified. Neither the research source patch nor the adapter is selected
by existing runtime or default navigation configuration.
