# Native `imu_gnss_fusion` integration

This experiment runs the actual Rust `SensorFusion` runtime, pinned to upstream
commit `8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6`, on its hosted recordings.
It adds alternative road-conditioned copies of that runtime, not a flattened
IMU adapter to the old planar simulation. Default publication remains the
unchanged map-free output; corrected candidates are experimental diagnostics.

Read [REPORT.md](REPORT.md) for results, including initialization availability,
a rejected timestamp stream, clean-run null results, and controlled GNSS
perturbations. **Reference discrepancy is not independently established absolute
accuracy.** Source metadata does not identify a surveyed truth system; iOS
recordings have no reference. Independent route labels are unavailable.

## Reproduce

From the repository root, with Rust 1.90.0 and Python 3.13:

```sh
git clone https://github.com/yongkyuns/imu_gnss_fusion.git .cache/fusion-upstream
git -C .cache/fusion-upstream checkout 8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6
python -m pip install -r requirements-reproduce.txt
python integration/imu_gnss/prepare_native.py \
  --upstream .cache/fusion-upstream --output .cache/fusion-native
cargo test --manifest-path .cache/fusion-native/Cargo.toml --workspace --locked
cargo build --manifest-path .cache/fusion-native/Cargo.toml \
  -p routefusion_native_bridge --release --locked
python integration/imu_gnss/run_evaluation.py \
  --upstream .cache/fusion-upstream \
  --library .cache/fusion-native/target/release/libroutefusion_native_bridge.so \
  --map-cache integration/imu_gnss/map_cache \
  --output runs/native-replay --report runs/native-report.md
```

On macOS use the built `.dylib`; on Windows use the `.dll` path. Results were
qualified on Linux, not those additional platforms. Output directories must be
new. Timing values are host dependent; floating-point threshold crossings can
affect road diagnostics. Bitwise cross-platform reproducibility is not promised.

## Integration boundary

`prepare_native.py` verifies upstream crate file hashes before copying it into
an isolated build. It adds facade cloning, an f64 anchor accessor, and a research
error-moment hook using the existing injection/reset routines. Propagation,
sensor processing, GNSS/NHC, alignment, gating and health policies are unchanged.
The upstream repository is not edited. These hooks are not production APIs.

`native.py` manages native lifetimes and complete-state clones. Every alternative
receives original sensor samples once in timestamp order; IMU precedes GNSS on
equal timestamps, matching upstream replay. GNSS remains receiver PVT, so
coupling stays loose. Reference streams never enter runtime inputs. No selected
branch is fed into the fallback.

`road_bank.py` conditions finite-width corridors in the full 18-state covariance,
using five persistent map-offset alternatives and a 24-branch cap. Its inference
is approximate: GNSS branch scoring is not the exact likelihood of upstream's
gated update, and retained weights are not integrity probabilities. No colored
GNSS-error augmentation, calibrated off-road mode, full smoothing, MCU memory
optimization or lane-level guarantee is claimed.

## Data and artifacts

Raw recordings are reused at the pinned upstream revision, not duplicated here.
Input checksums, row counts and headers are verified; duplicate/backward times
fail closed. The audit records rejected streams without inventing timestamps.
`evidence/` holds per-recording JSON, compressed epoch traces, graph tiles,
source/build/test provenance and checksums.

`map_cache/` holds an independently fetched OSM extract with query, date and
hash. Reproduction uses this exact snapshot, not today's roads. Coverage is
intentionally limited to four selected drives. Compilation handles shared-node
connectivity and basic one-way rules, not turn restrictions, conditional access
or surveyed lane widths/heights.

Two stress runs remove GNSS or alter its position at seconds 90–120 of an existing
recording. These are controlled perturbations, not additional natural recordings.

## Licensing and scope

Upstream sensor_fusion is MIT licensed; its license remains in the source checkout.
Road data © OpenStreetMap contributors, ODbL 1.0. See
https://www.openstreetmap.org/copyright and `map_cache/provenance.json`.
Map-data terms do not assign a new license to this research repository.
The full 3-D runtime is integrated, but the Python branch manager remains an
offline research component, not deployable IoT firmware.
