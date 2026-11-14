# RouteFusion: tiled-road-aided loosely coupled navigation

A runnable **research reference**, written for an existing INS/GNSS implementation using consumer IMU and receiver PVT. The recommendation is a bounded bank of alternative road-conditioned navigation filters, finite-width road constraints, persistent error states, and a separate map-free fallback. It is not a snap-to-road feedback loop.

## Read first

- `docs/DESIGN.md`: research synthesis, statistical formulation, architecture, approximations, and deployment recommendation.
- `docs/INTEGRATION.md`: full-ESKF adapter, timestamp/frame contract, input format, and tiled graph requirements.
- `docs/VALIDATION.md`: what was executed, positive and negative cases, and what remains unvalidated.
- `sources.json`: primary-source ledger with original URLs and inspection limits.

## Reproduce

Python 3.10 or newer, NumPy, SciPy; pytest for tests. Commands run from this directory:

```sh
python -m venv .venv
# Linux/macOS:
. .venv/bin/activate
# Windows PowerShell instead: .venv\Scripts\Activate.ps1
python -m pip install -e '.[test,plots]'
python -m pytest -q

# 6 scenarios x 10 seeds x 120 seconds; overwrite the supplied result files:
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m routefusion.simulate \
  --seeds 10 --start-seed 100 --save-traces --output results

# Replay an included chronological input log, without consulting truth:
python -m routefusion.replay results/parallel_bias_example \
  --output replay.jsonl
```

On Windows, omit the POSIX environment-variable prefixes or set equivalent environment variables separately. Single-threaded BLAS is a reproducibility/runtime convenience, not a mathematical requirement.

## Delivered and executed

24 tests passed. The final evaluation used seeds 100–109, six synthetic scenarios, and 7,200 map epochs. All 120 replay outputs for the included parallel-bias example matched the simulation output. These are **synthetic results**, not M9/LSM6DSO field measurements. Noise values are illustrative, not measured sensor specifications.

The implementation contains a 14-state planar inertial filter, robust PVT likelihood, persistent map offset, GNSS position/velocity colored biases, directed road graph, JSON+gzip tile cache, bounded route branches, covariance-aware corridor conditioning, abstention, test suite, simulator, and chronological replay adapter.

**Important negative results:** a 6 m shifted map worsened mean horizontal RMSE from 2.12 m to 4.01 m despite correct road identification. Leaving the mapped road retained a stale latch for 3 seconds before rejection. Symmetric parallel roads correctly remained ambiguous. Confidence values are conditional weights, not calibrated integrity probabilities.

## Deliberately not claimed

This is not a complete 3-D strapdown INS, Rust/MCU firmware, UBX binary decoder, raw OSM parser, surveyed city-map deployment, calibrated off-road Bayesian mode, full fixed-lag smoother, ray tracer, or field-qualified navigation system. It does not certify a safe wrong-road probability. The Python input IMU is planar, mounting-corrected and gravity-compensated; do not feed raw six-axis samples into it unchanged.

A frozen evaluation is bundled in `results/`. Development results were not packaged. The baseline uses the same augmented robust navigation model with map updates disabled; it is not a measurement of the user's existing production filter.

## Code layout

| Module | Responsibility |
|---|---|
| `filter.py` | Planar propagation, normalized robust PVT mixture, interval moments |
| `tracker.py` | Alternative route branches, topology, policy and fallback |
| `tiles.py` | Versioned local-frame graph tiles, canonical IDs and coverage |
| `build_tiles.py` | Already-directed GeoJSON graph to local EN tiles |
| `replay.py` | Input event contract and reproducible navigation replay |
| `simulate.py` | Six adversarial synthetic scenarios and measurements |

For a real deployment, retain the existing full INS mechanization and replace only the planar adapter. Begin in output-only/shadow mode; qualify geometric feedback separately from road-label accuracy.
