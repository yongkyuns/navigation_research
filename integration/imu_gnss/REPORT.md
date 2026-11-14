# Native fusion evaluation reports

The current maneuver-aware algorithm and its validation are documented in
[INTERSECTIONS.md](INTERSECTIONS.md). The published navigation output remains
map-free; map-corrected candidates are experimental.

[Initial native integration report](REPORT-initial-native.md) preserves the
original evaluation at `bc8cbb91b6efc82cadd0ce2c67769b93a07b9352`, including the
35-recording baseline, initialization gaps and independent-map experiments.
Its `evidence/` files have not been rewritten.

[Local intersection comparison](intersection-local-validation.json) records
unchanged baseline navigation and the stricter directed-edge/junction policy.
Latch availability decreased, and the previous single-outage coordinate benefit
was not retained. These are reference discrepancies, not independently verified
positioning or correct-road accuracy.

The read-only **Qualify native fusion and intersection handling** workflow
rebuilds the pinned upstream Rust code, runs the unit and recorded-data tests,
compares the baseline, and uploads a complete exact-head evaluation artifact.
Use the commands in [README.md](README.md) and [INTERSECTIONS.md](INTERSECTIONS.md)
to reproduce into a new run directory.
