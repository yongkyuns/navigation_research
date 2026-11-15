# Recovery merge review: partial map coverage

## Defect reproduced

Before this review, `RelocalizingRoadBank` checked that the receiver tile was loaded but inferred the absence of competing routes from `TileStore.candidates()`, which omits unloaded tiles. In a deterministic fixture with parallel roads at map north 230 m and 270 m, and a receiver at 250 m, complete coverage retained both roads with weights near 0.5 and no latch. Evicting the northern neighbor tile, while keeping the receiver tile, incorrectly produced a single-road latch with weight 1.0 after recovery.

This is a data-availability error, not evidence about which road the receiver occupies. The deterministic fixture is a policy regression, not a new field-accuracy measurement.

## Correction

Before declaring that every plausible road rejects the local estimate, require the entire automatic candidate-search square to have loaded coverage, including known-empty tiles. For a bounded OSM extract, also require the search square to lie within the projected extract footprint. A loaded boundary tile does not certify the portion outside the downloaded region. Repeat this check before constructing the replacement bank and clear pending loss confirmations when coverage is incomplete.

The local footprint check is conservative and does not replace the map compiler's obligation to provide accurate, versioned coverage metadata. It assumes the current small-area map frame, not arbitrary antimeridian-spanning maps.

Four new tests cover missing-neighbor false confidence, restoring both parallel alternatives, restarting confirmations after a cache gap, and a loaded tile straddling the extract boundary. The 70 prior checks are retained. No road width, covariance prior, activation threshold, GNSS observation, or default navigation path is retuned.

## Review and evidence

The original 70 tests and the four new tests pass locally. The paired recorded-input comparison is rerun using the same unaugmented native library and explicit case definitions. Its results and exact source hashes are retained in the commit-specific read-only CI artifact; check the workflow status rather than treating this note as a claim of completed CI.

PR #5 is already merged into main. The reviewed recovery branch incorporates that main revision and preserves `.mergify.yml`; it does not replace unrelated merge-queue configuration. Recovery remains opt-in and the published map-free filter remains independent. `POSITION_RECOVERY.md` records the original 70-test evaluation and scientific limitations; this note adds the merge-review correction.
