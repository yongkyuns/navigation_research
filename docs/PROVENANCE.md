# Provenance and reproducibility

## Preserved originals

The conversation delivered a 433-file prototype ZIP, a standalone report, and an evaluation PNG. `provenance/original-artifacts.json` records their original identifiers. The repository preserves all 17 original source, test, documentation and configuration files with SHA-256 checks; it also preserves the standalone report byte-for-byte.

The exact original `metrics.json`, `summary.json`, `environment.json` and `tests.txt` are retained under `provenance/original-*`. Their checksums and source-file checksums are in `original-numerical-record.json`. The original runtime values are historical evidence, not measurements taken on a GitHub runner.

## Separately labelled reproduction

The first import attempt tried to reconstruct the entire original archive byte-for-byte by rerunning the simulation. That was unsuitable across different numerical-library CPU dispatch paths and rendering environments. The final publication does not loosen the original-byte check and pretend it passed: it distinguishes original records from new numerical reproduction.

The published `routefusion_prototype/results/` was generated from the unchanged original source. Navigation metrics in all 60 scenario/seed records and their summaries must match the preserved original within 1e-9 absolute/relative tolerance. Missing or duplicated cases, changed fields, non-finite numbers and changed navigation statuses are failures. Runtime is explicitly excluded from numerical equality and retained as a fresh measurement.

The published examples include six input event logs and tile caches, 60 trajectory CSVs, six replay outputs, metrics, a chart, environment information and fresh test logs. Each of the six 120-output replays is checked against its simulator trace. Subsequent reproduction also compares full replay records, including covariance and route weights, with the published baseline.

`provenance/IMPORT_VERIFICATION.json` records what was verified, the publication run and source commit, and the new archive/manifest hashes. The generated ZIP is a new research bundle; **it is not the original ZIP**, and the original archive hash is never used as its checksum.

The historical design/validation documents remain unchanged. Their references to experiment timing and original evidence describe the original study; the root README and the results README identify the later reproduced files.

## Exact-tie diagnostic exception

The deliberately symmetric scenario places parallel road centerlines at north=0 and north=26 m, with fallback fixes on the north=13 m bisector. Its naive nearest-road diagnostic makes a forced choice even though neither road is identifiable. Floating-point differences can switch that tie-breaking choice without changing the navigation estimate. For example, local cross-CPU testing changed seed 109's diagnostic rate from 0.95 to 0.9583333333333334; a GitHub runner also exhibited differences for other seeds.

Only `naive_correct_rate` in `ambiguous_parallel` can qualify for this exception. The verifier requires all 120 epochs, positions within 1e-10 m of the bisector, equal route weights within 1e-9, and `AMBIGUOUS` status with no published road. It recomputes the diagnostic rate from the trace and its summary from the qualified per-seed replacements. Each original/reproduced difference is included in the verification record. It does not exempt a physical positioning metric, a false latch, another scenario, or a changed navigation status. Two regression tests reject both an unqualified tie and a non-tied discrepancy.

This diagnostic is not valid evidence that nearest-road matching solved the ambiguity; the original validation report already identifies that limitation. The reference algorithm and its frozen results are not changed to make the diagnostic agree.

## Independent integrity and numerical checks

From the repository root:

```sh
python scripts/verify_snapshot.py
python scripts/reproduce.py --output runs/new-evaluation
```

The first command verifies original source/document/record hashes, every published project file, the original standalone report, chart and ZIP, and every ZIP entry. It also compares the published numerical metrics with the preserved original under the stated rules. The second command verifies the snapshot before and after a fresh evaluation; it refuses to overwrite existing runs or write inside the preserved project directory.

The suite contains 24 original algorithm tests and six publication checks. No algorithm or scenario tuning was changed for the repository import. The publication checks protect original record hashes, schema/case completeness, meaningful metric comparisons, finite values and narrowly qualified exact-tie behavior. These checks do not turn synthetic results into a physical-device qualification.

The one-time transfer used small checked binary parts to carry the original numerical records through the connector. These are transport artifacts, not executable data. Their decoded contents must match independently captured SHA-256 hashes. They are unnecessary after the original JSON/text files have been installed.

## Limits and exclusions

No actual M9/LSM6DSO recordings, UrbanNav datasets, surveyed city maps or licensed commercial data were supplied or evaluated. Original references and their access limits remain in the source ledger. Importing the report is not a new independent literature review.

The earlier generated urban-canyon infographic is not included because it predates the final architecture and can suggest a corrected-PVT feedback loop and capabilities not implemented here. The README and the final design document show the implemented architecture.

The shifted-map degradation, parallel-road ambiguity, delayed off-road rejection, conservative covariance and approximate branch pruning are retained. No third-party PDFs/maps are redistributed and no new project license is selected. Future field datasets and production implementations need separate provenance and qualification.
