# Provenance and reproducibility

## Preserved originals

The conversation delivered a 433-file prototype ZIP, a standalone report, and an evaluation PNG. `provenance/original-artifacts.json` records their original identifiers. The repository preserves all 17 original source, test, documentation and configuration files with SHA-256 checks; it also preserves the standalone report byte-for-byte.

The exact original `metrics.json`, `summary.json`, `environment.json` and `tests.txt` are retained under `provenance/original-*`. Their checksums and source-file checksums are in `original-numerical-record.json`. The original runtime values are historical evidence, not measurements taken on a GitHub runner.

## Separately labelled reproduction

The first import attempt tried to reconstruct the entire original archive byte-for-byte by rerunning the simulation. That was unsuitable across different numerical-library CPU dispatch paths and rendering environments. The final publication does not loosen the original-byte check and pretend it passed: it distinguishes original records from new numerical reproduction.

The published `routefusion_prototype/results/` was generated from the unchanged original source. Every non-timing metric in all 60 scenario/seed records, and the non-timing summary, must match the preserved original within 1e-9 absolute/relative tolerance. Missing or duplicated cases, changed fields, non-finite numbers and changed categorical outputs are failures. Runtime is explicitly excluded from numerical equality and retained as a fresh measurement.

The published examples include six input event logs and tile caches, 60 trajectory CSVs, six replay outputs, metrics, a chart, environment information and fresh test logs. Each of the six 120-output replays is checked against its simulator trace. Subsequent reproduction also compares full replay records, including covariance and route weights, with the published baseline.

`provenance/IMPORT_VERIFICATION.json` records what was verified, the publication run and source commit, and the new archive/manifest hashes. The generated ZIP is a new research bundle; **it is not the original ZIP**, and the original archive hash is never used as its checksum.

The historical design/validation documents remain unchanged. Their references to experiment timing and original evidence describe the original study; the root README and the results README identify the later reproduced files.

## Independent integrity and numerical checks

From the repository root:

```sh
python scripts/verify_snapshot.py
python scripts/reproduce.py --output runs/new-evaluation
```

The first command verifies original source/document/record hashes, every published project file, the original standalone report, chart and ZIP, and every ZIP entry. It also compares the published non-timing metrics against the original records. The second command verifies the snapshot before and after a fresh evaluation; it refuses to overwrite existing runs or write inside the preserved project directory.

The suite contains 24 original algorithm tests and four publication checks. No algorithm or scenario tuning was changed for the repository import. The publication checks protect original record hashes, schema/case completeness, meaningful metric comparisons and categorical/finite-value consistency. These checks do not turn synthetic results into a physical-device qualification.

The one-time transfer used small checked binary parts to carry the original numerical records through the connector. These are transport artifacts, not executable data. Their decoded contents must match the independently captured SHA-256 hashes. They are unnecessary after the original JSON/text files have been installed.

## Limits and exclusions

No actual M9/LSM6DSO recordings, UrbanNav datasets, surveyed city maps or licensed commercial data were supplied or evaluated. Original references and their access limits remain in the source ledger. Importing the report is not a new independent literature review.

The earlier generated urban-canyon infographic is not included because it predates the final architecture and can suggest a corrected-PVT feedback loop and capabilities not implemented here. The README and the final design document show the implemented architecture.

The shifted-map degradation, parallel-road ambiguity, delayed off-road rejection, conservative covariance and approximate branch pruning are retained. No third-party PDFs/maps are redistributed and no new project license is selected. Future field datasets and production implementations need separate provenance and qualification.
