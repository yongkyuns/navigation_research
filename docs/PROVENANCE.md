# Research provenance and reproduction

## What belongs to the original result

The initial chat deliverables were `routefusion_prototype.zip`, the standalone `routefusion_research_and_design.md`, and the evaluation chart. The ZIP contains 433 files: 17 source/documentation/configuration files, 415 result files, and a SHA-256 manifest covering the other 432 files. The repository preserves those project bytes without changing the algorithm, tunings, tests, or recorded results.

The research result is synthetic: six scenarios, seeds 100–109, 120 epochs each. No real M9/LSM6DSO recordings, UrbanNav data, surveyed city tiles, or licensed third-party datasets were provided. The source ledger records the original research session's references and inspection limits; importing the files is not a new independent literature review.

## Integrity records

`provenance/original-artifacts.json` records hashes captured from the original mounted artifacts, plus the original 60 runtime values and environment/test-log text. These timing values are historical and must not be represented as timings measured on a GitHub runner.

| Artifact | SHA-256 |
|---|---|
| Original ZIP container | `018e07ba7b71eddd14fe44375dd4ff8339696355ee3fdc7e480065ab4d748d59` |
| Original project SHA256SUMS.txt | `266dfa440176d4114b2e58de5cd5419702604fa24a44e2444faf39a93d619adf` |
| Standalone report | `e06d9c95ef3560a988575232f874d372cbaccef0d11f0ec50337f758ee0a0872` |
| Evaluation PNG | `4fca1a2f6232e49d36833a50a61fb85b4c23423d98aa04942f802f3dbcd3aecf` |

## Import mechanism

The connector supports text Git objects, but not directly forwarding this local multi-file archive. Source and documentation were imported as text Git objects. `scripts/restore_original_evidence.py` reconstructs the deterministic simulation inputs, traces, tiles, replay and figure; restores explicitly captured historical metadata; and requires the entire aggregate manifest to match the original hash **before** installing the snapshot. It then separately checks the standalone report hash. An approximate numerical match is not sufficient for this import.

`provenance/IMPORT_VERIFICATION.json` is written only after those checks pass. It records the actual repacked archive hash. ZIP entry timestamps/packaging differ from the original archive, but every contained project file is verified against the original manifest. The original ZIP hash must not be used as the hash of the repacked ZIP.

Fresh test logs, simulation timings and environment details remain separate from the frozen snapshot. The import job uploads its fresh verification output as an Actions artifact; normal reproduction writes to a new directory under `runs/`.

The import helper intentionally refuses to replace existing `routefusion_prototype/results/`. It is an archival reconstruction tool, not the command for new research.

## Safe ongoing reproduction

From the repository root after installing dependencies:

```sh
python scripts/verify_snapshot.py
python scripts/reproduce.py --output runs/new-evaluation
```

The verifier checks every frozen project file, report, PNG, archive hash, and all ZIP entries. The reproduction command refuses to overwrite an existing run or write inside the frozen project. It reruns the 24 tests and 60 synthetic runs, compares non-timing metrics at relative/absolute tolerance 1e-9, checks replay, and renders a new chart. New runtime measurements are retained separately. A numerical tolerance for reruns is distinct from the exact-byte archival check.

The original project README is unchanged and includes an old command that overwrites `results/`; use the root README's safe commands instead. Additional ignored Python caches are not original evidence and are not packed into the archive.

## Retained limitations and exclusions

The shifted-map degradation, three-second stale latch after off-road departure, unresolved parallel roads, conservative covariance, and approximate branch pruning remain visible. None of these are silently fixed or omitted by the import.

The earlier generated urban-canyon infographic is excluded because it predates the final architecture and can misleadingly suggest corrected-PVT feedback and capabilities not implemented in this prototype. The final Mermaid architecture in the README and original design document is authoritative for this research snapshot.

Only reference URLs and research summaries are included, not redistributions of third-party papers or map extracts. This import does not select or grant a new project license. Future production implementations and hardware datasets should have their own provenance, usage permissions, and validation records.
