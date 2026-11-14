# Recorded-data experiment results

These are executed replay diagnostics, **not absolute-accuracy or correct-road measurements**. See ../README.md.

| Dataset | Mode | Epochs | Latched | P95 / max map correction (m) | Receiver agreement baseline / output RMS (m) |
|---|---|---:|---:|---:|---:|
| urban-short-turn-loop-nominal-001 | original | 264 | 56.1% | 7.90 / 8.08 | 3.69 / 3.45 |
| urban-short-turn-loop-nominal-001 | withheld_gnss_30s | 264 | 54.9% | 19.17 / 21.26 | 4.58 / 5.24 |
| urban-short-turn-loop-nominal-001 | missing_map_30s | 264 | 46.2% | 4.67 / 5.56 | 3.69 / 3.27 |
| ios-mixed-drive-reverse-parking-001 | original | 2494 | 54.0% | 25.95 / 95.76 | 18.58 / 19.73 |
| ios-drive-20260521-130835 | original | 183 | 69.4% | 10.89 / 11.43 | 4.56 / 4.68 |
| ios-drive-20260521-130835 | withheld_gnss_30s | 183 | 69.9% | 5.61 / 6.57 | 23.76 / 24.56 |
| ios-drive-20260521-130835 | missing_map_30s | 183 | 35.5% | 9.16 / 10.16 | 4.56 / 3.93 |
| ios-drive-20260521-131420 | original | 1014 | 0.0% | 0.00 / 0.00 | 4.22 / 4.22 |

ios-drive-20260521-131420: completed_without_map 

A higher latch rate or smaller distance to a map is not proof of better positioning. Reference-track comparison is agreement with another fused estimate, not surveyed truth.

Maps: © OpenStreetMap contributors; Open Database License (ODbL) 1.0; https://www.openstreetmap.org/copyright

Input source commit: `8a34212c7befe30f8d3eae4ad708cc0dcd4aa0d6`. Full diagnostics and checksums: `summary.json`.
