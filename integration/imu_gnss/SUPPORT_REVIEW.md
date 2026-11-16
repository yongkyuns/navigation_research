# PR #9 review: strict endpoint joins

The first-stage support implementation accepted connected endpoint differences
up to 1e-6 m, then concatenated both coordinates. An edge ending at [50, 0]
followed by its explicit successor starting at [50, 1e-7] therefore passed the
join gate but failed while constructing a sub-minimum-length segment. Other
very small discrepancies could be silently simplified into a joined path.

The review fix requires identical endpoint coordinates. Nonidentical edges
remain separate, with their original geometry unchanged. It neither snaps the
map nor invents a connector. A missing outgoing map constraint suspends only
that measurement: the persistent corrected EKF retains its state and continues
processing sensors. Exact shared-node joins still work.

## Checks executed on the corrected source

- 69 support tests pass: the original 48, plus 21 endpoint review regressions.
- All 53 unchanged earlier artifact tests also pass: 122 tests total, no skips
  or failures. These are not a claim that the full repository CI ran locally.
- Running the new tests against the previous code reproduces 17 failures;
  the four larger-gap controls already pass there.
- All 56 native corrected replay combinations (four recordings, seven sigmas,
  two models) were rerun and then independently repeated. Every per-recording
  trajectory/diagnostic JSON is byte-identical both to its repeat and to the
  previously retained PR #9 evidence. Availability, map-step, and output counts
  in MAP_SUPPORT.md are unchanged for these recordings.
- All 37 pinned native source hashes verify. The existing original native
  library was reused; no fresh local Rust build or upstream Rust tests are
  claimed. Recorded IMU/GNSS input hashes verify, raw/control/map-disabled
  outputs remain identical, and skipped factors preserve the native snapshot.

The correction changes no measurement thresholds, covariance model, estimator
default, existing workflow or Pages publication file. The whole-recording
results are unchanged because the recorded map uses identical shared-node
coordinates. The added tests address malformed/inconsistent input geometry,
not an observed accuracy failure in those four recordings.

Machine-readable review evidence is in support-review-validation.json. New-head
CI must independently qualify this correction before merging. Earlier-head
results, queued jobs and local checks do not substitute for that CI gate.
