# Existing automatic-recovery failures: diagnostic-only review

## Scope and evidence

This is a post-hoc examination of the existing PR #7 cases, not a new navigation
experiment, parameter search or estimator change. The original 18-state runtime,
map-only recovery policy, road widths, confidence thresholds, default map-free
outputs and all existing workflows remain untouched.

The input is the retained **post-coverage-fix** `map_only_recovery_review.zip`:

- SHA-256: `71218d670f6caefcec42935329d89f8bf3b63122d88734a14dee308e36485c7d`.
- `review/review-summary.json` identifies reviewed head
  `4549f074429d23518114ebb537a64a40accbf18c`.
- Merged baseline: `d5e67976b0d89753edfde39d9576efa7354ba8a3`; its tree
  `f550f7989cc5590cda9d6bca92787c21ebbe729a` equals the reviewed head's tree.
- The archived evaluator was independently matched to current Git blob
  `d58d9c4c711a4e91474eb30d58bdde944384f71f`, and its case file to blob
  `f6643d7947ba70da748a22ab3131cca54ac0bd6f`.
- The qualification record describes local execution, not a new CI run. It records
  the post-fix recovery source SHA-256
  `e48dff39418f4f266499f65b53aa8f34cfb0699992fe58ddee61466848a8a073`.

The earlier `map_only_position_recovery_evidence.zip` was used only to recover the
byte-matched evaluator and case file, not as the source of post-fix traces. Its
older recovery implementation was neither imported nor executed.

All times below are elapsed since each case checkpoint. “Selected road” means a
selected branch whose directed edge belongs to the **supplied development
itinerary**, not independently verified road truth. “Inside” means the selected
native state lies within the finite itinerary corridor under the existing 1 mm
numerical tolerance. It is not a mixture mean or a good fallback coordinate.

## The five misses are not five wrong-road selections

The original metric requires entry before the scheduled 10 s withholding
boundary followed by five sampled seconds of accepted native-state retention.
Its endpoint must itself be an accepted sample. The deadline is not the timestamp
of the first subsequently delivered GNSS message.

| Fixture | Offset | Supplied road first selected | First accepted native position inside | First qualifying five-second hold starts | Observed failure |
|---|---:|---:|---:|---:|---|
| short2-turn | -10 m | 3.015198 s | 10.050660 s | 10.050660 s | Corridor entry is after the cutoff. |
| short2-turn | +10 m | 3.015198 s | 10.050660 s | 10.050660 s | Corridor entry is after the cutoff. |
| medium-route | -10 m | 2.010124 s | 9.045558 s | 33.167047 s | Early entry is interrupted by junction abstention. |
| medium-route | +10 m | 2.010124 s | 9.849608 s | 33.167047 s | Early entry is interrupted by junction abstention. |
| short1-route | +20 m | 5.025262 s | 5.025262 s | None in the evaluated window | Recovery succeeds locally, but accepted retention is interrupted. |

All five have zero sampled selections outside the supplied itinerary. This does
**not** prove that all unselected alternatives were incorrect or that the
supplied route is physical ground truth. It does show why calling these cases
“failed road selection” is too broad.

### Moderate offsets: slow correction, not activation of recovery

None of the six +/-10 m cases activates the recovery prior. Their complete
original/recovery trajectory CSVs are byte-identical, including the two passing
cases. The recovery extension therefore did not cause the four moderate-offset
misses and did not change their outcomes.

In the two `short2-turn` misses, the supplied road is selected at 3.015198 s,
but the native position is still outside its corridor at the last pre-boundary
sample, 9.950154 s: by **0.077744 m** for -10 m and **0.017589 m** for +10 m.
The first accepted inside sample occurs at 10.050660 s. Do not attribute that
entry to GNSS return: this trace does not log the exact first admitted GNSS event.
It is enough to establish that the original fixed deadline was missed.

The initial supplied-route factor log probabilities in the four moderate misses
are approximately -5.45, above the unchanged -12 rejection threshold. The
supervisor deliberately tests severe map inconsistency, not whether an otherwise
plausible road candidate converges inside a chosen deadline. These initial
same-state factor diagnostics and the recorded absence of activation are
consistent with that design. Subsequent supplied-route factor rows are **not**
automatic-bank gate observations.

For `medium-route`, both signs select the supplied road early and enter the
corridor before 10 s. Their first accepted runs last only **3.919742 sampled
seconds** (-10 m) and **3.115692 sampled seconds** (+10 m). At 13.065807 s the
reported state becomes `JUNCTION_AMBIGUOUS`. Subsequent accepted intervals also
fail to provide a qualifying early hold; the first full hold begins at 33.167047 s.
This is a retention failure, not an absence of initial correction.

### Large-offset exception: preserve the sampled-endpoint rule

`short1-route`, +20 m activates recovery at **2.010105 s** and selects an inside
native state at **5.025262 s**. Accepted samples continue through **9.950020 s**:
that is **4.924757 s** of observed accepted-sample span. The next sample, at
**10.050525 s**, is `JUNCTION_AMBIGUOUS` and does not supply a publishable branch.

The nominal five-second endpoint is 10.025262 s. There is no accepted sample at
or after that endpoint before abstention. Treating the previous sample as held
until the next rejected timestamp would yield a duration over five seconds, but
that is a different criterion from the original sampled-hold test. No interpolation,
rounding, shortened hold or reclassification was used to turn this case into a pass.

This establishes an observed junction-policy refusal. It does not establish that
the refusal was unavoidable from all available branch information; those branch
states and rejection reasons were not retained.

## Why +/-20 m improves while +/-10 m does not

The larger errors trigger the explicit localization-loss hypothesis; moderate
ones retain the normal plausible-road candidates. That difference is an intentional
consequence of the bounded recovery policy, not evidence that larger errors are
intrinsically easier to estimate. Increasing the compatibility threshold would
change that policy; it is not a diagnostic-only correction.

The automatic bank also retains map-offset alternatives, and the evaluator
reports one highest-weight native branch rather than the mixture. However, the
retained traces omit the selected offset, per-branch weight/covariance and precise
junction-phase/gate reasons. They cannot isolate how much of the slower moderate
correction comes from offset selection, covariance, reuse of map cells or branch
switching. No causal attribution among those mechanisms is claimed here.

## Checks actually executed for this diagnosis

`diagnose_recovery.py` imports only the Python standard library. It reads the
existing qualification, paired summaries, per-case CSVs and clean comparisons;
it never imports or invokes the estimator. It preserves the original metric
configuration and rejects missing/duplicate cases, mismatched retained metrics,
changed paired map-free outputs, changed clean logged fields and input-file changes.

The diagnostic run:

- Independently recomputed and matched **54 automatic metric sets** (27 cases,
  original and recovery arms), then cross-checked them with the exact original
  evaluator metric functions extracted without importing the native runtime.
- Rechecked paired logged map-free outputs and the original clean comparison
  fields at **1,569 epochs across four recordings**; all match.
- Passed **21 new offline regression tests**, including duplicate-timestamp
  ownership, late coordinate entry, interrupted retention, malformed data and the
  non-interpolated five-second endpoint. All input evidence hashes remained unchanged.

The aggregate results remain **2/6 at +/-10 m** and **5/6 at +/-20 m**. Neither
cohort has an automatic five-second hold completed wholly before the scheduled
10 s boundary: **0/6 in each**. The five large-offset successes start at about
5.025 s, so their five-second holds extend past that boundary.

The bundle's 74 original tests and 37 native-source checks are prior execution
provenance, not tests rerun in this analysis. No new Rust build, sensor replay,
independent truth assessment or physical-device qualification is claimed.

## Reproduce

Extract the retained review bundle, then run from the repository root:

```sh
python integration/imu_gnss/diagnose_recovery.py \
  --evaluation /path/to/extracted/evaluation \
  --output runs/automatic-recovery-diagnostics
python -m pytest -q \
  integration/imu_gnss/route_recovery_tests/test_diagnostics.py
```

The same reader can consume the existing recovery workflow's `runs/route-recovery`
directory. Output must be a new directory outside the evidence. It writes a full
`diagnostics.json`, all-case counts, accepted intervals, input hashes and `REPORT.md`.
Existing native qualification workflows and their assertions are not changed.

## Decision

No narrowly demonstrated estimator bug was isolated by this pass. Leave all
thresholds, recovery activation rules, junction abstention and default outputs
unchanged. The specific remaining diagnostic gap is observer-only capture of
selected branch offsets/weights/covariances and junction reasons in these same
cases. Until that evidence exists, “avoidable rejection” is unproven; it would be
premature to weaken abstention or add recovery for every slow-converging case.

## Diagnostic merge-review correction

The pre-boundary completion diagnostic now uses the **actual confirming sample**,
not merely the nominal `entry + 5 s` time. A review fixture accepts the route at
4.99 s, remains accepted at 9.98 s, and has its first sample at or after the
9.99 s endpoint at 10.05 s. The earlier diagnostic incorrectly called that a hold
completed before the 10 s boundary. The corrected diagnostic reports the 10.05 s
completion sample and rejects that pre-boundary claim. A completion exactly at
the boundary is also not strictly before it. The original entry/retention
metrics and the native evaluator are unchanged.

All 21 original offline tests remain; five new completion-sample cases bring the
diagnostic total to **26 passing tests**. Three new cases reproduce false
pre-boundary claims on the previous code. The correction was applied to both
the retained review evidence and the independently downloaded CI artifact from
PR #8 run `43704292395`, tested head
`7f00a42caf823a0873ce5729aa7172324eac08ce`. In each bundle all **54 automatic metric
sets** still match the exact original evaluator, all prior diagnostic fields
remain unchanged, and all **1,569 clean epochs** match. The additional
`first_5s_hold_completion_sample_s` field makes the confirming sample explicit.
The 2/6 and 5/6 outcomes, and zero complete pre-boundary automatic holds, remain
unchanged. This is an offline reporting correction, not a navigation correction.

That earlier CI artifact contains a fresh pinned native build, 95 passing Python
tests and the full paired replays. Those passes qualify its stated head only;
the corrected source requires fresh CI and must not inherit that earlier result.
