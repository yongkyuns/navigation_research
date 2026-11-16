# Scale-aware covariance review

## Reproduced defect

The original research `covariance_psd` guard used an eigenvalue tolerance based
on the largest covariance diagonal. Navigation combines quantities in different
units and with widely different variances. A large uncertainty in one coordinate
could consequently hide an impossible map cross-correlation in another.

A concrete 18+2 fixture uses `P[0,0]=1e-8`, `P[8,8]=1e4`, `U=2.25*I`, and
`C[0,0]=0.002`, with other navigation variances equal to one. The admissible
pairwise correlation bound for that entry is only `sqrt(1e-8*2.25)=0.00015`.
The joint matrix has minimum eigenvalue approximately -1.768e-6, but passed the
previous `2e-5*max(diag(Q),1)` tolerance. A corridor factor with distance 4 m,
half-width 0.5 m and local sigma 0.25 m then returned `APPLIED` with a **negative
posterior attitude variance**, approximately -1.170e-6. This is a constructed
numerical guard failure, not an observed error in the recorded trajectories.

## Correction

The shared opt-in native Schmidt covariance validator retains its original raw
symmetry/eigenvalue checks and adds a variance-normalized check. For positive
variances it forms `D^-1 Q D^-1`, where `D[i,i]=sqrt(Q[i,i])`, and applies the
existing tolerance to this dimensionless correlation matrix. Diagonal congruence
preserves positive semidefiniteness in exact arithmetic. Negative variances are
rejected; an exactly zero-variance coordinate must have an exactly zero row and
column. Malformed matrices and nonfinite/negative tolerances fail explicitly.

This is validation only. It never adds diagonal jitter, clips eigenvalues,
inflates covariance, changes units in the actual estimator, or alters a mean
update. Valid input bytes are unchanged. The corridor adapter already latches
post-native failures, so an invalid joint state cannot silently resume or switch
to raw EKF output. The matched evaluator now applies the same scale-aware check
to every initialized exported output, including the map-disabled arm.

No measurement sigma, acceptance threshold, native navigation equation, map
geometry, information-reuse budget, workflow definition, default output or
published viewer is changed.

## Executed checks

- **212 focused Python tests pass**, with zero failures/skips: all previous 178
  plus 34 new scale, singularity, unit-scaling and lifecycle regressions.
- Against the old validator, the new suite has **21 failures and 13 passing
  controls**. It reproduces the invalid joint prior and negative-posterior path,
  not merely a different error message.
- Valid full-rank and singular covariance fixtures spanning variance scales
  from 1e-18 to 1e18 pass without modification. Zero-map-uncertainty control and
  frozen directions remain supported.
- All four unmodified recordings were freshly replayed through the **original
  map-free native library**. All **159,075 sensor events** were processed, and
  the new guard accepted **142,267 initialized native covariance snapshots**.
- All 37 pinned upstream source hashes and input package hashes verify. Original
  library SHA-256: `5ad7955f748154e3ae66ed6fd5f04c0519991aac3ca3298dbcd772d4cd93affd`.

These baseline checks do not qualify the correlated corridor's evolving cross
covariance. No new instrumented build or correlated recorded-data replay was run
locally. The unchanged PR workflow must qualify both corrected arms on the exact
new head; earlier-head results are not substituted. If genuine correlated native
output fails the stronger guard, retain that failure and diagnose its equations
or transformations rather than relaxing the check.
