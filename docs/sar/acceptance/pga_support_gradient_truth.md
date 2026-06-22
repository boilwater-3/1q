# PGA Support and Phase-Gradient Truth Acceptance Report

## Scope

Stage 135 implements deterministic peak-relative support selection and wrapped
forward phase-gradient truth generation.

## Accepted Behavior

- Requires explicit threshold, minimum support, aperture profile, and injected
  phase vector.
- Uses deterministic first-index tie breaking for equal peaks.
- Reports peak-relative support mask and supported sample count.
- Produces wrapped forward phase differences and is invariant to constant phase
  offsets.
- Rejects invalid, zero-energy, insufficient-support, and malformed phase truth
  requests atomically.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.

## Boundary

This executor supplies deterministic support and gradient truth. It does not
estimate phase gradients from image data and does not perform unwrap,
integration, iteration, or production image correction.
