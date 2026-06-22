# PGA Gradient Truth Comparison Evaluator Report

## Scope

Stage 139 implements jointly valid wrapped-error comparison for bounded PGA
phase-gradient estimates.

## Accepted Behavior

- Requires aligned finite estimator/truth gradient vectors and binary validity
  masks.
- Ignores pairs not valid in both masks.
- Reports jointly valid pair mask and count.
- Computes wrapped maximum absolute error and RMS error.
- Distinguishes valid tolerance failure from malformed-request rejection.
- Rejects insufficient jointly valid pairs atomically.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.

## Boundary

The evaluator can accept the bounded gradient estimator against supplied truth.
It does not authorize gradient integration, unwrap, iterative correction, or
production image modification.
