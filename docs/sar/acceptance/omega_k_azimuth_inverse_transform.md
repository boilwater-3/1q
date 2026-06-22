# Omega-K Numerical Azimuth Inverse Transform Acceptance Report

## Scope

Stage 119 implements the explicitly coordinated and normalized numerical
azimuth inverse transform approved by Stage 118.

## Accepted Behavior

- Requires a nonzero request identifier.
- Requires finite absolute-range and output azimuth coordinate axes.
- Requires a finite positive additional normalization factor.
- Validates matrix dimensions and finite complex samples.
- Applies the inverse FFT down each range column.
- Applies and reports the explicit additional normalization.
- Preserves both coordinate axes and rejects malformed requests atomically.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.
- Windows Eigen 3.3.9 SAR runtime test launch remains pending because managed
  sandbox spawn setup refresh failed.

## Boundary

The output is named a numerical image candidate. Physical SAR image acceptance
remains deferred until independent point-target truth verifies coordinates,
peak response, and sidelobes.
