# Omega-K Reference Mapping Acceptance Report

## Scope

Stage 115 implements the explicit request executor defined by
`SAR_OMEGA_K_REFERENCE_PHASE_ABSOLUTE_RANGE_CONTRACT.md`.

## Accepted Behavior

- Requires a nonzero request identifier.
- Requires finite positive propagation speed and transform normalization.
- Requires a finite nonnegative reference slant range.
- Requires explicit delay and reference-phase sign conventions.
- Validates finite relative-delay and azimuth coordinate axes.
- Validates the complex matrix shape and finite samples.
- Maps relative delay to absolute slant range using an explicit increasing or
  decreasing range convention.
- Rejects negative or non-finite absolute ranges atomically.
- Preserves the validated complex intermediate without claiming that reference
  phase compensation or final image focusing has been completed.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.
- Windows Eigen 3.3.9 SAR runtime test launch: pending because the managed
  sandbox repeatedly failed during spawn setup refresh.

## Boundary

This executor establishes explicit physical metadata and absolute range
coordinates. It does not apply an analytic reference-phase compensation or
perform the final azimuth-domain transform, so production Omega-K image
integration remains deferred.
