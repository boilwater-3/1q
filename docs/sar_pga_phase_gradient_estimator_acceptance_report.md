# PGA Adjacent-Sample Phase-Gradient Estimator Report

## Scope

Stage 137 implements the first bounded PGA phase-gradient estimator using
adjacent supported-sample conjugate products.

## Accepted Behavior

- Requires a finite complex aperture profile, explicit binary support mask, and
  minimum valid-pair count.
- Estimates wrapped phase difference using `arg(conj(z[k]) * z[k+1])`.
- Does not bridge unsupported gaps or zero-magnitude adjacent samples.
- Reports one validity flag and one gradient slot per adjacent pair.
- Rejects malformed or insufficient requests atomically.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.

## Boundary

The estimator reports wrapped gradients only. Truth comparison, integration,
unwrap, iterative correction, and production image modification remain
separate stages.
