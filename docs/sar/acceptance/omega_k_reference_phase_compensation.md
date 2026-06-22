# Omega-K Explicit Reference Phase Compensation Acceptance Report

## Scope

Stage 117 implements a narrowly scoped executor that applies an explicit
request-supplied phase vector to the validated Omega-K intermediate.

## Accepted Behavior

- Requires a nonzero request identifier and explicit positive or negative phase
  application sign.
- Requires finite absolute-range, azimuth, and per-range phase vectors.
- Requires matrix dimensions that match the coordinate axes.
- Applies a unit-magnitude complex phase factor independently to every range
  column.
- Preserves absolute-range and azimuth coordinates.
- Rejects malformed requests atomically.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed before acceptance recording.

## Boundary

The executor does not derive phase from geometry and does not perform the final
azimuth inverse transform. It remains a numerical intermediate until an
independent point-target truth validates the production phase model.
