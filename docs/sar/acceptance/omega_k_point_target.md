# Omega-K Point-Target Acceptance Evaluator Report

## Scope

Stage 121 implements an atomic evaluator that consumes independent point-target
truth and measures the Omega-K numerical image candidate.

## Accepted Behavior

- Rejects truth that is not declared independent or lies outside common
  support.
- Requires strictly increasing finite coordinate axes and a finite candidate.
- Locates the maximum-power sample and reports coordinate, wrapped phase, and
  relative magnitude errors.
- Reports range and azimuth PSLR and ISLR from explicit mainlobe widths.
- Distinguishes valid quality failures from malformed request rejection.
- Publishes no partial metrics for rejected requests.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed before acceptance recording.
- Eigen 3.3.9 targeted runtime launch remains pending because managed sandbox
  spawn setup refresh failed.

## Boundary

The evaluator provides the acceptance mechanism, but production Omega-K image
acceptance still requires a genuinely independent physical point-target
dataset. The synthetic unit fixture validates evaluator mathematics only.
