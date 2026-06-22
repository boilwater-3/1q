# SAR External Raw IQ Pulse Trajectory Acceptance Report

Date: 2026-06-12

## Result

External complete-aperture IQ can now carry a validated local Cartesian
per-pulse trajectory and drive the existing small-scene L3 BP path.

## Acceptance evidence

- Valid external IQ and pulse states complete BP and return the real public
  focused image.
- Mission waypoints are not required when the external trajectory is complete.
- Missing and invalid trajectories fail atomically.
- External-IQ L1 RDA behavior is unchanged.
- L2 motion compensation requires the later approved dual-trajectory contract;
  summary replay remains rejected.

## Verification

- Modern Windows debug build passed.
- Full CTest passed, 25/25.
- VS2015 `sar_core` build passed.
- `git diff --check` passed.
