# SAR External Raw IQ L2 Motion Compensation Acceptance Report

Date: 2026-06-12

## Result

External complete-aperture IQ with complete actual and ideal local Cartesian
trajectories can now run the existing first-order L2 motion compensation before
L1 RDA and return the real public focused image.

## Acceptance evidence

- Valid dual trajectories execute motion compensation and RDA.
- Missing actual or ideal trajectories fail atomically.
- Invalid ideal trajectories fail independently from actual-trajectory
  validation.
- Existing external-IQ L1 RDA and BP paths remain available.
- Summary replay remains rejected.

## Verification

- Modern Windows debug build passed.
- Full CTest passed, 25/25.
- VS2015 `sar_core` build passed.
- `git diff --check` passed.
