# SAR External Raw IQ Pulse Trajectory Contract

Date: 2026-06-12

## Approved boundary

External complete-aperture IQ may carry one local Cartesian platform state for
each pulse row. This first trajectory stage approves those states only for the
existing small-scene L3 BP path.

Each pulse state contains:

- contiguous `pulse_id`;
- strictly increasing `time_s`;
- local Cartesian position in meters;
- Cartesian velocity in meters per second.

All fields must be finite. The local coordinate convention matches the SAR
internal geometry: x is azimuth, y is ground range, and z is altitude.

## Behavior

- External IQ plus a complete valid pulse-state list may drive L3 BP without
  mission waypoints.
- External IQ L1 RDA remains supported; supplied pulse states are explicitly
  ignored because the current RDA path uses the configured nominal geometry.
- Missing, malformed, non-finite, non-contiguous, or non-monotonic BP
  trajectory metadata is rejected atomically.

## Explicit exclusions

- External-IQ L2 motion compensation requires the later approved ideal plus
  actual dual-trajectory contract.
- Geodetic pulse-state input, attitude, antenna phase-center offsets, timing
  uncertainty, and navigation covariance remain deferred.
- External IQ and trajectory replay remain unsupported by the summary schema.
