# SAR External Raw IQ L2 Motion Compensation Contract

Date: 2026-06-12

## Approved boundary

External complete-aperture IQ may request the existing first-order L2 motion
compensation before L1 RDA when it supplies two complete local Cartesian
trajectories:

- `pulse_states`: actual platform trajectory associated with the IQ rows;
- `ideal_pulse_states`: nominal trajectory defining the desired aperture.

Both lists must contain one state per IQ row. Each list must independently be
finite, pulse-ID contiguous, and strictly time ordered.

## Behavior

- Valid dual trajectories are converted to the existing internal trajectory
  representation.
- The existing first-order range-envelope and phase compensation runs before
  the existing L1 RDA path.
- Missing or malformed actual and ideal trajectories fail atomically.
- External IQ without L2 enabled continues to ignore supplied trajectories in
  L1 RDA.

## Explicit exclusions

- The stage does not estimate an ideal trajectory from the actual trajectory.
- Trajectory frame conversion, time synchronization, navigation covariance,
  autofocus, and higher-order motion compensation remain deferred.
- External IQ and trajectories remain unsupported by summary replay.
