# Omega-K Explicit Phase Compensation Follow-up Decision

## Decision

Approve a numerical azimuth inverse-transform executor as the next Omega-K
intermediate stage.

The executor may transform the compensated matrix from azimuth frequency to
azimuth coordinate only when the request supplies the output axis and declares
the transform normalization. Its output remains a numerical image-domain
candidate, not an accepted physical SAR image.

## Required Boundary

The request must provide:

- a nonzero request identifier;
- absolute slant-range coordinates;
- explicit output azimuth coordinates;
- a finite positive transform normalization;
- the compensated complex intermediate matrix.

The executor shall apply the inverse transform down each range column, apply
the declared normalization, preserve coordinates, and reject malformed input
atomically.

## Deferred Work

Production image acceptance remains deferred until an independently generated
point-target truth verifies absolute location, peak phase and magnitude, and
range/azimuth sidelobe behavior.
