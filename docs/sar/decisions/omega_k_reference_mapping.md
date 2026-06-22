# Omega-K Reference Mapping Follow-up Decision

## Decision

Approve a narrowly scoped analytic reference-phase compensation executor as the
next Omega-K intermediate stage.

The approved executor may apply only a phase model supplied explicitly by the
request. It must not derive hidden geometry, choose a Fourier sign convention,
or perform the final azimuth inverse transform.

## Required Boundary

The compensation request must provide:

- a nonzero request identifier;
- an explicit phase sign;
- one finite phase value in radians for each absolute slant-range sample;
- the absolute slant-range and azimuth coordinate axes;
- the validated complex intermediate matrix.

The executor shall multiply each matrix column by the declared unit-magnitude
complex phase factor, preserve all coordinates, and reject malformed requests
atomically.

## Deferred Work

This approval does not establish the production phase model. Deriving phase
from flight geometry and validating final image focus still require an
independent point-target truth and a later integration decision.
