# PGA Support and Gradient Truth Follow-up Decision

## Decision

Approve a bounded adjacent-sample phase-gradient estimator as the first PGA
estimation stage.

For adjacent supported complex aperture samples `z[k]` and `z[k+1]`, the
estimator shall compute:

`gradient[k] = arg(conj(z[k]) * z[k + 1])`

The estimator shall report validity for each adjacent pair and shall not bridge
unsupported samples.

## Required Boundary

The request must provide:

- a nonzero request identifier;
- a finite complex aperture profile;
- an explicit support mask with matching length;
- a minimum valid adjacent-pair count.

The result must report:

- one wrapped gradient estimate per adjacent aperture pair;
- one validity flag per estimate;
- the valid pair count;
- deterministic atomic rejection for invalid input or insufficient valid pairs.

## Acceptance Comparison

Estimator acceptance against Stage 135 truth shall use only valid adjacent
pairs and shall report maximum absolute wrapped error and RMS wrapped error.

## Deferred Work

Phase integration, unwrap, iterative correction, stopping criteria, entropy
optimization, and production image modification remain deferred.
