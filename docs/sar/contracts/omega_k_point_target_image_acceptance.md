# Omega-K Point-Target Image Acceptance Contract

## Purpose

This contract defines the independent truth and image-quality evidence required
to accept an Omega-K numerical image candidate as a physically meaningful SAR
image.

## Independence Requirement

The point-target truth shall be generated independently from the executor under
test. It shall not reuse the executor's coordinate mapping, reference-phase
implementation, interpolation output, or transform results to compute expected
values.

The truth definition shall explicitly provide:

- target absolute slant range in metres;
- target azimuth coordinate and unit;
- expected complex peak phase and magnitude;
- expected range and azimuth impulse-response characteristics;
- tolerances and the physical reason for each tolerance;
- whether the target is inside the reduced common-support region.

## Candidate Inputs

An acceptance request shall provide:

- a nonzero request identifier;
- absolute slant-range coordinates;
- azimuth coordinates;
- a finite complex numerical image candidate;
- the independently generated point-target truth;
- explicit peak-location, phase, magnitude, and sidelobe tolerances.

All axes shall be finite, strictly monotonic, and dimensionally consistent with
the image candidate.

## Required Metrics

The acceptance evaluator shall report at least:

- measured peak row and column;
- measured peak absolute slant range and azimuth coordinate;
- absolute range and azimuth location errors;
- measured peak complex phase and magnitude;
- wrapped peak-phase error;
- relative peak-magnitude error;
- peak sidelobe ratio in range and azimuth;
- integrated sidelobe ratio in range and azimuth;
- pass/fail status for every metric.

## Support Boundary

Targets declared outside the reduced common-support region shall be rejected
before image-quality metrics are evaluated. A target near a support edge shall
use explicitly relaxed tolerances or be classified as a separate edge case.

## Atomic Behavior

The evaluator shall validate the complete request before publishing metrics. A
rejected request shall not return partial measurements or a partial acceptance
decision.

## Acceptance Rule

The numerical image candidate may be classified as a physical SAR image only
when every required metric passes its declared tolerance and the truth
independence record is present.

An FFT round trip, a visually plausible image, or a self-generated expected
matrix is insufficient acceptance evidence.
