# PGA Gradient Estimator Truth Comparison Contract

## Purpose

This contract defines deterministic comparison between bounded PGA
phase-gradient estimates and independently supplied wrapped gradient truth.

## Required Inputs

The comparison request shall explicitly provide:

- a nonzero request identifier;
- one estimated wrapped gradient per adjacent aperture pair;
- one estimator validity flag per pair;
- one truth wrapped gradient per adjacent pair;
- one truth-validity flag per pair;
- a minimum jointly valid pair count;
- maximum absolute wrapped-error and RMS wrapped-error tolerances.

All vectors shall have identical lengths. Validity flags shall be binary.

## Pair Alignment

Only pairs valid in both estimator and truth masks shall contribute to metrics.
Invalid-pair gradient placeholders shall be ignored completely.

The result shall report:

- the jointly valid pair mask;
- jointly valid pair count;
- maximum absolute wrapped gradient error;
- RMS wrapped gradient error;
- pass or fail against both tolerances.

## Wrapped Error

Each error shall be computed as:

`wrap(estimated[k] - truth[k])`

with wrapping to `[-pi, pi]`.

## Atomic Behavior

Malformed requests or insufficient jointly valid pairs shall be rejected
without publishing partial masks or metrics.

## Deferred Work

Comparison acceptance does not authorize phase integration, unwrap, iterative
correction, stopping criteria, or production image modification.
