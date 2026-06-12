# PGA Support Selection and Phase-Gradient Truth Contract

## Purpose

This contract defines deterministic support selection and independently known
phase-gradient truth required before implementing a Phase Gradient Autofocus
estimator.

## Input Model

The truth request shall explicitly provide:

- a nonzero request identifier;
- a finite row-major complex image or phase-history matrix;
- the axis designated as slow time or aperture;
- one finite injected residual phase value per aperture sample;
- a support-selection policy and all thresholds;
- the expected phase gradient derived independently from the estimator under
  test.

No support threshold, axis orientation, or phase sign may be inferred from
global state.

## Deterministic Support Selection

The first supported policy is peak-relative magnitude support:

- compute magnitude for each candidate sample;
- determine the peak magnitude using deterministic first-index tie breaking;
- retain samples whose magnitude is greater than or equal to the explicit
  relative threshold multiplied by the peak;
- require an explicit minimum supported sample count;
- report the support mask and supported sample count.

The selector shall reject zero-energy input, non-finite input, thresholds
outside `(0, 1]`, and requests that do not meet minimum support.

## Phase-Gradient Truth

For injected residual phase values `phi[k]`, the reference forward phase
gradient is:

`gradient[k] = wrap(phi[k + 1] - phi[k])`

where wrapping maps to `[-pi, pi]`.

The truth result shall report one gradient value for each adjacent aperture
sample pair. Constant phase offsets shall not change the gradient truth.

## Acceptance Metrics

Future estimator acceptance shall report:

- support-mask equality;
- supported sample count;
- maximum absolute wrapped gradient error;
- root-mean-square wrapped gradient error;
- deterministic repeatability;
- rejection behavior for insufficient or invalid support.

## Atomic Behavior

Validation, support selection, and truth generation shall complete before
publishing a result. Rejected requests shall return no partial support mask or
gradient vector.

## Deferred Work

This contract does not authorize:

- estimated phase-gradient computation from image data;
- gradient integration or phase unwrap;
- iterative correction or stopping criteria;
- entropy optimization;
- production complex-image modification or session integration.
