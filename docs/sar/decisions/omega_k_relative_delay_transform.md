# Omega-K Relative Delay Transform Follow-up Decision

## Decision

The relative-delay transform is accepted as an internal numerical Omega-K
intermediate. It is not accepted as a focused SAR image or as evidence that
the production Omega-K imaging path is complete.

The next Omega-K step must not silently apply an azimuth inverse transform and
label the result as a physical image. Before that integration is approved, the
pipeline still needs an explicit contract for:

- reference-phase compensation and its sign convention;
- mapping relative delay samples to absolute slant range;
- azimuth output coordinates and normalization;
- validity limits introduced by common-support grid reduction;
- end-to-end comparison against an independently generated point-target truth.

## Evidence

The completed relative-delay executor:

- accepts only an explicitly reduced, uniformly spaced range-frequency grid;
- performs the inverse range-frequency transform atomically;
- reports the relative delay and relative range spacing;
- preserves the row count and reduced column count;
- rejects malformed axes and non-finite spectra.

These properties establish a deterministic numerical intermediate, but they do
not establish the physical phase reference or absolute image coordinates.

## Follow-up

Defer production azimuth inverse-transform integration. The next implementation
stage should freeze an Omega-K reference-phase and absolute-range mapping
contract, including a point-target truth case that can distinguish a
numerically plausible matrix from a correctly focused physical image.
