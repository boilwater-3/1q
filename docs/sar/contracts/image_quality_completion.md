# SAR Image Quality Completion Contract

Date: 2026-06-22

## Purpose

This contract completes the SAR image-quality evaluator for point target and
focused-image regression use, plus scalar result summaries for public
session/replay/trace boundaries. It does not approve public product quality
reporting or radiometric acceptance.

## Approved Boundary

The image-quality module may provide:

- deterministic peak location and peak magnitude;
- mainlobe width in bins;
- optional range and azimuth resolution in meters when pixel spacing is valid;
- PSLR and ISLR using an explicit mainlobe estimation method;
- entropy from normalized image power;
- image contrast from image intensity distribution;
- global constant phase comparison through the phase-reference helper.

Existing `EvaluateImageQuality(image)` behavior shall remain source compatible.
An overload may accept an explicit quality config.

## Mainlobe Methods

The first approved methods are:

- `k3dB`: current behavior, using half-power threshold from the peak row and
  column;
- `k20dB`: wider threshold method for deterministic tests and diagnostics.

Any future IRW method requires a separate truth definition before it can be an
acceptance gate.

## Atomic Behavior

The evaluator shall reject malformed matrices and zero-energy images without
publishing partial quality metrics.

Pixel spacing values must be finite and positive before meter-scale resolution
fields are marked available. Invalid spacing shall not invalidate bin-scale
metrics.

## Explicit Exclusions

- Public changes are limited to scalar `SarOutputFrame` summaries; no focused
  complex image matrix is serialized into replay.
- No radiometric accuracy metric until the radiometric calibration path is
  separately approved for the relevant boundary.
- No threshold weakening in existing RDA, GBP, BP, motion-compensation, PRF, or
  reference-matrix tests.
- No use of quality metrics to hide spatially varying phase errors.

## Acceptance

Acceptance requires:

- existing image-quality tests continue to pass;
- deterministic tests for meter-scale resolution, contrast, and method-specific
  mainlobe width;
- global phase comparison still removes only constant phase and normalizes only
  global amplitude scale;
- RDA diagnostics continue to publish existing quality fields, with additional
  fields only after focused regression tests pass.
