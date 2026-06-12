# SAR Public Focused Image Output Contract

Date: 2026-06-12

## Current delivery

`SarSession::StepWithResult` returns the real focused complex image through
`SarCycleResult::focused_image` whenever RDA or BP focusing succeeds.

The public payload is independent from the internal matrix type:

- `source` identifies RDA or BP.
- `row_count` and `column_count` define the image shape.
- `real_values` and `imaginary_values` contain row-major complex samples.
- `is_placeholder` is `false` for the current implementation because the
  payload is copied from the actual focused image.

The two value arrays must each contain exactly `row_count * column_count`
samples when `source` is not `kNone`.

## Replay boundary

Phase 1 replay remains summary-only. Focused complex image arrays are not
serialized into the existing replay schema. A replayed or reused summary must
not claim to contain a current focused-image payload.

## Follow-up work

- Add an explicit opt-in retention policy if large public image copies become
  a performance concern.
- Add artifact-backed replay and product export without expanding the current
  summary replay event.
- Add calibrated magnitude, georeferencing, and image product metadata as
  separately versioned public contracts.
