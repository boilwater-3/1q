# SAR External Raw IQ Input Acceptance Report

Date: 2026-06-12

## Result

The first bounded public external-IQ path is accepted for complete-aperture L1
RDA processing.

`SarCycleInput::raw_iq` now carries separate row-major I and Q arrays. An
accepted frame bypasses internal point-target echo generation, runs the
existing RDA implementation, and returns the real public focused-image payload.

## Accepted behavior

- Exact configured aperture shape is required.
- Every I and Q value must be finite.
- L1 RDA completes and returns a non-placeholder focused image.
- Point targets are explicitly ignored when external IQ is present.
- Trace sessions without a replay writer can process external IQ.

## Rejected behavior

- Shape mismatch and non-finite values fail atomically.
- L2 motion compensation and L3 BP require the later approved per-pulse
  trajectory contracts.
- Summary-only replay encoding rejects external IQ instead of silently dropping
  the payload.

## Verification

- Modern Windows debug build passed.
- SAR targeted tests passed.
- Public API boundary checks passed.
- Full CTest passed, 25/25.
- VS2015 `sar_core` build passed.
- `git diff --check` passed.
