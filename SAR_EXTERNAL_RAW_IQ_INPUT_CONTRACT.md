# SAR External Raw IQ Input Contract

Date: 2026-06-12

## Approved boundary

The first public external-IQ stage accepts one complete aperture through
`SarCycleInput::raw_iq` and feeds it directly into the existing L1 RDA path.

- Samples are complex values represented by separate row-major I and Q arrays.
- `pulse_count` is the azimuth row count.
- `samples_per_pulse` is the range column count.
- Shape must exactly match the configured mission aperture.
- Every I and Q value must be finite.
- Point targets are ignored when external IQ is present.

## Explicit exclusions

- L2 motion compensation and L3 BP are rejected because external per-pulse
  trajectory metadata is not yet contracted.
- Partial-aperture streaming and ring-buffer accumulation are not supported.
- Current summary-only replay cannot serialize external IQ. `SarTraceSession`
  rejects external IQ when a replay writer is attached.
- Calibration, quantization metadata, channel ordering, and real sensor file
  formats require later contracts.

## Acceptance

An accepted external IQ frame must complete L1 RDA and return the real public
focused image payload. Malformed shapes, non-finite samples, and advanced-path
requests must fail atomically with explicit error codes.
