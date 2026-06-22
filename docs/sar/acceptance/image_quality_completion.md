# SAR Image Quality Completion Acceptance Report

Date: 2026-06-22

## Scope

Completed the internal SAR image-quality evaluator for focused regression
metrics.

## Implemented

- Added `ImageQualityConfig`.
- Added `MainlobeEstimationMethod{k3dB,k20dB}`.
- Added meter-scale resolution fields guarded by valid pixel spacing.
- Added image contrast.
- Kept `EvaluateImageQuality(image)` source compatible.
- Kept global phase comparison behavior limited to constant phase removal.
- Added RDA diagnostics for phase reference, meter-scale resolution, and
  contrast.
- Added Session diagnostic visibility for phase reference, meter-scale
  resolution, and contrast.
- Added `SarOutputFrame`, FlatBuffers replay, and TraceSink scalar summaries
  for mainlobe method, bin widths, meter-scale resolution, entropy, contrast,
  and validity flags.

## Preserved Boundary

- Public result/replay/trace changes are limited to scalar summaries.
- No radiometric accuracy metric.
- No threshold weakening in RDA/GBP/BP comparison tests.
- No use of image-quality metrics to hide spatially varying phase errors.

## Verification

- `cmake --build --preset llvm-ninja-release --target 1q_unit_tests`
- `build/llvm-ninja-release/bin/1q_unit_tests --gtest_filter='SarPhaseReferenceTest.*:SarImageQualityTest.*:SarRdaTest.*:SarGbpTest.*:SarSessionPipelineTest.*:SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`
- `build/llvm-ninja-release/bin/1q_unit_tests`
- `ctest --preset llvm-ninja-release -R '^sar_' --output-on-failure -j 4`

Result: 69/69 focused phase/reference/quality/session/replay tests passed;
full unit binary passed 918/918; 6/6 SAR CTest registrations passed.
