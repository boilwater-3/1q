# SAR Phase Reference Acceptance Report

Date: 2026-06-22

## Scope

Completed the internal phase-reference extraction for the SAR imaging layer.

## Implemented

- Added `src/sar/imaging/SarPhaseReference.h`.
- Added `src/sar/imaging/SarPhaseReference.cpp`.
- Replaced the anonymous RDA broadside phase-reference helper with
  `ApplyBroadsideCenterPhaseReference`.
- Added global constant phase helper functions used by image comparison.
- Added RDA and Session diagnostics for applied phase-reference mode.
- Added `SarOutputFrame`, FlatBuffers replay, and TraceSink scalar summaries
  for phase-reference mode and applied status.
- Added focused unit coverage in `tests/unit/sar_phase_reference_test.cpp`.

## Preserved Boundary

- No public SAR config widening; public output changes are scalar summaries
  only.
- No Auto selector enablement.
- No spatially varying phase repair, unwrap, autofocus correction, or
  radiometric acceptance.

## Verification

- `cmake --build --preset llvm-ninja-release --target 1q_unit_tests`
- `build/llvm-ninja-release/bin/1q_unit_tests --gtest_filter='SarPhaseReferenceTest.*:SarImageQualityTest.*:SarRdaTest.*:SarGbpTest.*:SarSessionPipelineTest.*:SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`
- `ctest --preset llvm-ninja-release -R '^sar_' --output-on-failure -j 4`

Result: 69/69 focused phase/reference/quality/session/replay tests passed;
6/6 SAR CTest registrations passed.
