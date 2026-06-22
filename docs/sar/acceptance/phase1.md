# SAR Phase 1 Acceptance Report

Date: 2026-06-05

## Scope

This report summarizes the current-platform acceptance state for SAR Phase 1 in the `codex/sar-phase1` worktree.

Approved Phase 1 scope:

- Public SAR API and CMake module skeleton.
- LFM waveform generation.
- Matched filter construction.
- Range compression.
- Point-target raw echo generation.
- L1 straight stripmap geometry.
- Internal FFT facade.
- Minimal RDA focusing for L1 broadside point-target scenes.
- Pulse ring buffer primitive.
- `SarSession::StepWithResult()` raw/range/RDA execution.
- Summary-level FlatBuffers trace/replay.

Explicitly out of scope:

- Auto algorithm selection.
- GBP/BP/CSA/Omega-K.
- L2/L3 trajectory imaging.
- Motion compensation and autofocus.
- Radiometric calibration and clutter modeling.
- HDF5/GeoTIFF output.
- GPU acceleration.
- Performance approval beyond the current 1024x1024 point-target gate.

## Acceptance Matrix

| Area | Evidence | Status |
|---|---|---|
| Public API boundary | `PublicHeadersSmokeTest.SarPublicSurfaceSupportsMinimalUsage`, `public_api_boundary_guard` | Pass |
| FFT facade | `SarFftBackendTest.*` | Pass on macOS AppleClang + Conan Eigen 3.4.0 and 3.3.9 |
| C++11 compatibility | `sar_cxx11_compat` | Pass for all SAR engine sources with Conan Eigen 3.3.9 |
| LFM and range compression | `SarSignalChainTest.*` | Pass |
| Geometry/raw echo/buffer | `SarGeometryTest.*`, `SarEchoTest.*`, `PulseRingBufferTest.*` | Pass |
| RDA | `SarRdaTest.*` | Pass for L1 broadside point targets |
| Current-platform performance | `SarPerformanceTest.*` | Pass for 1024x1024 FFT, internal RDA, point-target pipeline, and public Session |
| Session execution | `SarSessionPipelineTest.*` | Pass |
| Replay codec | `SarReplayCodecRoundtripTest.*` | Pass |
| Trace/replay playback | `SarReplaySessionTest.*` | Pass |

## CI Entry Points

SAR-specific CTest labels:

```sh
ctest --test-dir build/llvm-ninja-debug-local -L sar_unit
ctest --test-dir build/llvm-ninja-debug-local -L sar_replay
ctest --test-dir build/llvm-ninja-debug-local -L sar_integration
ctest --test-dir build/llvm-ninja-debug-local -L sar_contract
ctest --test-dir build/llvm-ninja-debug-local -L sar_ci
ctest --test-dir build/llvm-ninja-debug-local -L sar_performance
cmake --preset llvm-ninja-debug-eigen339
ctest --test-dir build/llvm-ninja-debug-eigen339-local -L sar_cxx11_compat
```

Direct verification commands used for the current acceptance snapshot:

```sh
cmake --preset llvm-ninja-debug
cmake --build --preset llvm-ninja-debug --target 1q_unit_tests
cmake --build --preset llvm-ninja-debug --target 1q_replay_fast_tests
cmake --build --preset llvm-ninja-debug --target 1q_contract_tests
build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*:SarSessionPipelineTest.*:SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'
build/llvm-ninja-debug-local/bin/1q_replay_fast_tests
build/llvm-ninja-debug-local/bin/1q_contract_tests
cmake -DSOURCE_DIR=/Users/aurora/Code/1q-sar-phase1 -P tests/contract/check_public_api_boundary.cmake
git diff --check
```

Latest observed results:

- `sar_ci` CTest label: 4/4 passed.
- SAR filtered tests: 42/42 passed.
- SAR performance entry: 1/1 CTest entry and 4/4 internal gtests passed.
- 1024x1024 two-dimensional FFT facade core transform observed at approximately 344 ms in Debug.
- 1024x1024 internal synthetic RDA observed at approximately 934-996 ms in Debug.
- 1024x1024 internal point-target pipeline observed at approximately 1493 ms in Debug.
- 1024x1024 public Session observed at approximately 1421-1496 ms in Debug.
- Isolated 1024x1024 public Session maximum resident set size observed at 137035776 bytes.
- AppleClang + Conan Eigen 3.3.9 SAR FFT/signal/RDA/Session tests: 25/25 passed; performance entry passed.
- SAR engine C++11 + Conan Eigen 3.3.9 compatibility gate: 1/1 passed.
- Replay fast tests: 71/71 passed.
- Contract tests: 72/72 passed.
- Public API boundary guard: passed.
- `git diff --check`: passed.

## Metrics Coverage

Current implemented metrics:

- Range pulse 3 dB width.
- Range pulse 20 dB width.
- Azimuth 3 dB width at the focused-image peak range column.
- PSLR.
- ISLR.
- RDA reference range diagnostics.
- RDA Doppler rate diagnostics.
- RDA range-bin spacing diagnostics.
- RDA RCMC interpolation and out-of-bounds diagnostics.
- Focused-image normalized-power Shannon entropy in nats.

Not yet implemented:

- Performance and peak-memory evidence beyond 1024x1024.
- Absolute radiometric metrics.

These missing metrics do not block the current minimum Phase 1 algorithm loop, but they must block any claim of production image-quality validation beyond the current point-target acceptance scenes.

## Replay Policy

Phase 1 trace/replay records public summaries only:

- `SarCycleInput`.
- `SarCycleResult`.
- `SarSessionConfig`.
- `SarRuntimeConfigPatch`.

Focused complex image matrices are not serialized in Phase 1. Full-matrix replay, external artifact references, compression policy, and image product formats require a separate approval gate.

## Known Limits

- Current RDA is accepted only for L1 straight stripmap, broadside or near-broadside, point-target scenes.
- `SarSession` has a current-platform runtime RDA size gate: `range_sample_count <= 1024` and `azimuth_pulse_count <= 1024`.
- C++11 + Conan Eigen 3.3.9 has been verified for all SAR engine sources. Windows/VS2015 is not a
  Phase 1 mandatory approval gate.
- Performance beyond 1024x1024 has not been measured or approved.
- The current `sar_performance` label does not provide Windows/VS2015 evidence.
- `PulseRingBuffer` is now used by `SarSession` as the cross-cycle slow-time accumulator, but only for the current small-scene Phase 1 path.
- LLA-to-local conversion in Session uses a small-scene flat-earth approximation.

## Approval Recommendation

Approve SAR Phase 1 with a frozen 1024x1024 point-target runtime ceiling, summary-level replay,
and C++11 + Eigen 3.3.9 SAR engine compatibility evidence.

Do not approve it yet for:

- SAR image performance beyond the current 1024x1024 point-target gate.
- Operational image-quality claims.
- Full-image trace/replay.
- Phase 2 algorithms or Auto selection.
