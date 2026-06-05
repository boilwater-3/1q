# SAR Phase 1 Engineering Contract

## 1. Purpose

This document freezes the engineering contract for SAR Phase 1 implementation in 1Q. It narrows the broader SAR construction and design documents to a buildable, testable scope.

Phase 1 is approved only for a minimum stripmap SAR imaging loop:

1. Public module/session contract.
2. LFM waveform generation.
3. Matched filter construction.
4. Range pulse compression.
5. Point-target raw echo generation.
6. L1 uniform straight-line platform trajectory.
7. RDA focusing.
8. Pulse ring buffer for fast/slow time decoupling.
9. Basic image quality metrics and deterministic tests.

The following are explicitly out of Phase 1: Auto focusing selection, GBP, BP, CSA, Omega-K, L2/L3 trajectory imaging, motion compensation, autofocus, radiometric calibration, clutter modeling, HDF5, GeoTIFF, GPU acceleration, maneuver behavior, reconnaissance scheduling, and multi-sensor fusion.

## 2. Approval Status

| Item | Phase 1 Status | Reason |
|---|---|---|
| Public API and CMake integration | Required | Must match existing 1Q module style before algorithm code is public |
| LFM waveform | Approved | Closed-form signal source, low integration risk |
| Matched filter | Approved | Contract must prevent duplicate conjugation |
| Range pulse compression | Approved | Requires real FFT backend decision before large data tests |
| Point-target raw echo | Conditionally approved | Must use double-way delay and explicit sampling grid |
| RDA | Conditionally approved | Only for L1 straight stripmap and fixed reference scenes |
| Pulse ring buffer | Approved | Must enforce pulse ID continuity and fractional pulse carry |
| Auto algorithm selection | Rejected for Phase 1 | Needs multiple validated algorithms first |
| GBP/BP/CSA/Omega-K | Rejected for Phase 1 | Separate algorithm contracts required |
| Radiometric calibration | Rejected for Phase 1 | Existing RCS helpers are not a full SAR calibration chain |
| HDF5/GeoTIFF/GPU | Rejected for Phase 1 | No current dependency evidence in the repo |

## 3. Public API Contract

SAR must follow the same high-level shape as `airborne_radar`, `electronic_surveillance_radar`, and `electro_optical_sensor`.

Required public files:

- `include/1q/sar/sar.hpp`
- `include/1q/sar/config/sar_config.hpp`
- `include/1q/sar/config/SarHardwareConfig.h`
- `include/1q/sar/config/SarMissionConfig.h`
- `include/1q/sar/config/SarPolicyConfig.h`
- `include/1q/sar/config/SarEnvironmentConfig.h`
- `include/1q/sar/config/SarSessionConfig.h`
- `include/1q/sar/config/SarRuntimeConfigPatch.h`
- `include/1q/sar/session/SarCycleInput.h`
- `include/1q/sar/session/SarCycleResult.h`
- `include/1q/sar/session/SarSession.h`
- `include/1q/sar/session/SarSessionFactory.h`
- `include/1q/sar/session/SarTraceSession.h`
- `include/1q/sar/session/SarReplaySession.h`

Required implementation groups:

- `src/sar/CMakeLists.txt`
- `src/sar/session/*`
- `src/sar/runtime/*`
- `src/sar/signal/*`
- `src/sar/geometry/*`
- `src/sar/echo/*`
- `src/sar/imaging/*`
- `src/sar/output/*`
- `schemas/replay/sar_replay.fbs`
- `schemas/replay/sar_session_replay.fbs`

`SarSession` must expose:

```cpp
class ONEQ_API SarSession {
 public:
  SarSession();
  ~SarSession();
  SarSession(const SarSession&) = delete;
  SarSession& operator=(const SarSession&) = delete;
  SarSession(SarSession&&) noexcept;
  SarSession& operator=(SarSession&&) noexcept;

  session::SarOutputFrame Step(const session::SarCycleInput& input);
  session::SarCycleResult StepWithResult(const session::SarCycleInput& input);

  void ApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);
  bool TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);
};
```

The public API must not expose internal algorithm objects, mutable Eigen matrices, raw pointers to image storage, or thread primitives.

## 4. Coordinate and Unit Contract

All Phase 1 geometry uses a local Cartesian scene frame.

| Quantity | Unit | Contract |
|---|---|---|
| Platform position | m | Local scene frame, x = azimuth, y = ground range, z = altitude |
| Platform velocity | m/s | L1 constant velocity along +x |
| Target position | m | Local scene frame |
| Slant range `R` | m | Euclidean platform-target distance |
| Fast time `t_r` | s | Range sampling time |
| Slow time `t_a` | s | Pulse time, indexed by pulse ID |
| Carrier frequency `f_c` | Hz | Required for wavelength |
| Wavelength `lambda` | m | `c / f_c` |
| Bandwidth `B` | Hz | LFM bandwidth |
| Sample rate `f_s` | Hz | Must satisfy `f_s >= 2B` |
| PRF | Hz | Independent of macro simulation step |

ECEF/geodetic conversion may be added later through existing coordinate facilities, but Phase 1 tests must use a local frame to avoid geodesy noise in algorithm validation.

## 5. Sampling Contract

LFM parameters:

```text
T = BT / B
K = B / T
N_s = ceil(T * f_s)
s[n] = exp(j * 2*pi * (f0 * n/f_s + 0.5 * K * (n/f_s)^2))
```

Range-bin spacing:

```text
delta_r = c / (2 * f_s)
```

Two-way delay for pulse `p` and target `q`:

```text
R[p, q] = norm(platform_position[p] - target_position[q])
tau[p, q] = 2 * R[p, q] / c
n_delay[p, q] = round(tau[p, q] * f_s)
```

Pulse count per macro step must preserve fractional carry:

```text
n_burst = floor(dt_macro * prf + pulse_fraction_carry)
pulse_fraction_carry_next = dt_macro * prf + pulse_fraction_carry - n_burst
```

Dropping fractional pulses is not allowed.

## 6. Matrix Layout Contract

All SAR algorithm matrices in Phase 1 use row-major semantic indexing:

```text
echo(range_bin, pulse_index)
compressed(range_bin, pulse_index)
focused(range_bin, azimuth_pixel)
```

Implementation may store with Eigen default column-major layout, but function names and comments must preserve the semantic axes above.

Do not use ambiguous `rows x cols` language without naming the axes.

## 7. FFT Backend Contract

The current `common/numerics::ZFFT1D` is a naive O(n^2) DFT and is not acceptable for Phase 1 imaging sizes beyond small unit tests.

Before implementing large RDA tests, choose one of:

1. Introduce a real FFT backend dependency through Conan/CMake.
2. Use a vetted in-tree FFT implementation.
3. Add a constrained Eigen FFT wrapper if available in the configured dependency set.

Required FFT API:

```cpp
std::vector<std::complex<double>> Fft1D(
    const std::vector<std::complex<double>>& input,
    bool inverse);

Eigen::MatrixXcd FftRows(const Eigen::MatrixXcd& input, bool inverse);
Eigen::MatrixXcd FftCols(const Eigen::MatrixXcd& input, bool inverse);
```

Normalization convention:

- Forward FFT is unnormalized.
- Inverse FFT divides by `N`.
- Tests must assert this convention with a round-trip identity check.

Phase 1 freezes the approved runtime ceiling at 1024x1024. The SAR engine must
compile as C++11 against Eigen 3.3.9 on the current validation platform.
Windows/VS2015 is not a Phase 1 approval gate. Any size above 1024x1024 requires
a later-stage performance and memory approval.

## 8. Matched Filter Contract

The time-domain matched filter is:

```text
h[n] = conj(s[N_s - 1 - n])
```

The frequency-domain matched kernel is:

```text
H[k] = conj(S[k])
```

Implementations must choose one primary representation and record it in the type name or function name. It is not allowed to build `h[n]` with conjugation and then multiply by `conj(FFT(h))` again.

Linear convolution length:

```text
L_f >= N_x + N_s - 1
```

Pulse compression:

```text
Y[k] = FFT(x, L_f)[k] * FFT(h, L_f)[k]
y = IFFT(Y)
```

The returned compressed pulse must define whether it is:

- Full convolution output of length `L_f`.
- Cropped output aligned to original range bins.
- Peak-centered diagnostic output.

Phase 1 production path uses cropped output aligned to range bins. Unit tests may also inspect full convolution output.

## 9. Raw Echo Contract

Phase 1 raw echo generation supports only point targets:

```cpp
struct SarPointTarget {
  std::string id;
  Eigen::Vector3d position_m;
  double rcs_linear;
};
```

For pulse `p`, target `q`, and fast-time sample `n`:

```text
x_p[n] += A[p,q] * s[n - n_delay[p,q]] * exp(-j * 4*pi*R[p,q] / lambda)
```

Amplitude for Phase 1:

```text
A[p,q] = sqrt(rcs_linear) / max(R[p,q]^2, epsilon)
```

This is a relative-power echo model. It is not a radiometrically calibrated received-power model.

Noise, clutter, antenna pattern, atmospheric attenuation, and absolute transmitter power are out of Phase 1 unless explicitly added as disabled-by-default diagnostic parameters.

## 10. RDA Contract

RDA Phase 1 is allowed only under these assumptions:

- Stripmap mode.
- L1 uniform straight-line platform motion.
- Broadside or near-broadside geometry.
- Point targets.
- Constant PRF.
- Constant carrier frequency and bandwidth.
- No squint correction.
- No motion compensation.
- No autofocus.
- No radiometric calibration.

Required RDA stages:

1. Range compression.
2. Azimuth FFT.
3. RCMC.
4. Azimuth matched filtering.
5. Azimuth IFFT.
6. Focused complex image extraction.

Azimuth Doppler rate:

```text
K_a = 2 * v^2 / (lambda * R_0)
```

Azimuth matched filter:

```text
H_az(f_a) = exp(j * pi * f_a^2 / K_a)
```

RCMC offset:

```text
delta_r_cm(f_a) = lambda^2 * R_0 * f_a^2 / (8 * v^2)
delta_n_cm(f_a) = delta_r_cm(f_a) / delta_r
```

RCMC interpolation:

- Phase 1 may use linear interpolation for the first working implementation.
- A sinc interpolation implementation may be added after the linear baseline has passing tests.
- Out-of-range samples must be zero-filled and counted in diagnostics.

The implementation must expose diagnostic metadata:

```cpp
struct SarRdaDiagnostics {
  double reference_range_m;
  double doppler_rate_hz_per_s;
  double range_bin_spacing_m;
  std::size_t rcmc_out_of_bounds_samples;
  bool used_linear_rcmc;
};
```

## 11. Phase Reference Contract

Phase 1 may apply only a constant scene-center reference:

```text
I_ref = I * exp(-j * 4*pi*R_0 / lambda)
```

This can align a global phase convention. It must not be documented or tested as a substitute for:

- RCMC.
- Motion compensation.
- Autofocus.
- Pixel-dependent residual phase correction.

## 12. Pulse Ring Buffer Contract

Required behavior:

- `push(pulse_id, echo)` rejects negative pulse IDs.
- Duplicate pulse IDs are rejected.
- `popRange(start_id, end_id)` succeeds only when every pulse ID in the inclusive range is present.
- `popLatestN(n)` succeeds only when the latest `n` pulse IDs are continuous.
- Overflow sets a sticky overflow flag until `clear()`.
- `clear()` resets data, overflow flag, and continuity diagnostics.

The buffer must report:

```cpp
struct PulseRingBufferStatus {
  int capacity_pulses;
  int size_pulses;
  bool overflowed;
  int64_t oldest_pulse_id;
  int64_t newest_pulse_id;
  std::size_t dropped_pulse_count;
};
```

Thread safety:

- Single writer and single reader is the Phase 1 target.
- Multi-producer and multi-consumer behavior is out of Phase 1.
- Methods may be mutex-protected, but no public API should expose locks.

## 13. Quality Metrics Contract

Phase 1 must compute these metrics for fixed point-target scenes:

| Metric | Required | Notes |
|---|---|---|
| Peak location error | Yes | In range bins and azimuth bins |
| Range 3 dB width | Yes | Converted to meters by `c/(2f_s)` |
| Azimuth 3 dB width | Yes | Converted by platform spacing or scene pixel spacing |
| PSLR | Yes | Method must state mainlobe estimator |
| ISLR | Yes | Method must state mainlobe estimator |
| Image entropy | Yes | For regression tracking, not alone as pass/fail |
| Radiometric accuracy | No | Phase 4 item |

The metrics must be deterministic for the same configuration and random seed.

## 14. Test Data Contract

Required Phase 1 reference scenes:

1. `sar_point_single_center`
   - One target at scene center.
   - Validates delay, compression, and peak location.

2. `sar_point_three_separated`
   - Three targets separated in range and azimuth.
   - Validates no target merging and relative peak order.

3. `sar_point_near_edge`
   - One target near valid range boundary.
   - Validates clipping and diagnostics.

4. `sar_ring_buffer_fractional_prf`
   - Macro step and PRF chosen so `dt * prf` is fractional.
   - Validates fractional pulse carry and pulse ID continuity.

Reference data should be generated by a repo-local deterministic script and checked into `tests/reference/sar/` or regenerated in tests with exact config values.

## 15. Acceptance Gates

Phase 1 is not complete until all gates pass:

1. Public include contract builds with `#include "1q/sar/sar.hpp"`.
2. SAR sources are split into engine/core groups in `src/sar/CMakeLists.txt`.
3. `SarSession::StepWithResult()` returns structured success/failure diagnostics.
4. Trace and replay can serialize `SarCycleInput`, `SarCycleResult`, `SarSessionConfig`, and `SarRuntimeConfigPatch`.
5. LFM and matched-filter unit tests pass.
6. Pulse compression unit tests pass.
7. Ring buffer continuity and overflow tests pass.
8. Point-target raw echo tests pass.
9. RDA point-target integration tests pass.
10. No Phase 2+ algorithm is reachable from default config.
11. All SAR engine sources compile with C++11 and Eigen 3.3.9.
12. `SarSession` accepts at most 1024 range samples by 1024 azimuth pulses; larger requests fail
    with the structured size gate.
13. Phase 1 replay stores public summaries only. Focused complex image matrices remain deferred.

## 16. Explicit Non-Goals

Do not implement these during Phase 1 unless this contract is revised:

- Auto algorithm switching.
- GBP/BP/CSA/Omega-K.
- Full scene distributed clutter.
- SAR image detection/classification.
- Absolute RCS inversion.
- HDF5 or GeoTIFF writers.
- GPU or CUDA paths.
- L2/L3 trajectory imaging.
- Motion compensation.
- Autofocus.
- Multi-sensor fusion.
- Maneuver behavior or reconnaissance behavior.

## 17. Resolved Phase 1 Decisions

1. FFT backend: internal facade backed by Conan Eigen unsupported FFT.
2. Compatibility gate: C++11 SAR engine compilation with Eigen 3.3.9 on the current platform;
   Windows/VS2015 is not mandatory for Phase 1.
3. Namespace: `sar::...`.
4. Replay: public summaries only; full focused complex matrices and external artifacts are deferred.
5. Runtime and performance ceiling: 1024x1024. Larger sizes remain gated until a later phase.
6. RCMC: linear interpolation only in Phase 1.

## 18. Source Alignment

This contract refines:

- `sar_construction_scheme_complete.md`
- `SAR_MODULE_DESIGN.md`

If either source document conflicts with this contract for Phase 1, this contract should control until the source document is revised.
