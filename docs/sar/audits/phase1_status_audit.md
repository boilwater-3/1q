# SAR Phase 1 现状审计报告

Date: 2026-06-23

## 1. 审计目的

`module_design.md` 把 Phase 1(LFM + 匹配滤波 + 距离压缩 + 点目标回波 + RDA + 脉冲缓冲区)
标为 `✅ 完成(最小可审批闭环达成)`,完成度矩阵第 31-42 行逐项标 ✅。本审计逐条比对该
描述与 `src/sar/{signal,geometry,echo,runtime,imaging}/` 的实际实现,识别过时/不准确标记。
本审计不修改代码。

## 2. 审计结论

Phase 1 **功能交付完整,但文档中有 2 处"重构承诺"与实际代码不符(均为已宣称但未落地的
跨模块提炼),1 处 API 可见性措辞偏差,2 处轻微签名措辞偏差**。其余信号/几何/回波/运行时/
RDA 全部一致。

| 审计层 | 实际状态 | 结论 |
|---|---|---|
| signal(LFM/匹配滤波/距离压缩/窗函数/2D压缩/FFT facade) | 全部存在且签名/字段一致 | ✅ 一致 |
| geometry(L1/L2/L3 轨迹/斜距/多普勒/天线) | 全部存在 | ✅ 一致 |
| echo(点目标/噪声/杂波/场景) | 全部存在,`ApplyFractionalDelay` 已公开 | ✅ 一致 |
| runtime(脉冲环形缓冲区) | 存在,API 命名与设计初稿分叉一致 | ✅ 一致(IsContiguous 可见性除外) |
| imaging RDA(7 步 + sinc/linear RCMC) | 存在且正确 | ✅ 一致 |
| **"sqrt(range²+x²) 提炼到 geometry 层"(L252)** | **未落地** — 仍在 imaging 层 | ❌ **不一致(显著)** |
| **"RDA 内部改调 ComputeDopplerParams"(L265,L338)** | **未落地** — RDA 内联,ComputeDopplerParams 对 RDA 是死代码 | ❌ **不一致(显著)** |

## 3. 信号层实证(`src/sar/signal/`)

全部一致:
- `LfmWaveformConfig`/`LfmWaveform`/`RangeCompressionResult`/`PulseQualityMetrics` 字段逐一匹配(`SarWaveform.h:20-60`)。
- `GenerateLfmWaveform`/`BuildMatchedFilter`(基+窗重载)/`LinearConvolveFft`/`RangeCompress`(基+窗重载)/`EstimatePulseQuality` 签名匹配(`:65-87,117-125`)。
- `WindowType{kNone,kHamming,kHanning,kBlackman,kKaiser}`、`WindowSpec.kaiser_beta=8.6`、`GenerateWindow`(产复数窗,实部置窗值,`:258`)、`RangeAzimuthCompressionConfig`、`Compress2D` 一致。
- `SarFft`:`Fft1D`(forward 不归一化,inverse /N,Eigen 后端)、`FftRows`、`FftCols` 一致。
- `ComplexSample=complex<double>`、`ComplexVector`、`ComplexMatrix`(行主序 + `operator()`)一致。

## 4. 几何层实证(`src/sar/geometry/`)

- 轨迹:`PlatformPulseState`/`StraightStripmapTrackConfig`(L1)/`PerturbedStripmapTrackConfig`(L2)/`WaypointTrackConfig`(L3)/`TrajectoryErrorDiagnostics` 字段一致。
- 三工厂 + `AdvanceFractionalPrf`(分数累积防 floor 漂移)+ `Distance` 一致。
- 斜距:`ExactSlantRange`/`ClosestSlantRange`/`QuadraticApproxRange`/`RangeRate` 一致。
- 多普勒:`DopplerParams`/`DopplerComputationInput`/`ComputeDopplerParams`/`DopplerFrequencyAt`/`AzimuthResolution`/`DopplerBinFrequency` 均存在且自测(`sar_geometry_model_test.cpp:116-167`)。
- `Sinc`、`DeterministicGaussianSampler`(`mt19937(seed)` + 平台无关 Box-Muller,seed 2026 约定)一致。
- 天线:`AntennaParams`/`AntennaGain`/`AzimuthPattern`/`SincPattern`/`SyntheticApertureTime`/`AntennaResolution` 存在。

## 5. 回波层实证(`src/sar/echo/`)

全部一致:
- `PointTarget`/`RawEchoConfig`/`EchoTargetDiagnostic`/`RawEchoResult` 字段一致。
- `GeneratePointTargetRawEcho`(频域分数延迟)、`ApplyFractionalDelay`(**已公开**于 `echo::`,`SarEcho.h:66-67`,被 `GeneratePointTargetRawEcho` 与 `GenerateClutterScene` 复用)一致。
- `NoiseSpec`(seed 默认 2026)/`AddNoise`(实虚各 σ/√2)/`ClutterType`/`ClutterModel`/`GammaClutterRcs`/`SeaClutterRcs`(GIT)/`SceneDescription`/`GenerateClutterScene` 一致。

## 6. 运行时实证(`src/sar/runtime/`)

- 容器 `std::deque<PulseRecord>` 一致。
- API:`Push`/`ReadLatest`/`ReadRange`/`size`/`capacity`/`overflow_sticky` 一致;设计初稿 `popLatestN`/`popRange`/`clear`/`isOverflow` 确认未沿用。

**不一致 3(轻微)**:文档 L318 把 `IsContiguous()` 列为公共 API,实际 `PulseRingBuffer.h:39` 它是 **private**(仍强制 pulse_id 连续,门控 `ReadRange`/`ReadLatest`,但非公共可调用)。

## 7. RDA 实证(`src/sar/imaging/SarRda.*`)

- `RcmcInterpolation{kNone,kLinear,kSinc}`、`RdaConfig.rcmc_interpolation` 默认 `kLinear`、`sinc_half_width` 默认 4 一致。
- `FocusStripmapRda`(7 步:逐行 RangeCompress → ApplyBroadsideCenterPhaseReference → FftCols → RCMC → 方位匹配滤波 → FftCols 逆 → 质量评估)一致。
- `ApplyRangeMigrationCorrection`(kSinc = Lanczos 加窗 `Sinc(d)*Sinc(d/half_width)`,`SarRda.cpp:64-65`)一致。
- `ComputeRdaSamplingDiagnostics` 存在(`:99-138`)。

## 8. 发现的不一致

### 不一致 1 — ❌ 显著:`sqrt(range²+x²)` 提炼目标层错误(文档 L252)

**文档声明**(L252):
> 实施时把 `SarRda.cpp` 内 `ApplyBroadsidePhaseReference` 的 `sqrt(range²+x²)` 几何提炼到**本层(geometry)**,RDA 改调用,消除重复。

**实际代码**:
- `sqrt(range²+x²)` 公式仍在 **imaging 层**,位于 `src/sar/imaging/SarPhaseReference.cpp:71`:
  ```cpp
  const double slant_m = std::sqrt(range_m * range_m + x_m * x_m);
  ```
- 该函数已从 `SarRda.cpp` 抽出到独立模块 `SarPhaseReference.{h,cpp}`(imaging 层),并更名为 `ApplyBroadsideCenterPhaseReference` —— 这一步提炼确实做了。
- 但 **geometry 层没有任何函数计算此 broadside-slant 形式**;`ExactSlantRange` 只是纯 3D 欧氏 `Distance` 包装,非 `sqrt(range_col²+x_slowtime²)` 形式。
- 已 grep 确认:`SarPhaseReference.cpp:66-72` 持有该几何,geometry 无对应。

**结论**:提炼确实发生(抽成 `SarPhaseReference` 模块),但**目标层描述错误** —— 提炼到的是 imaging 层的 `SarPhaseReference`,不是 geometry 层。文档 L252 应改为"提炼到 imaging 层的 `SarPhaseReference` 模块"。

### 不一致 2 — ❌ 显著:RDA 未改调 `ComputeDopplerParams`(文档 L265, L338)

**文档声明**:
- L265:> 实施时把 `SarRda.cpp::ComputeRdaSamplingDiagnostics` 内的多普勒计算提炼到 `ComputeDopplerParams`,RDA 公开签名不变,内部改调用(`sar_rda_test.cpp` 回归兜底)。
- L338:> 批次2 后内部改调 `geometry::ComputeDopplerParams`。

**实际代码**(已双重确认):
- `SarRda.cpp` 中 grep `ComputeDopplerParams` = **零命中**。
- `ComputeRdaSamplingDiagnostics`(`SarRda.cpp:111-113`)**内联**了多普勒率:
  ```cpp
  diagnostics->doppler_rate_hz_per_s =
      2.0 * config.platform_velocity_mps * config.platform_velocity_mps /
      (wavelength_m * config.reference_range_m);
  ```
- `SarRda.cpp` 中 `geometry::` 仅 3 处:`Sinc`(`:65`)、`DopplerBinFrequency`(`:228,243`)—— **无 `geometry::ComputeDopplerParams`**。
- `ComputeDopplerParams` 的全局引用只在 4 处:`docs/sar/design/module_design.md`、`tests/unit/sar_geometry_model_test.cpp`、`src/sar/geometry/SarGeometry.{h,cpp}`。**对 RDA 而言它是死代码**,仅被自身单元测试调用。

**结论**:文档描述的重构(多普勒计算提炼到 `ComputeDopplerParams` 并由 RDA 内部调用)**从未接线**。RDA 仍内联 `2v²/(λR0)`,与 `ComputeDopplerParams` 存在重复实现。文档 L265/L338 应标注此重构未落地,或代码应补接线(后者需单独审批,本审计不授权)。

### 不一致 3 — 轻微:`IsContiguous()` 可见性(文档 L318)

文档把 `IsContiguous()` 列为公共 API,实际 `PulseRingBuffer.h:39` 为 private。连续性仍强制(门控 Read*),但非公共可调用。

### 不一致 4 — 轻微:`AntennaResolution` 签名措辞(文档 L283)

文档 L283 描述 `AntennaResolution(antenna, synthetic_aperture)`;实际为 `AntennaResolution(antenna, slant_range_m, wavelength_m, bool synthetic_aperture)`(4 参数)。函数存在且行为一致,仅文档省略了 2 个参数。

### 不一致 5 — 轻微:`Compress2D` FFT 复用措辞(文档 L218)

文档 L218 称 `Compress2D` "复用 `FftRows`/`FftCols`";实际 `Compress2D` 仅用 `FftCols`(距离向由 `RangeCompress`→`Fft1D` 处理,非 `FftRows`)。功能等价,措辞不精确。

## 9. 处置建议

- **不一致 1、2(显著)**:建议修正 `module_design.md` L252、L265、L338 的措辞,如实反映:
  - L252:`sqrt(range²+x²)` 提炼到 **imaging 层 `SarPhaseReference` 模块**(非 geometry)。
  - L265/L338:多普勒计算提炼到 `ComputeDopplerParams` **未落地**;RDA 仍内联 `2v²/(λR0)`;`ComputeDopplerParams` 当前仅几何层自测使用,对 RDA 是死代码。
  - 这是纯文档修正,不触碰代码。
- **不一致 3、4、5(轻微)**:可选修正措辞;非阻塞。

## 10. 不一致性优先级

| 优先级 | 不一致 | 影响 |
|---|---|---|
| 高 | L265/L338 RDA 改调 ComputeDopplerParams 未落地 | 文档宣称的重构未执行,存在重复实现 + 死代码 |
| 高 | L252 sqrt 几何提炼目标层错误 | 文档误导读者去 geometry 层找该公式 |
| 低 | IsContiguous 可见性 | 措辞,功能正确 |
| 低 | AntennaResolution 签名 | 措辞,功能正确 |
| 低 | Compress2D FFT 复用 | 措辞,功能等价 |

## 11. 本审计的非目标

- 不修改任何 C++ 源代码(含不补接 RDA→ComputeDopplerParams 的接线)。
- 不变更冻结清单或公共 API。
- 不构成 RDA 内部重构授权(改接线需单独审批)。
- 不外推到 Phase 3/4/5。
