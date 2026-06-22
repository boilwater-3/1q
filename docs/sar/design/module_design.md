# SAR 仿真模块设计方案

## 背景与目标

在现有 1Q 仿真模型库(机载雷达、电子侦察雷达等)基础上,新增 **合成孔径雷达 (SAR) 成像仿真** 能力,实现从回波仿真到图像形成的完整处理链路。

### 设计目标

1. 支持条带模式 (Stripmap) SAR 成像仿真
2. 兼容现有 1Q 公共基础设施(几何变换、大气模型、数值求解器、RCS 计算等)
3. 提供可扩展的算法框架,后续可扩展聚束 (Spotlight)、扫描 (Scan)、滑动聚束 (Sliding Spotlight) 等模式
4. 保证工程级精度与鲁棒性
5. **快慢时间异步解耦**:PRF 与平台宏观步长独立设计,通过环形脉冲缓冲区实现跨步长拼接
6. **聚焦算法受控选择**:Phase 1 固定 RDA;后续在 GBP/BP 等扩展算法完成回归测试和性能基准后,再启用自动选择 —— 其中自适应选择(Auto)已标注【未进行设计需求,不再扩展】
7. **相位基准统一**:支持跨算法 / 跨合成孔径图像的相位重参考,避免下游处理伪影
8. **辐射定标闭环**:通过标定目标反推定标常数,支持绝对 RCS 反演与辐射精度评估 —— 已标注【未进行设计需求,不再扩展】
9. **轨迹保真度分级**:L1~L3 三级保真度适配不同场景需求

> **文档性质说明**:本文件描述 SAR 模块的**接口与实现形态**。为避免设计稿代码块与实际实现脱节,本文档不再嵌入 C++ 代码段;各模块以「struct 字段表 + 自由函数清单 + 实施状态」的描述性形态呈现。实际签名以 `src/sar/` 与 `include/1q/sar/` 下的头文件为准。算法层面的数学公式与审批口径见 `construction_scheme.md`。

---

## 实施状态总览

### 完成度矩阵

| 层 | 模块 | 实施状态 | 位置 |
|---|---|---|---|
| 公共契约 | 对外会话 (`SarSession`/Config/Input/Result/Patch) | ✅ 完整(超设计:含 Trace/Replay/Factory) | `include/1q/sar/session/`、`src/sar/session/` |
| 公共契约 | 公共算法类型层 (`sar_types.h`) | ❌ 废弃(被四域配置取代,见下) | — |
| signal | LFM 波形 / 匹配滤波 / 距离压缩 | ✅ 完整 | `src/sar/signal/SarWaveform.*` |
| signal | 窗函数 / 2D 脉冲压缩 | ✅ 已实现 | `src/sar/signal/SarWaveform.*` |
| signal | FFT facade | ✅ 完整 | `src/sar/signal/SarFft.*` |
| geometry | 平台轨迹 L1/L2/L3 | ✅ 完整 | `src/sar/geometry/SarGeometry.*` |
| geometry | 斜距模型 | ✅ 已实现 | `src/sar/geometry/SarGeometry.*` |
| geometry | 多普勒模型 | ✅ 已实现 | `src/sar/geometry/SarGeometry.*` |
| geometry | 天线方向图 | ✅ 已实现 | `src/sar/geometry/SarAntenna.*` |
| echo | 点目标原始回波 | ✅ 完整 | `src/sar/echo/SarEcho.*` |
| echo | 杂波 / 噪声 / 场景 | ✅ 已实现 | `src/sar/echo/SarEcho.*` |
| runtime | 脉冲环形缓冲区 | ✅ 完整 | `src/sar/runtime/PulseRingBuffer.*` |
| runtime | 慢时间重采样(超设计) | ✅ 完整 | `src/sar/imaging/SarSlowTimeResampling*` |
| imaging | RDA 聚焦 | ✅ 完整(7 步 + sinc/线性 RCMC) | `src/sar/imaging/SarRda.*` |
| imaging | GBP / BP | ✅ 完整(共享内核,128 上限) | `src/sar/imaging/SarGbp.*` |
| imaging | 相位重参考 | ✅ 完成(RDA 抽模块 + 跨算法全局相位对齐 + public 标量摘要;公共策略后置) | `src/sar/imaging/SarPhaseReference.*` / `SarImageQuality.*` |
| imaging | 成像质量评估 | ✅ 完成(点目标/米制/对比度/相干比较 + public/replay/trace 标量摘要;产品 QA 后置) | `src/sar/imaging/SarImageQuality.*` |
| imaging | 运动补偿(一阶) | ✅ 完整 | `src/sar/imaging/SarMotionCompensation.*` |
| imaging | 自聚焦 PGA(估计+真值) | ✅ 完整(无 applyCorrection 闭环) | `src/sar/imaging/SarPga*` |
| calibration | 辐射定标 | 🔒 完整实现但冻结(见孤儿文件) | `src/sar/calibration/SarRadiometricCalibration.*` |
| output | 图像输出(Binary / GeoTIFF sidecar / HDF5) | ✅ 已实现(HDF5 条件编译) | `src/sar/output/ImageFormatter.*` |

图例:✅ 完整且纳入构建 · 🟡 部分或实施中 · 🔒 完整实现但被冻结排除构建 · ❌ 废弃/缺失

### 孤儿文件状态(已完整实现但冻结排除构建)

下列文件**均有实质实现(非空壳)**,但被 `src/sar/CMakeLists.txt` 的 `SAR_ENGINE_SOURCES` 排除,且无任何已构建目标引用、无对应测试、公共入口 `sar.hpp` 不暴露。它们内部相互 include 形成自洽闭环,对外完全孤立。详见文末「孤儿文件状态」章节。

- **Omega-K 全管线**(14 文件):Stolt 插值、参考映射、相位补偿、网格缩减、方位逆变换、点目标验收,生产级完整;`SarOmegaKTruth*` 系列含自实现 SHA-256(160 行)的验收链。
- **CSA**:仅几何 + 中间态真值,主流程未实现(半成品)。
- **`SarFocusingSelector`**:L1/L2/L3 × 3 目的的路由器(刻意不含 CSA/OmegaK 选项)。
- **`SarRadiometricCalibration`**:单/多目标定标 + RCS 反演 + 执行流水线。

---

## 架构约定

### 编码风格

实际 SAR 代码统一采用 **自由函数 + POD struct** 风格(非设计初稿的 OOP 类继承体系):

- 函数返回 `bool` 表示成功,输出参数放末尾 `T* output`(对齐 `GenerateLfmWaveform`、`FocusStripmapRda` 等)。
- 配置以 POD struct 聚合,字段带物理量单位后缀。
- 文件内部匿名命名空间放常量(`kPi`、`kSpeedOfLightMps`)与 helper。
- 绝不向新 API 引入 `Eigen::MatrixXcd`;2D 复数运算统一用 `sar::signal::ComplexMatrix`。

### 命名空间分层

| 子命名空间 | 职责 |
|---|---|
| `sar::signal` | FFT facade、LFM 波形、匹配滤波、距离压缩、窗函数 |
| `sar::geometry` | 本地坐标、平台轨迹、斜距、多普勒、天线方向图 |
| `sar::echo` | 点目标回波、杂波、噪声、场景描述 |
| `sar::imaging` | RDA / GBP / BP 聚焦、质量评估、运动补偿、PGA、慢时间重采样 |
| `sar::output` | 图像序列化(Binary / GeoTIFF sidecar / HDF5) |
| `sar::runtime` | 脉冲环形缓冲区 |
| `sar::session` / `sar::config` | 对外会话契约与四域配置(公共层) |

### 统一数据类型

| 类型 | 定义 | 用途 |
|---|---|---|
| `sar::signal::ComplexSample` | `std::complex<double>` | 复数样本 |
| `sar::signal::ComplexVector` | `std::vector<ComplexSample>` | 1D 复向量(脉冲/波形) |
| `sar::signal::ComplexMatrix` | `{rows, cols, ComplexVector values}` 行主序,带 `operator()(r,c)` | 2D 复矩阵(回波历史/聚焦图像) |
| `sar::geometry::LocalPoint` | `{x_m, y_m, z_m}`(x=azimuth, y=ground range, z=altitude) | 本地 Cartesian 点 |
| `sar::geometry::PlatformPulseState` | `{pulse_id, time_s, position_m, velocity_x/y/z_mps}` | 单脉冲平台状态 |

### 物理量单位后缀(强约定)

`_hz` / `_mps` / `_m` / `_s` / `_m2` / `_db` / `_rad` / `_dbsm`。

### 确定性随机

沿用 `std::mt19937` + 平台无关 Box-Muller + 固定整数 seed(SAR 内约定值 2026),禁用 `std::random_device`,保证跨平台可重现。

### 头文件边界

- **内部头**:`src/sar/` 下,include guard 形式 `ONEQ_SRC_..._H_`,仅供模块内部引用。
- **公共头**:`include/1q/sar/` 下,`ONEQ_API` 导出,经 `sar.hpp` 聚合。优先内部实现,不轻易 widening 公共层。

---

## 目录结构(实际)

```text
include/1q/sar/
├── sar.hpp                         模块统一入口
├── config/
│   ├── sar_config.hpp              配置聚合入口
│   ├── SarSessionConfig.h          四域配置聚合(ONEQ_API)
│   ├── SarHardwareConfig.h         硬件域(载频/带宽/脉宽/PRF/采样率/功率/天线)
│   ├── SarMissionConfig.h          任务域(采样数/L2 扰动/L3 航路点)
│   ├── SarPolicyConfig.h           策略域(布尔开关:enable_raw_echo/enable_range_compression/enable_l1_rda/enable_l3_bp/retain_*)
│   ├── SarEnvironmentConfig.h      环境域
│   └── SarRuntimeConfigPatch.h     运行期可变配置补丁(has_* + 值)
└── session/
    ├── SarCycleInput.h             单周期输入(平台状态/点目标/raw IQ/时间步长)
    ├── SarCycleResult.h            单周期结果(SarOutputFrame/SarFocusedImage/诊断)
    ├── SarSession.h                PIMPL 门面:Step/StepWithResult/ApplyRuntimeConfig
    ├── SarSessionFactory.h         会话工厂
    ├── SarTraceSession.h           trace 录制会话
    └── SarReplaySession.h          replay 重放会话

src/sar/
├── signal/   SarFft.{h,cpp}  SarWaveform.{h,cpp}
├── geometry/ SarGeometry.{h,cpp}  SarAntenna.{h,cpp}(批次4)
├── echo/     SarEcho.{h,cpp}
├── imaging/  SarRda  SarGbp  SarImageQuality  SarMotionCompensation
│             SarPga*  SarAutofocusPhaseTruth  SarSlowTimeResampling*
│             (冻结:SarCsa*  SarOmegaK*  SarFocusingSelector)
├── calibration/ (冻结:SarRadiometricCalibration.{h,cpp})
├── output/   ImageFormatter.{h,cpp}(批次5)
├── runtime/  PulseRingBuffer.{h,cpp}
└── session/  SarSession(调度骨架)  SarRawHistoryBuilder  SarRuntimeConfigValidation
              SarImagingExecutor  SarFocusedImageAssembler  SarTraceSession
              SarReplaySession  SarReplayFlatbufferCodec  generated/
              (SarSources.cmake 集中维护源清单,供生产目标与合同测试复用)

tests/
├── unit/        sar_*.cpp(GLOB_RECURSE 自动收集)
├── performance/ sar_fft_performance_test.cpp
├── contract/    check_sar_cxx11_compat.cmake
└── support/     sar_reference_scene.h(确定性参考场景构造器)
examples/sar/    session_usage.cpp
```

---

## 公共类型与对外会话契约

### 公共算法类型层(初稿废弃)

设计初稿(`sar_types.h`)曾定义 `FocusingAlgorithm` / `PhaseReferenceMode` / `MainlobeEstimationMethod` / `TrajectoryFidelity` / `SarSimulationConfig` 等公共枚举与单体配置。**实施时该层被废弃**,改为:

- 公共对外**不暴露算法选择枚举**,改由 `SarPolicyConfig` 的一组布尔开关表达(`enable_l1_rda_imaging`、`enable_l3_bp_imaging`、`enable_range_compression` 等)。其中 `enable_range_compression` 当前 Phase 1 不产出独立可消费的距离压缩产物,真实距离压缩在 RDA / BP 内部完成;该开关作为 L3 BP 的前置条件门(参见 `l3_bp_session_integration` 契约),并在启用时触发 `SarProcessingStage::kRangeCompression` 阶段标记与 `has_range_compressed_echo` 摘要(前置条件摘要,非独立输出载荷)。语义详见 `SarPolicyConfig.h` / `SarCycleResult.h` 注释。
- 保真度分级不再有公共枚举,由 `SarMissionConfig` 的 `l2_*`、`l3_waypoints` 字段隐式表达。
- 等价枚举仅存在于内部 `src/sar/imaging/SarFocusingSelector.h`(`RecommendedFocusingAlgorithm`、`SelectorTrajectoryFidelity`),且该选择器已被冻结(见孤儿文件)。
- 单体 `SarSimulationConfig` 被**四域配置**(`SarHardwareConfig` + `SarMissionConfig` + `SarPolicyConfig` + `SarEnvironmentConfig`)取代。

> 若后续需要把算法选择作为对外稳定契约(进入配置/日志/回放),应在 `include/1q/sar/` 下补一个公共类型头并对齐枚举值。当前以布尔开关形态交付。

### 对外会话契约(✅ 完整,超设计)

| 契约 | 形态 | 说明 |
|---|---|---|
| `sar.hpp` | 模块入口 | 聚合 config + 全部 session 头 |
| `SarSessionConfig` | 四域聚合 struct(`ONEQ_API`) | hardware/mission/policy/environment |
| `SarCycleInput` | 单周期输入(`ONEQ_API`) | `SarPlatformState`(10 字段)、`SarPointTarget`、`SarRawIqFrame`、`cycle_index`、`dt_sec`;运行期指令通过 `ApplyRuntimeConfig` 旁路注入 |
| `SarCycleResult` | 单周期结果(`ONEQ_API`) | `SarOutputFrame`(状态/采样数/斜距/SNR/各阶段 has_* 标志 + 相位参考/图像质量标量摘要)、`SarFocusedImage`(行主序双 vector<double> 实虚分离 + `is_placeholder` + source)、`SarDiagnosticIssueList`、`has_error`/`executed_this_cycle`/`reused_previous_output`/`abort_reason` |
| `SarSession` | PIMPL 门面(`ONEQ_API`) | 私有构造 + `friend SarSessionFactory`;`Step()` / `StepWithResult()` / `ApplyRuntimeConfig()` / `TryApplyRuntimeConfig()` |
| `SarRuntimeConfigPatch` | 运行期补丁(`ONEQ_API`) | `has_*` + 值的可选字段模式,覆盖 `SarPolicyConfig` 子集(含 `retain_focused_image`),不暴露内部算法对象 |

**超设计交付**:`SarSessionFactory`(强制走工厂)、`SarTraceSession`(trace 录制,写 `TraceSink` + `ReplayTraceWriter`)、`SarReplaySession`(重放,`ReplaySarTrace()`)、`SarReplayFlatbufferCodec`(FlatBuffers 编解码)。整条 trace/replay 链已落地,设计初稿未列。

### SarFocusedImage 数据形态

聚焦复图像在公共层以**实/虚双 `std::vector<double>`** 拆分行主序存储(`index = row*column_count + col`),非 `vector<complex<double>>`、非 Eigen。`SarPolicyConfig::retain_focused_image=false` 时 `is_placeholder=true`,实虚数组为空,仅形状有效,用于避免大图拷贝;该策略可通过 `SarRuntimeConfigPatch` 运行期切换,并已纳入 session replay 配置 schema。内部计算态用 `sar::signal::ComplexMatrix`;按策略导出的 `ExportFocusedImage`/`CopyFocusedImage` 由 `SarImagingExecutor` 持有(`SarSession` 仅编排,不再直接负责导出)。

---

## 信号层 (`sar::signal`)

### LFM 波形 / 匹配滤波 / 距离压缩(✅ 完整)

位于 `src/sar/signal/SarWaveform.*`。

| 实体 | 字段 / 签名 |
|---|---|
| `LfmWaveformConfig` | `bandwidth_hz`、`time_bandwidth_product`、`sample_rate_hz`、`start_frequency_hz` |
| `LfmWaveform` | `config`、`pulse_width_s`、`chirp_rate_hz_per_s`、`samples` |
| `RangeCompressionResult` | `full_convolution`、`range_aligned_output`、`range_bin_spacing_m`、`full_peak_index`、`aligned_peak_index` |
| `PulseQualityMetrics` | `peak_index`、`peak_magnitude`、`main_lobe_start/end`、`width_3db_bins`、`width_20db_bins`、`pslr_db`、`islr_db` |
| `GenerateLfmWaveform(config, waveform*)` | 生成基带复 LFM(由 BT 积推导脉宽,无载频上变频) |
| `BuildMatchedFilter(waveform, filter*)` | 时域 `h[n]=conj(s[N-1-n])` |
| `LinearConvolveFft(input, filter, output*)` | 补零 FFT 线性卷积 |
| `RangeCompress(input, matched_filter, sample_rate_hz, result*)` | 距离向脉冲压缩 |
| `EstimatePulseQuality(compressed_pulse, metrics*)` | 3dB/20dB 主瓣宽度 + PSLR/ISLR |

### 窗函数 / 2D 脉冲压缩(✅ 已实现)

| 实体 | 字段 / 签名(目标) |
|---|---|
| `WindowType` | `kNone`/`kHamming`/`kHanning`/`kBlackman`/`kKaiser` |
| `WindowSpec` | `type`、`kaiser_beta`(默认 8.6) |
| `GenerateWindow(spec, length, window*)` | 产复数窗(实数窗提升) |
| `BuildMatchedFilter(waveform, window, filter*)` | 加窗重载(原签名保留) |
| `RangeCompress(input, matched_filter, sample_rate_hz, window, result*)` | 加窗重载(RDA 仍走 kNone) |
| `RangeAzimuthCompressionConfig` | `sample_rate_hz`、`prf_hz`、`range_window`、`azimuth_window`、`azimuth_matched_filter_rate_hz_per_s` |
| `Compress2D(raw_history, range_matched_filter, config, output*)` | 距离+方位双向压缩,复用 `FftRows`/`FftCols` |

### FFT facade(✅ 完整)

`SarFft.*` 提供 `Fft1D`(forward 不归一化,inverse 除以 N)、`FftRows`、`FftCols`,后端 Eigen `unsupported/Eigen/FFT` 完全封装。

---

## 几何层 (`sar::geometry`)

### 平台轨迹 L1/L2/L3(✅ 完整)

位于 `src/sar/geometry/SarGeometry.*`。

| 实体 | 字段 / 签名 |
|---|---|
| `PlatformPulseState` | `pulse_id`、`time_s`、`position_m`、`velocity_x/y/z_mps` |
| `StraightStripmapTrackConfig`(L1) | `start_position_m`、`velocity_x_mps`、`prf_hz`、`first_pulse_id`、`pulse_count` |
| `PerturbedStripmapTrackConfig`(L2) | `ideal` + 三向速度误差 stddev + `initial_position_error_m` + `random_seed` |
| `WaypointTrackConfig`(L3) | `waypoints`、`pulse_times_s`、`first_pulse_id` |
| `TrajectoryErrorDiagnostics` | L2 的 max/RMS 位置与速度误差 |
| `GenerateStraightStripmapTrack` / `GeneratePerturbedStripmapTrack` / `GenerateWaypointTrack` | 三工厂 |
| `AdvanceFractionalPrf` | 分数 PRF 累积,避免 floor 截断漂移 |
| `Distance(a, b)` | 3D 欧氏距离 |

### 斜距模型(✅ 已实现)

| 函数(目标) | 说明 |
|---|---|
| `ExactSlantRange(platform, target)` | 精确斜距(复用 `Distance`) |
| `ClosestSlantRange(track, target)` | 最近斜距 R0 |
| `QuadraticApproxRange(approx, time_s)` | 二次近似 `R(t)≈R0+(v·Δt)²/(2R0)` |
| `RangeRate(platform, target)` | 斜距变化率 `dR/dt` |

> 实施时把 `SarRda.cpp` 内 `ApplyBroadsidePhaseReference` 的 `sqrt(range²+x²)` 几何提炼到本层,RDA 改调用,消除重复。

### 多普勒模型(✅ 已实现)

| 实体 / 函数(目标) | 说明 |
|---|---|
| `DopplerParams` | `fd_central_hz`、`fd_rate_hz_per_s`、`synthetic_aperture_time_s`、`doppler_bandwidth_hz` |
| `DopplerComputationInput` | `wavelength_m`、`platform_velocity_mps`、`reference_slant_range_m`、`squint_angle_rad`、`real_aperture_length_m` |
| `ComputeDopplerParams(input, params*)` | `Ka = 2v²/(λR0)` 等 |
| `DopplerFrequencyAt(params, slow_time_s)` | 瞬时多普勒 |
| `AzimuthResolution(params, v)` | `ρ_az = v/B_doppler` |
| `DopplerBinFrequency(index, count, prf_hz)` | FFT bin → 多普勒频率(提炼自 RDA) |

> 实施时把 `SarRda.cpp::ComputeRdaSamplingDiagnostics` 内的多普勒计算提炼到 `ComputeDopplerParams`,RDA 公开签名不变,内部改调用(`sar_rda_test.cpp` 回归兜底)。

### 数学工具与采样器(✅ 已实现)

| 实体(目标) | 说明 |
|---|---|
| `Sinc(x)` | 归一化 sinc `sin(πx)/(πx)`,提炼自 RDA 匿名,供天线 sincPattern 与 RCMC 共用 |
| `DeterministicGaussianSampler` | 平台无关 Box-Muller,提炼自 `SarGeometry.cpp` 匿名,供 `echo::AddNoise` 复用;seed→序列映射须保持不变(L2 轨迹测试依赖) |

### 天线方向图(✅ 已实现,新建 `SarAntenna.*`)

| 实体 / 函数(目标) | 说明 |
|---|---|
| `AntennaParams` | `length_m`、`width_m`、`peak_gain_linear`、`beam_width_azimuth_rad`、`beam_width_range_rad`、`boresight_azimuth_rad` |
| `AntennaGain(antenna, wavelength_m)` | `G = 4π A_eff/λ²` |
| `AzimuthPattern(antenna, off_boresight_rad)` | `sinc²(πL sinθ/λ)` |
| `SincPattern(length_m, wavelength_m, off_boresight_rad)` | 复用 `Sinc` |
| `SyntheticApertureTime(antenna, slant_range_m, v)` | `T_sa = R0·θ_bw/v` |
| `AntennaResolution(antenna, synthetic_aperture)` | 真实孔径 `L/2` 或合成孔径 `L/2` |

---

## 回波层 (`sar::echo`)

### 点目标原始回波(✅ 完整)

位于 `src/sar/echo/SarEcho.*`。

| 实体 | 字段 / 签名 |
|---|---|
| `PointTarget` | `position_m`(`LocalPoint`)、`rcs_m2`(线性 m²) |
| `RawEchoConfig` | `sample_rate_hz`、`carrier_frequency_hz`、`range_sample_count` |
| `EchoTargetDiagnostic` | `slant_range_m`、`two_way_delay_s`、`delay_sample_index`、`fractional_delay_samples`、`clipped`、`clipped_samples` |
| `RawEchoResult` | `samples`、`diagnostics`、`has_clipping` |
| `GeneratePointTargetRawEcho(config, platform, targets, transmit_waveform, result*)` | 单脉冲逐目标叠加,频域分数延迟(`ApplyFractionalDelay`) |

### 杂波 / 噪声 / 场景(✅ 已实现)

| 实体 / 函数(目标) | 说明 |
|---|---|
| `NoiseSpec` | `signal_to_noise_ratio_db`、`random_seed`(默认 2026) |
| `AddNoise(spec, samples*)` | 复高斯噪声,复用 `DeterministicGaussianSampler`(实虚各 σ/√2) |
| `ClutterType` | `kGamma` / `kSea` |
| `ClutterModel` | `type`、`gamma_constant`、`sea_state`、`wind_speed_mps`、`incidence_angle_rad`、`resolution_cell_area_m2` |
| `GammaClutterRcs(model)` | `σ = γ·sin(θ)·A_cell` |
| `SeaClutterRcs(model)` | GIT 经验模型 |
| `SceneDescription` | `scene_center`、`scene_extent_x/y_m`、`point_targets`、`clutter`、`clutter_grid_spacing_m` |
| `GenerateClutterScene(config, platform, scene, transmit_waveform, result*)` | 网格化杂波单元逐个叠加,复用提炼后的 `ApplyFractionalDelay` |

> 实施时把 `SarEcho.cpp` 匿名的 `ApplyFractionalDelay` 提炼为 `echo::` 公开 API。

### 脉冲环形缓冲区(✅ 完整,`sar::runtime`)

位于 `src/sar/runtime/PulseRingBuffer.*`。容器为 `std::deque<PulseRecord>`(不加锁,单写单读)。API 与设计初稿命名不同但语义一致:`Push(PulseRecord)`、`ReadLatest(count, output)`、`ReadRange(first_pulse_id, count, output)`、`size()`/`capacity()`/`overflow_sticky()`(粘滞溢出位)、`IsContiguous()`(强制 pulse_id 连续)。设计初稿的 `popLatestN`/`popRange(start,end)`/`clear()`/`isOverflow()` 未沿用。

### 慢时间重采样(✅ 超设计,`sar::imaging`)

`SarSlowTimeResampling*` + `SarSlowTimeResamplingExecutor`。补全快慢时间异步链路最后一环:把时变 PRF / 非均匀慢时间轴 raw history 重采样到标称均匀网格,含丢脉冲诊断(`DiagnoseSlowTimeGaps`,`gap_ratio≥1.5` 拒绝)与结构化拒绝理由。

---

## 成像层 (`sar::imaging`)

### RDA(✅ 完整)

`SarRda.*`。7 步:距离 FFT → 距离匹配滤波 → 距离 IFFT → 方位 FFT → RCMC → 方位匹配滤波 → 方位 IFFT。

| 实体 | 要点 |
|---|---|
| `RdaConfig` | `rcmc_interpolation`(`kNone`/`kLinear`/`kSinc`,**默认 kLinear**)、`sinc_half_width`(默认 4) |
| `RdaDiagnostics` | `doppler_rate_hz_per_s`、`max_geometric_doppler_hz`、`doppler_nyquist_margin`、`image_entropy_nats`、`out_of_bounds_samples` 等 |
| `FocusStripmapRda` | 主聚焦入口 |
| `ApplyRangeMigrationCorrection` | RCMC 独立公开 API,Lanczos 加窗 sinc |
| `ComputeRdaSamplingDiagnostics` | 多普勒/采样诊断(批次2 后内部改调 `geometry::ComputeDopplerParams`) |

### GBP / BP(✅ 完整)

`SarGbp.*`。GBP 与 BP 共享 `FocusSmallSceneBackprojection` 内核,通过 `BackprojectionTraversal{kPixelMajor/kPulseMajor}` 区分遍历顺序。`kMaxApprovedDimension=128`(小场景上限)。**无多线程、无 GPU**(`setNumThreads`/`setUseGpu` 未实现,Phase 5)。

### 相位重参考(✅ 完成)

`SarPhaseReference.*` 已抽出内部自由函数模块,覆盖当前两个实际使用路径:

| 能力 | 实施状态 | 说明 |
|---|---|---|
| RDA 内部相位基准统一 | ✅ 已实现 | `ApplyBroadsideCenterPhaseReference` 在距离压缩后、方位 FFT 前施加 `exp(j·4π·R/λ)`,参考为场景中心行零多普勒的 broadside 几何;RDA diagnostics 记录 `phase_reference_applied` 与 `phase_reference_mode`。 |
| 跨算法全局相位对齐 | ✅ 已实现 | `EstimateGlobalPhaseOffset` 提供全局常数相位估计;`CompareImagesWithGlobalPhaseReference(reference, candidate)` 继续输出 `phase_offset_rad`/`normalized_rms_error`/`coherent_correlation`,已用于 RDA↔GBP/BP、慢时间重采样、运动补偿、参考场景矩阵测试。 |
| Session 诊断可见性 | ✅ 已实现 | `sar.rda_peak` 诊断字符串包含相位参考模式和是否应用;`SarOutputFrame`、FlatBuffers replay、TraceSink JSON 同步输出标量摘要。 |
| 空/无效输入保护 | ✅ 已实现 | 图像维度不一致、空矩阵、零能量输入返回 `valid=false`,不抛异常。 |
| 独立内部 API | ✅ 已实现 | `PhaseReferenceMode`、`PhaseReferenceConfig`、`PhaseReferenceDiagnostics`、`NeedsPhaseReference` 均为内部 API;public 仅暴露结果摘要枚举,公共配置仍未暴露相位参考策略。 |
| 空间变相位误差处理 | ❌ 明确不做 | 现有比较工具只消除**全局常数相位**;不会也不应掩盖空间变化相位误差,相关行为由 `sar_image_quality_test` 覆盖。 |

**后续口径**:公共配置是否暴露 `PhaseReferenceMode` 需要单独审批;当前不引入设计初稿中的 OOP `PhaseReference` 类。

### 成像质量评估(✅ 完成)

`SarImageQuality.*` 已成为 imaging 内部质量指标入口,并被 RDA/GBP/BP、参考场景矩阵、PRF 重采样和运动补偿测试复用。

| 能力 | 实施状态 | 说明 |
|---|---|---|
| 点目标峰值定位 | ✅ 已实现 | `ImageQualityMetrics` 输出 `peak_row`、`peak_col`、`peak_magnitude`;空矩阵/零能量返回 `valid=false`。 |
| 3dB 分辨率(bin) | ✅ 已实现 | 输出 `range_width_3db_bins`、`azimuth_width_3db_bins`;主瓣边界为峰值所在行/列的 3dB 连通区。 |
| 分辨率米制换算 | ✅ 已实现 | `ImageQualityConfig` 提供 range/azimuth pixel spacing 时输出 `range_resolution_3db_m`、`azimuth_resolution_3db_m`;非法 spacing 不影响 bin 级指标。 |
| 主瓣判定策略枚举 | ✅ 已实现 | `MainlobeEstimationMethod{k3dB,k20dB}` 已实现;IRW 仍需单独真值定义。 |
| PSLR / ISLR | ✅ 已实现 | 以 3dB 主瓣矩形划分主瓣/旁瓣,输出 `pslr_db`、`islr_db`。 |
| 图像熵 | ✅ 已实现 | 基于全图功率归一化概率计算 `entropy_nats`。 |
| 图像对比度 | ✅ 已实现 | 基于幅度均值和标准差输出 `image_contrast`。 |
| 跨图像相干比较 | ✅ 已实现 | `ImageComparisonMetrics` 输出 `phase_offset_rad`、`normalized_rms_error`、`coherent_correlation`,并归一化全局幅度尺度。 |
| Session 诊断可见性 | ✅ 已实现 | `sar.rda_peak` 诊断字符串包含 range/azimuth 米制分辨率与 `image_contrast`;`SarOutputFrame`、FlatBuffers replay、TraceSink JSON 同步输出质量标量摘要。 |
| 全图/场景级产品 QA | ❌ 后置 | 暂无 public `ComputeImageMetrics`/产品级 QA 入口;多目标汇总、峰值内存、产品级质量报告未闭环。 |
| 辐射精度指标 | ❌ 冻结缺口 | `radiometric_accuracy_db` 依赖冻结的辐射定标链,当前不纳入继续扩展。 |

**后续口径**:公共结果摘要已通过 `SarOutputFrame`/replay/trace 完成;公共质量配置、产品级 QA 报告与辐射精度仍需单独审批。

### 运动补偿(✅ 一阶完整)

`SarMotionCompensation.*`(位于 imaging 而非 calibration)。`ApplyFirstOrderMotionCompensation`:逐脉冲算 ΔR、包络线性插值重采样 + 相位校正 `exp(j·4π·ΔR/λ)`,带 RMS/max 诊断。**二阶/高阶缺失**,无 IMU 低通滤波。

### 自聚焦 PGA(✅ 估计+真值完整)

`SarPgaPhaseGradientEstimator` + `SarPgaSupportGradientTruth` + `SarPgaGradientTruthComparison` + `SarAutofocusPhaseTruth`。相邻样本共轭乘积取 `arg()` 得 wrapped 梯度,带 support_mask/门槛;真值链注入常数/线性/二次/三次相位并最小二乘分离不可观测分量。**缺**:统一 `Autofocus` 类、`applyCorrection` 闭环、`MapDrift`/`ContrastOptimization` 方法。

### 冻结项(见孤儿文件)

CSA、Omega-K、`SarFocusingSelector`(Auto 选择)均完整实现但被冻结排除构建。

---

## 输出层 (`sar::output`)

### 图像序列化(✅ 已实现:Binary+GeoTIFF sidecar+HDF5条件编译)

新建 `src/sar/output/ImageFormatter.*`。入参为 `SarFocusedImage`(非设计初稿未实现的 `ImagingResult`)。

| 函数(目标) | 依赖 | 说明 |
|---|---|---|
| `WriteBinaryImage(image, meta, filepath)` | 零依赖 | magic `1QSAR\x01\x00` + header(元数据)+ float32 交错实虚;升级自 `examples/session_usage.cpp` 的临时格式 |
| `WriteGeoTiffSidecar(image, meta, base_filepath)` | 零依赖 | `base.raw` + `base.json` manifest(origin_lat/lon/pixel_spacing/投影说明);SAR 尚无地理编码,真 GeoTIFF 后置 |
| `WriteHdf5Image(image, meta, filepath)` | HighFive(条件编译) | `/image/real`、`/image/imag` dataset + attrs;`ONEQ_ENABLE_HDF5_OUTPUT` 默认 OFF |

**依赖决策**:项目当前无 HDF5/TIFF/GeoTIFF 依赖,受 C++11 + Eigen 3.3.9 + VS2015 vendor 约束。采用:`toBinary` 零依赖真做;`toHdf5` 用 HighFive 条件编译(默认 OFF,不污染 vendor 链);`toGeoTiff` 降级为 sidecar manifest(等 SAR 地理编码就绪后再做真 GeoTIFF)。

---

## 与现有 1Q 模块的复用关系

| SAR 模块需求 | 复用 1Q 现有模块 |
|---|---|
| 坐标变换、斜距计算 | `sar::geometry::Distance`(自建,common/geometry 仅 Az/El/姿态,无斜距) |
| 大气传播延迟修正 | `common/atmosphere/`(损耗可用;折射弯曲/群延迟未实现,SAR 暂不复用) |
| FFT/IFFT | `sar::signal::SarFft`(Eigen 后端 facade;common/numerics 的 `ZFFT1D` 为朴素 DFT,不适用) |
| 目标 RCS 计算 | `common/rcs/`(点目标/植被散射;SAR σ⁰ 表模型自建) |
| 数值限幅/常量 | `common/numerics/ClampUtils`、`Constants`(`kPi`/`kLightSpeed`) |
| 日志输出 | `common/logging/`(`spdlog`) |
| 时序管理 | `common/timing/` |
| 运行时基础设施 | `common/runtime/` |
| 验证辅助 | `common/validation/` |

**窗函数 / 数值积分 / sinc·Lanczos 插值**:common 完全没有,SAR 自建。

---

## 算法实现优先级与阶段

| 阶段 | 内容 | 状态 |
|---|---|---|
| **Phase 0** | 会话/配置/trace-replay 契约 + CMake 安装清单 | ✅ 完成(公共 API 冻结) |
| **Phase 1** | LFM + 匹配滤波 + 距离压缩 + 点目标回波 + RDA + 脉冲缓冲区 | ✅ 完成(最小可审批闭环达成) |
| **补齐** | 窗函数 / 斜距 / 多普勒 / 杂波 / 天线 / 输出格式 | ✅ 已实现(批次1-6, 2026-06-22) |
| **Phase 2** | 相位重参考独立自由函数 + GBP 扩展 + 质量指标闭环 | ✅ 完成(相位重参考抽模块,质量指标米制/对比度/public summary/replay/trace 闭环) |
| **Phase 3** | BP 增强 + 运动补偿二阶 + 杂波建模深化 | 🟡 部分(BP 合并形式,运动补偿仅一阶) |
| **Phase 4** | 聚束/扫描 + 多视 + L2/L3 联动 + 真 GeoTIFF | 未启动 |
| **Phase 5** | OpenMP/GPU 加速 + 实时处理 + 联合仿真 | 未启动 |

**已冻结(【未进行设计需求,不再扩展】)**:自适应选择(Auto/ImagingSelector)、CSA、ω-K(Omega-K)、辐射定标。相关代码完整保留但排除构建,详见孤儿文件章节。

---

## 验证策略

### 单元测试(`tests/unit/sar_*.cpp`,GLOB_RECURSE 自动收集)

- 全部用 `TEST`(无 `TEST_F`),工厂函数构造场景,`EXPECT_NEAR` 按场景缩放容差,确定性随机用固定整数 seed(2026)。
- 覆盖:LFM 波形(采样/调频斜率/瞬时频率单调)、匹配滤波(`h[n]=s*[N-1-n]`)、脉冲压缩(3dB/20dB 主瓣 + PSLR/ISLR)、斜距(解析 vs 采样)、多普勒参数、脉冲环形缓冲区(覆盖/连续 pulse_id/跨步长/分数累积)、相位重参考(跨算法对齐)。
- 补齐测试(随批次):`sar_window_function_test`、`sar_geometry_model_test`、`sar_echo_clutter_test`、`sar_antenna_pattern_test`、`sar_image_output_test`。
- 新增库源文件须显式加入 `src/sar/SarSources.cmake` 的 `SAR_ENGINE_SOURCES`(engine 层)或 `SAR_CORE_SOURCES`(session 层);`src/sar/CMakeLists.txt` 仅 `include()` 该 manifest,不再持有源列表;测试源无需改 CMake。

### 集成测试

- Phase 1 点目标成像(`sar_rda_test` / `sar_session_pipeline_test`):RDA 距离/方位聚焦、PSLR/ISLR、图像熵、峰值位置误差。
- 算法对比(`sar_gbp_test`):RDA vs GBP/BP,经 `CompareImagesWithGlobalPhaseReference` 对齐。
- 轨迹 L1/L2/L3 对比(`sar_echo_geometry_buffer_test` / `sar_gbp_test`)。
- external raw IQ 端到端(`sar_session_pipeline_test`)。

### 性能基准(`tests/performance/sar_fft_performance_test.cpp`)

- 1024×1024 为 Phase 1 冻结上限;FFT / RDA / 点目标管线 / public Session / 一阶运动补偿 / sinc RCMC 均 `EXPECT_LT(elapsed, limit)`。
- GBP/BP 冻结在 128 门。
- 缺口:峰值内存断言、环形缓冲区吞吐量基准。

### Replay / Trace 决策

- Phase 1 trace/replay 仅记录 public 摘要,**不序列化 focused complex image 全矩阵**(已三重验证:schema 全标量、codec 只存 `SarOutputFrame` 摘要字段、trace JSON 只存摘要)。
- C++11 + Eigen 3.3.9 编译门(`tests/contract/check_sar_cxx11_compat.cmake`,9 个核心源 `-std=c++11 -Werror`);Windows/VS2015 非强制门。

---

## 孤儿文件状态

下列文件均有实质实现,但被 `src/sar/SarSources.cmake` 的 `SAR_ENGINE_SOURCES` / `SAR_CORE_SOURCES` 排除,且无任何已构建目标引用、无对应测试、公共入口 `sar.hpp` 不暴露。它们相互 include 形成内部闭环,对外完全孤立。**处置策略:不动代码,仅文档记录状态**(保留可恢复性,符合"不再扩展"而非"删除"的原始决策)。

| 文件 / 文件组 | 实现深度 | 备注 |
|---|---|---|
| `SarOmegaK*`(14 文件) | 生产级完整:Stolt 插值(二分+线性)、参考映射、相位补偿、网格缩减、方位逆变换、点目标八项容差验收 | 含 `SarOmegaKTruth*` 验收链:清单解析 + 自实现 SHA-256(160 行)+ 资格判定 + 编排 |
| `SarCsaGeometry` / `SarCsaIntermediateTruth` | 半成品:几何 + 中间态真值,主流程(scaling/RCMC)未实现 | — |
| `SarFocusingSelector` | 完整:L1/L2/L3 × 3 目的路由(刻意不含 CSA/OmegaK 选项) | 设计标注 Auto【不扩展】 |
| `SarRadiometricCalibration` | 完整:单/多目标定标 + RCS 反演 + 误差评估 + 执行流水线 | API 与设计初稿分叉;设计标注【不扩展】 |

**风险提示**:这些是"写完后被冻结的完整实现"。若误把它们加回 `src/sar/SarSources.cmake`,会突破 v2.1 的冻结决策。恢复任一项前须先重开设计审批。`tests/contract/check_sar_frozen_sources.cmake` 合同测试会拦截这类误注册。

---

## 参考文献

1. **I.G. Cumming, F.H. Wong**, *Digital Processing of Synthetic Aperture Radar Data: Algorithms and Implementation*, Artech House, 2005.
2. **C. Oliver, S. Quegan**, *Understanding Synthetic Aperture Radar Images*, SciTech Publishing, 1998.
3. **J.C. Curlander, R.N. McDonough**, *Synthetic Aperture Radar: Systems and Signal Processing*, Wiley, 1991.
4. **R. Bamler, P. Hartl**, "Synthetic Aperture Radar Interferometry", *Inverse Problems*, 1998.

---

*文档版本: v2.3*
*创建日期: 2026-06-04*
*更新日期: 2026-06-22*
*变更摘要: v2.3 同步阶段 4-5 拆分结果:源清单引用由 `CMakeLists.txt` 更正为 `SarSources.cmake`(含 `SAR_ENGINE_SOURCES`/`SAR_CORE_SOURCES` 双清单);session 目录树补全 `SarRawHistoryBuilder`/`SarRuntimeConfigValidation`/`SarImagingExecutor`/`SarFocusedImageAssembler`;`ExportFocusedImage` 归属由 `SarSession` 更正为 `SarImagingExecutor`;补充 `enable_range_compression`/`kRangeCompression` 语义说明(前置条件门,非独立输出);孤儿文件章节补充 `sar_frozen_sources` 合同护栏。v2.2 删除全部 C++ 代码段,改为 struct 字段表 + 自由函数清单的描述性形态;对齐实际架构(自由函数+POD、`sar::{signal,geometry,echo,imaging,output}` 子命名空间、四域配置取代 `SarSimulationConfig`);新增「实施状态总览」「架构约定」「孤儿文件状态」章节;记录 6 个待补模块(窗函数/斜距/多普勒/杂波/天线/输出)的实施计划与目标 API;补充相位重参考和成像质量评估的 public 标量摘要闭环、测试覆盖边界与剩余产品 QA 缺口;记录 `retain_focused_image` 的公共结果导出策略。v2.1 已将 CSA、Omega-K、自适应选择(Auto)与辐射定标标注为【未进行设计需求,不再扩展】。*
