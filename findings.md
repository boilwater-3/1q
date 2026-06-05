# 发现：SAR 模块工程化研究

## 当前探索来源

本轮研究基于：

- `sar_construction_scheme_complete.md`
- `SAR_MODULE_DESIGN.md`
- `SAR_PHASE1_ENGINEERING_CONTRACT.md`
- 现有 1Q 模块结构：`airborne_radar`、`electronic_surveillance_radar`、`electro_optical_sensor`
- 现有共享工具：`src/common/numerics`、`src/common/geometry`、`src/common/rcs`、`src/common/runtime`

## 核心结论

### Phase 1 范围

合理范围：

- 公共 API 和会话门面。
- LFM。
- 匹配滤波。
- 距离向脉冲压缩。
- 点目标原始回波。
- L1 匀速直线条带模式。
- RDA。
- 脉冲环形缓冲区。
- 基础质量指标和确定性测试。

不合理进入 Phase 1 的范围：

- Auto algorithm selector。
- GBP/BP/CSA/Omega-K。
- L2/L3 轨迹成像。
- motion compensation。
- autofocus。
- radiometric calibration。
- clutter modeling。
- HDF5/GeoTIFF。
- GPU/CUDA。
- maneuver behavior。
- reconnaissance scheduling。
- multi-sensor fusion。

## 已验证的仓库事实

### 1Q 模块形态

现有 AR/ESR/EOS 模块均采用：

- public include 入口。
- config/session/environment/extension 分层。
- `Session::Step()` 和 `StepWithResult()`。
- PIMPL。
- `ONEQ_API`。
- trace/replay 和 flatbuffer codec。
- CMake 中 engine/core object source 分组。

SAR 不能只交付算法类，必须补齐同等公共契约。

### 数值工具

`src/common/numerics/SpectralNumerics.cpp` 中 `ZFFT1D` 是 O(n^2) 朴素 DFT，注释明确当前默认场景小，n > 512 应替换为真正 FFT 或 Eigen FFT。

结论：

- 可用于小型单元测试。
- 不可用于 SAR Phase 1 大尺寸 RDA 性能声明。
- FFT 后端是阶段 2 前置门。

### FFT 依赖现状与初步后端验证

已核查：

- `conanfile.py` 当前依赖 Eigen、Boost、nanoflann、FlatBuffers、Zlib；非 Windows 额外依赖 spdlog/fmt/JSBSim。
- `cmake/ProjectDependencies.cmake` 没有 FFTW、KissFFT、PocketFFT、MKL、cuFFT 等 FFT 后端接入。
- Eigen 已存在，`unsupported/Eigen/FFT` 已在 macOS AppleClang + Conan Eigen 3.4.0 下通过编译和 SAR FFT facade 单元测试。

结论：

- 当前平台可先使用 Eigen unsupported FFT 推进 Phase 1 数值链路。
- C++11 + Conan Eigen 3.3.9 已通过全部 SAR engine 源文件编译门；按用户决策，Windows/VS2015 不作为 Phase 1 强制审批门。
- 若 Eigen FFT 在 VS2015 组合下不可接受，再引入 vendored PocketFFT/KissFFT。
- 1024x1024 级别 RDA 性能验收仍需单独性能基准。

研究记录：

- `docs/sar_fft_backend_research.md`

### 几何工具

`src/common/geometry/GeometryTransform.h` 支持姿态旋转、方位/俯仰和扫描几何，但不是完整 SAR 本地场景/斜距/多普勒模型。

结论：

- Phase 1 应先用本地 Cartesian 场景框架，避免 ECEF/geodetic 噪声进入算法验证。
- 后续可接入坐标转换。

### RCS 工具

`src/common/rcs/RcsPhysics.h` 是若干散射近似入口，不是 SAR 绝对辐射定标链。

结论：

- Phase 1 原始回波只做相对幅度 `sqrt(rcs)/R^2`。
- radiometric calibration 必须后置并单独成契约。

### 运行时工具

`src/common/runtime/RuntimeCycleExecutor.h` 提供轻量 cycle stamp/state 工具。

结论：

- 可复用为 SAR session 骨架的一部分。
- 不能替代 SAR 具体输入校验、诊断、trace/replay。

## 算法细节发现

### 匹配滤波风险

早期方案存在构造阶段共轭、压缩阶段再共轭的歧义。当前合同规定：

```text
h[n] = conj(s[N_s - 1 - n])
H[k] = conj(S[k])
Y[k] = FFT(x)[k] * FFT(h)[k]
```

必须选择一种主表示，禁止重复共轭。

### 距离门和双程传播

SAR 回波必须使用双程传播：

```text
tau = 2R/c
delta_r = c/(2f_s)
phase = exp(-j*4*pi*R/lambda)
```

旧式 `d_sig = T*c` 或单程时延口径会导致距离轴错误。

### RDA 合理边界

Phase 1 RDA 只在以下条件下合理：

- stripmap。
- broadside 或 near-broadside。
- L1 uniform straight line。
- constant PRF。
- point targets。
- no squint correction。
- no motion compensation。
- no autofocus。

RCMC 第一版允许 linear interpolation；sinc interpolation 后置。

### 信号链路实现口径

已实现：

- `src/sar/signal/SarWaveform.h`
- `src/sar/signal/SarWaveform.cpp`

当前口径：

- LFM 使用 `T = BT / B`、`K = B / T`、`N_s = ceil(T * f_s)`。
- 波形采样使用 `s[n] = exp(j*2*pi*(f0*t + 0.5*K*t^2))`。
- 匹配滤波只做一次时域反转共轭：`h[n] = conj(s[N_s - 1 - n])`。
- 距离压缩使用 FFT 线性卷积，补零长度取不小于 `N_x + N_h - 1` 的 2 次幂。
- range-aligned 输出从 `matched_filter.size() - 1` 开始裁剪回输入长度。
- 距离 bin 间隔使用 `c/(2*f_s)`。
- 质量指标当前覆盖 3dB 宽度、20dB 宽度、PSLR、ISLR。

已验证：

- `SarSignalChainTest.LfmWaveformUsesContractedDurationSlopeAndSamples`
- `SarSignalChainTest.LfmInstantaneousFrequencyIsMonotonic`
- `SarSignalChainTest.MatchedFilterIsTimeReversedConjugateOnce`
- `SarSignalChainTest.LinearConvolutionMatchesDirectReference`
- `SarSignalChainTest.RangeCompressionPlacesPeakAtDelayedPulse`
- `SarSignalChainTest.PulseQualityMetricsCapturePeakWidthAndSidelobes`

未完成：

- 点目标原始回波与多目标分离。
- 与 Session 输出帧的真实数据接线。

### 几何、原始回波与缓冲区口径

已实现：

- `src/sar/geometry/SarGeometry.h`
- `src/sar/geometry/SarGeometry.cpp`
- `src/sar/echo/SarEcho.h`
- `src/sar/echo/SarEcho.cpp`
- `src/sar/runtime/PulseRingBuffer.h`
- `src/sar/runtime/PulseRingBuffer.cpp`

当前口径：

- 本地 Cartesian 坐标固定为 `x=azimuth, y=ground range, z=altitude`。
- L1 轨迹为匀速直线、常 PRF、`pulse_id` 单调递增。
- 点目标回波使用 `tau = 2R/c`、`n_delay = round(tau*f_s)`、`phase = exp(-j*4*pi*R/lambda)`、`amplitude = sqrt(rcs)/R^2`。
- near-edge waveform 写入不足时返回 clipping diagnostics。
- `PulseRingBuffer` 拒绝重复和乱序 pulse_id，range/latest 读取均要求连续，overflow sticky 一旦置位不清除。
- fractional PRF 采用 carry 保留小数脉冲，避免平均 PRF 丢失。

已验证：

- `SarGeometryTest.StraightStripmapTrackUsesMonotonicPulseIdsAndConstantPrf`
- `SarGeometryTest.FractionalPrfCarryPreservesAveragePulseRate`
- `SarEchoTest.SinglePointTargetDelayPhaseAndAmplitudeAreCorrect`
- `SarEchoTest.ThreeSeparatedTargetsProduceSeparatedBins`
- `SarEchoTest.NearEdgeTargetReportsClippingDiagnostics`
- `PulseRingBufferTest.RejectsDuplicateAndRequiresContiguousRangeReads`
- `PulseRingBufferTest.LatestReadsRequireContinuityAndOverflowStickyPersists`
- `PulseRingBufferTest.RejectsOutOfOrderPulseIds`

未完成：

- pulse history 二维矩阵生成。
- RDA 所需 azimuth/range metadata 汇总。
- 与 `SarSession` 的真实执行链路接线。

### RDA 当前实现口径

已实现：

- `src/sar/imaging/SarRda.h`
- `src/sar/imaging/SarRda.cpp`

当前口径：

- 输入为 raw pulse history 行主序矩阵，rows=azimuth pulses，cols=range samples。
- 逐 pulse 执行阶段 3 的距离压缩，并生成 range-aligned history。
- 方位向处理执行 azimuth FFT、linear RCMC、azimuth matched filter、azimuth IFFT。
- RCMC 使用 `delta_r_cm = lambda^2*R_0*f_a^2/(8*v^2)` 与 `delta_n_cm = delta_r_cm/delta_r`。
- azimuth matched filter 使用 `H_az(f_a) = exp(j*pi*f_a^2/K_a)`，其中 `K_a = 2*v^2/(lambda*R_0)`。
- focused image 从 azimuth IFFT 后结果提取。
- diagnostics 记录 reference range、doppler rate、range bin spacing、out-of-bounds samples、RCMC 插值类型和各阶段执行标记。
- focused-image entropy 使用归一化像素功率 Shannon entropy：`H = -sum(p_i * ln(p_i))`，单位 nats。

已验证：

- `SarRdaTest.SingleCenterPointFocusesNearExpectedPixel`
- `SarRdaTest.ThreeSeparatedTargetsRemainResolvableWithExpectedPeakOrder`
- `SarRdaTest.DiagnosticsRecordRdaStagesAndReferenceParameters`
- `SarRdaTest.ImageEntropyUsesNormalizedPowerDistribution`
- `SarRdaTest.RejectsInvalidInputs`

限制：

- 当前验收限定 L1 broadside 点目标场景。
- 尚未覆盖 squint、motion compensation、autofocus、clutter、radiometric calibration。
- 尚未建立 1024x1024 性能基准。

### Session 管线当前实现口径

已实现：

- `src/sar/session/SarSession.cpp`
- `src/sar/session/SarTraceSession.cpp`
- `src/sar/session/SarReplaySession.cpp`
- `src/sar/session/SarReplayFlatbufferCodec.h`
- `src/sar/session/SarReplayFlatbufferCodec.cpp`
- `schemas/replay/sar_replay.fbs`
- `schemas/replay/sar_session_replay.fbs`
- `tests/unit/sar_session_pipeline_test.cpp`
- `tests/unit/sar_replay_codec_roundtrip_test.cpp`
- `tests/unit/sar_replay_session_test.cpp`

当前口径：

- `SarSession::StepWithResult()` 已不再返回纯骨架占位结果；当前平台路径会生成 LFM、点目标 raw echo history、匹配滤波器，并在策略启用时执行 RDA。
- public `SarCycleInput` 的平台 LLA 与点目标 LLA 使用 Phase 1 flat-earth 近似转换到本地 Cartesian 坐标。
- L1 轨迹使用输入平台位置为中心的匀速直线 stripmap 轨迹，方位向样本数来自 `SarMissionConfig::azimuth_pulse_count`。
- RDA 输出当前只以 `SarOutputFrame` 的阶段标记、样本数、中心斜距和 diagnostics 摘要形式公开；内部 focused complex image 未暴露到 public API。
- 当前平台 Session RDA 运行时尺寸门已提升到 `range_sample_count <= 1024` 且 `azimuth_pulse_count <= 1024`；超过该尺寸必须单独审批。
- 当前 Phase 1 RDA 依赖 raw echo generation；若 `enable_l1_rda_imaging=true` 且 `enable_raw_echo_generation=false`，Session 返回 `rda_requires_raw_echo` 结构化错误。
- 无效 `dt_sec`、非法运行时配置、raw echo/RDA 失败均通过 `SarCycleResult` 返回结构化错误；已有上一帧时复用上一帧输出。
- replay schema 分为 `sar_replay.fbs` 和 `sar_session_replay.fbs`，分别覆盖 cycle input/output/result 与 session config/runtime patch。
- `SarTraceSession` 使用 `ReplayTraceWriter` 写入 FlatBuffers payload，事件类型覆盖 `session_config`、`cycle_input`、`runtime_config_patch`、`cycle_output`。
- `ReplaySarTrace()` 通过 trace 重建 SAR Session，应用 runtime patch，重新执行 cycle input，并比对 `SarCycleResult` public 摘要。
- Phase 1 replay 策略固定为摘要级：记录 public `SarCycleResult`，不记录 focused complex image 全矩阵。

已验证：

- `SarSessionPipelineTest.StepWithResultRunsRawRangeAndRdaPipeline`
- `SarSessionPipelineTest.RuntimeSizeGateRejectsUnapprovedLargeRda`
- `SarSessionPipelineTest.RdaRequiresRawEchoGenerationInPhase1Pipeline`
- `SarSessionPipelineTest.InvalidCycleReusesPreviousOutput`
- `SarReplayCodecRoundtripTest.*`
- `SarReplaySessionTest.*`
- `PublicHeadersSmokeTest.SarPublicSurfaceSupportsMinimalUsage`

限制：

- Trace/replay 当前只比对 public 摘要；focused complex image 全矩阵、外部 artifact 引用和压缩图像输出后置。
- Session 已接入 `PulseRingBuffer` 作为跨周期慢时间累积机制；首帧补足 aperture，后续周期按 `dt * PRF + fractional carry` 追加新 pulse，并从 latest-N 连续 pulse history 执行 RDA。
- LLA 到 local Cartesian 仍为小场景近似，不作为大范围地理成像口径。

## 文档修改发现

已完成：

- `sar_construction_scheme_complete.md` 中算法编号统一为 `0=Auto,1=RDA,2=CSA,3=OmegaK,4=GBP,5=BP`。
- `SAR_MODULE_DESIGN.md` 中 Phase 0-5 重排。
- `SAR_PHASE1_ENGINEERING_CONTRACT.md` 新增并设为 Phase 1 优先控制文档。

## 决策记录

| 决策 | 结论 | 理由 |
|---|---|---|
| Phase 1 聚焦算法 | RDA only | 最小可审批闭环 |
| Auto selector | 暂缓 | 需要多个验证算法 |
| RCMC 插值 | linear first | 降低首版复杂度 |
| 场景坐标 | local Cartesian | 降低算法验证噪声 |
| 辐射定标 | 后置 | 现有 RCS helper 不够 |
| FFT 后端 | Phase 1 冻结 Conan Eigen FFT facade | Eigen 3.4.0/3.3.9 正确性通过；C++11 + Eigen 3.3.9 SAR engine 编译门通过 |
| 信号链路 | 当前平台完成内部 LFM/匹配滤波/距离压缩 | 单元测试已覆盖公式、峰值位置和基础质量指标 |
| 原始回波与缓冲区 | 当前平台完成 L1 本地点目标回波和 pulse ring buffer | 单元测试已覆盖双程延迟、裁剪诊断、连续性和 fractional PRF |
| RDA | 当前平台完成 L1 broadside 点目标 RDA 最小闭环 | 单元测试已覆盖单点、三点和 diagnostics |
| Session 管线 | 当前平台完成 `SarSession::StepWithResult()` 到 raw/range/RDA 的真实闭环，并接入跨周期 `PulseRingBuffer` | 1024x1024 public Session 点目标性能与峰值内存已测量，超过该尺寸仍受门禁 |
| Trace/Replay | 当前平台完成 SAR FlatBuffers 摘要级 replay 闭环 | replay fast 测试覆盖 codec round-trip、trace 回放、runtime patch 和尾随 input 拒绝 |
| CI/审批包 | 当前平台完成 SAR 专属 CTest 标签和 Phase 1 acceptance report | `sar_performance` 覆盖 FFT、内部 RDA、真实点目标内部管线和 public Session |

## 后续研究入口

优先级：

1. FFT 后端可行性。
2. 超过 1024x1024 的性能与峰值内存基准。
3. RDA reference scene 生成脚本。
4. SAR trace/replay 图像存储策略。
5. 非阻断 Windows/VS2015 补充验证。

## 外部内容安全

当前 findings 仅来自本地仓库文件和本轮推导；未使用网页或外部不可信内容。

## Phase 2 扩展方向决策

用户已批准：

- Phase 2 选择“参考级成像与算法对比闭环”。
- Auto algorithm selector 继续后置。

工程判断：

- 当前只有 RDA，提前启用 Auto 只能形成缺乏证据的选择器，不能形成可审批能力。
- Sinc RCMC 是风险最低的 RDA 精度增强，但必须通过相同场景的质量改善证据审批。
- 小场景 GBP 可作为独立高精度参考算法，为 RDA 正确性和后续 Auto 提供第二条证据链。
- 相位重参考应与跨算法比较一起实施，并严格限定为全局常数相位对齐。
- 当前完整聚焦复矩阵只存在于内部对象中，因此 Phase 2A 的质量比较应先保持内部实现；不应借此提前开放 public 全图输出或全图 replay。
- Phase 1 public Session `1024x1024` 上限继续冻结；GBP 需要更严格的独立小场景尺寸门。

推荐执行顺序：

1. 共用质量指标与确定性参考场景。
2. 显式 linear/sinc RCMC 与对照验收。
3. 小场景 GBP 与 RDA/GBP 跨算法比较。
4. 双算法分别审批后再讨论 Auto。

## Phase 2A 第一批实现发现

- 新增统一 `ImageQualityMetrics` 后，RDA 已直接使用该组件计算现有方位 3dB 宽度和图像熵，避免测试指标与生产聚焦指标分叉。
- 跨算法比较采用单个常数复旋转进行全局相位重参考；空间变化相位输入仍保留明显归一化 RMS 误差。
- RCMC 内部配置已从 bool 升级为显式 `none/linear/sinc`；public Session 仍明确指定 linear。
- 有限核 Sinc 使用 Lanczos 窗化核，当前批准核半宽范围为 2-16，默认实验半宽为 4。
- 已知带限复正弦的分数采样测试证明 Sinc 插值误差低于 linear。
- 1024x1024 独立 RCMC Debug 测量：linear 约 0.024230 s，Sinc 半宽 4 约 0.178153 s，约 7.4 倍代价。
- 当前证据不足以宣称 Sinc 在真实成像质量上默认优于 linear；该判断需要 GBP 独立参考真值，因此 public Session 默认切换继续门禁。
