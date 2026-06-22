# 发现：SAR 模块工程化研究

## 当前探索来源

本轮研究基于：

- `docs/sar/design/construction_scheme.md`
- `docs/sar/design/module_design.md`
- `docs/sar/design/phase1_engineering.md`
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

## Phase 2B GBP 与跨算法比较发现

- GBP 采用 local Cartesian 小场景网格，行对应方位 `x`，列对应地距 `y`，固定成像平面 `z`。
- GBP 与 RDA 共用 raw pulse history、匹配滤波和逐脉冲平台位置；内核先距离压缩，再按双程时延采样并补偿传播相位。
- `128x128` 是严格内部 GBP 上限，任一维 `129` 会明确拒绝；未增加 public Session 算法选择入口。
- 单点目标准确聚焦到预期像素，三点目标保持预期幅度顺序。
- RDA/GBP 相同窗口比较中：
  - 全局相位偏移约 `-0.047659 rad`。
  - 单位能量 NRMS 约 `0.042218`。
  - 相干相关系数约 `0.999109`。
- 原始仅相位对齐 NRMS 曾达到约 `7.956736`，根因是 GBP 与 RDA 的全局累积增益不同；修正为两图分别单位能量归一化后的形状 NRMS，且空间变化相位错误测试仍能检出残余误差。
- `128x128` GBP Debug 参考场景约 `0.149 s`，当前平台性能可接受，但该结果不批准扩大尺寸上限。

## Phase 2C 审批发现

- 使用 GBP 参考真值补充比较后，linear RCMC 的 NRMS 为 `0.042218`、相关系数为 `0.999109`；Sinc RCMC 的 NRMS 为 `0.042107`、相关系数为 `0.999114`。
- Sinc 在当前单点参考场景中有可测但极小的质量改善，结合约 7.4 倍独立 RCMC 性能代价，不支持成为 public Session 默认路径。
- Auto 仍缺少正式运行所需的算法覆盖、场景覆盖、选择阈值、降级规则和结构化诊断；且用户已明确批准继续后置。
- Phase 2C 正式结论：Auto 继续门禁，public Session 保持 RDA + linear；下一方向建议 L2 轨迹误差与一阶运动补偿，但需另行审批。

## L2 轨迹与一阶运动补偿契约发现

- 用户以“继续”批准进入 L2 方向。
- 逐脉冲直接对位置施加白噪声会产生不连续跳变，不适合作为工程可审批的轨迹误差模型。
- L2 冻结为固定种子零均值速度扰动，并通过逐脉冲积分生成连续位置轨迹；零扰动必须严格退化为 L1。
- 一阶运动补偿以实际/理想轨迹相对显式参考点的斜距差为输入，同时校正双程时延和载频传播相位。
- L2 public Session 配置、L3、姿态误差、IMU/GPS、自聚焦和 Auto 继续后置。

## L2 连续扰动轨迹实现发现

- L2 三轴速度扰动使用固定种子 `std::mt19937` 与高斯分布，位置由前一脉冲速度按 `1/PRF` 积分，轨迹连续且可复现。
- 零扰动递推积分与 L1 直接位置公式会产生约 `1e-14 m` 浮点差；为满足严格退化契约，全零扰动配置显式返回 L1 轨迹和零诊断。
- 轨迹诊断记录最大/RMS 位置误差和最大/RMS 速度误差，可作为后续运动补偿验收输入。

## 一阶运动补偿闭环发现

- 一阶补偿直接作用于 raw pulse history，使用实际/理想轨迹相对显式参考点的斜距差，同时执行双程包络平移和载频相位修正。
- 确定性 L2 单点场景中，最大参考斜距误差约 `1.300414 m`。
- 未补偿成像相对理想参考的 NRMS 为 `1.312794`、相关系数为 `0.138285`；补偿后 NRMS 降至 `0.249779`、相关系数升至 `0.968805`。
- `1024x1024` raw history 一阶补偿 Debug 核心处理约 `0.052741 s`。
- 当前内部能力已证明，但 public Session 尚无 L2 配置、实际/理想轨迹输入和 replay 契约，因此不得直接接入默认路径。

## L2 Public Session 接入契约发现

- Session 当前内部生成 L1 aperture，public 输入没有逐脉冲实际/理想轨迹，因此受控接入应继续由 Session 依据 mission 固定种子生成 L2。
- 为防止跨 aperture 运行期修改扰动统计特性，L2 标准差和 seed 只进入 session config replay，不进入 runtime patch。
- public Session 不开放 L2 未补偿 RDA：启用 L2 policy 时必须强制执行一阶补偿。
- 一阶补偿参考点冻结为 local `(0, nominal_slant_range_m, 0)`，避免依赖目标列表顺序。

## L2 公开配置与 replay 发现

- L2 public 配置只进入初始化 session config，不进入 runtime patch，避免 aperture 中途改变随机轨迹统计。
- FlatBuffers mission/policy 字段追加在表尾部，旧 payload 解码时新字段自然采用 `0/false`，保持默认 L1 行为。
- `enable_l2_motion_compensation` 默认 `false`，public smoke test 和既有 replay 回放保持不变。

## L2 Public Session 执行闭环发现

- Session 必须让 raw pulse history、理想轨迹和实际轨迹使用同一 latest-N aperture；只保存当前周期新轨迹会导致跨周期补偿错位。
- 跨周期 L2 轨迹通过上一实际脉冲位置与速度外推下一批首脉冲位置误差，保持批次边界位置连续。
- 实际轨迹用于 raw echo，理想/实际轨迹共同用于 RDA 前一阶补偿；public L2 未补偿 RDA 继续禁止。
- 全零扰动路径严格退化为 L1 输出摘要，并输出零位置/斜距误差诊断。
- L2 非零扰动摘要级 replay、第二周期增量脉冲和双轨迹 aperture 对齐均通过。
- 阶段 18 完成后仍没有证据支持开放多参考点、L3、二阶补偿、自聚焦或 Auto。

## 阶段 19 L3 方向决策发现

- 建设方案将外部轨迹文件/真实 IMU-GPS 数据定义为 L4，明确超出当前范围；因此不能将外部逐脉冲轨迹 public 输入作为当前首选。
- 多参考点/空间变化补偿需要明确参考面、目标分布和独立成像真值；当前单参考点 L2 证据不足以审批该扩展。
- L3 航路点轨迹是建设方案已定义的下一保真度等级，也是后续非直线轨迹成像与高阶补偿的必要前置。
- L3 表格要求配合时变 PRF，而设计草案仅提供固定 `prf_hz`；首批应使用显式脉冲时刻作为几何层输入，避免提前发明未审批的 PRF 调度策略。
- 折线转角包含瞬时速度方向变化，只能作为确定性几何与算法压力基准，不能宣称物理平台动力学保真。

## L3 航路点轨迹几何发现

- 显式脉冲时刻比在几何层内嵌时变 PRF 调度策略更可审批：它保留真实采样时间，又不提前绑定调度算法。
- 航路点转角时刻采用“进入新航段”的速度语义；位置准确命中转角，速度从该时刻起切换为下一航段速度。
- 两点直线航路点配合固定 `1/PRF` 脉冲时刻可与 L1 位置、速度、时间和 `pulse_id` 逐点一致。
- 当前 L3 仅证明几何与采样时间正确；尚未证明 RDA 能处理非直线轨迹，也不支持批准二阶补偿。

## L3 Raw Echo 与成像退化基线发现

- 在已审批 `9x9` 跨算法参考场景中，L1-RDA 相对 L1-GBP 的 NRMS 为 `0.042218`、相关系数为 `0.999109`。
- 同场景只将轨迹改为孔径末端横向偏移 `3 m` 的折线 L3 后，L1-RDA 相对 L3-GBP 的 NRMS 恶化到 `0.501878`、相关系数降至 `0.874059`。
- L3-GBP 使用实际逐脉冲位置后仍将单点目标聚焦到预期邻域，说明 L3 raw echo 与轨迹数据本身一致。
- 尝试扩大到 `33x9` 场景时，L1-RDA/GBP 基线本身已出现大差异，无法隔离 L3 影响；该场景未用于验收，也不能通过放宽阈值处理。
- 下一步应先量化现有单参考点一阶补偿对 L3 的效果与空间变化残余，再判断 BP、多参考点或二阶补偿的必要性。

## L3 一阶补偿适用性审计发现

- 在当前 `9x9`、孔径末端横向偏移 `3 m` 的 L3 场景中，一阶补偿相对 L3-GBP 将 NRMS 从 `0.501878` 降至 `0.185881`，相关系数从 `0.874059` 提升至 `0.982724`。
- 对偏离中心参考距离单元的目标，单参考点补偿后仍存在约 `0.000180 m` 最大残余斜距误差，证明补偿具有空间变化误差。
- 当前残余小于 `0.001 m`，且参考点成像达到 NRMS `< 0.25`、相关系数 `> 0.97`；没有证据支持立即增加二阶补偿、BP 或多参考点复杂度。
- 当前审批只适用于固定 PRF 小场景；必须建立转弯幅度和目标偏移矩阵后才能冻结适用范围。

## L3 一阶补偿适用边界矩阵发现

- 一阶补偿后相对 L3-GBP，孔径末端横向偏移 `0/1/3/6 m` 当前质量门通过；`12 m` 时 NRMS `0.386100`、相关系数 `0.925463`，明确失效。
- 补偿质量随横向偏移不严格单调，不能用单一转弯幅度阈值替代成像质量指标。
- 在 `12 m` 转弯下，目标距离单元从补偿参考点 `20` 偏移到 `18/16/12` 时，最大残余斜距误差为 `0.000594/0.001558/0.007119 m`，持续增大。
- 现有 GBP 已按真实逐脉冲位置执行精确后向投影，可作为 L3 BP 真值；设计中 BP 与 GBP 的数学结果相同，差异主要是遍历顺序。
- 后续 BP 应通过共享后向投影核心实现脉冲优先累加，不能复制一套独立距离压缩、插值和相位逻辑。

## L3 BP 工程契约发现

- GBP 与 BP 的数学输入输出、距离压缩、插值和传播相位完全相同；两者独立实现会制造高风险重复。
- BP 唯一需要新增的行为是将遍历顺序从“像素优先”改为“脉冲优先”，每个像素内部仍按脉冲顺序累加。
- 保持每个像素的累加顺序后，GBP/BP 相同输入应达到复图逐样本一致，而不是仅质量指标接近。
- 当前 BP 必须继续受 `128x128` 内部尺寸门约束，不得借 L3 失效区提前开放 public 或 Auto。

## L3 BP 内部闭环发现

- 提取共享后向投影核心后，GBP 像素优先与 BP 脉冲优先在 L1/L3 相同输入下复图逐样本一致。
- `12 m` L3 一阶补偿失效区中，BP 相对 L3-GBP 的 NRMS 为 `0.0`、相关系数为 `1.0`，明确优于补偿后 RDA。
- `128x128` Debug 下 GBP 约 `0.183516 s`、BP 约 `0.177757 s`，遍历顺序变化未引入不可接受性能回退。
- BP 内部能力已具备，但 public Session 尚无 waypoint、时间基准、显式算法 policy 或 BP replay 契约，因此不得直接接入。

## L3 BP Public Session 接入契约发现

- 现有 public SAR 输入与目标均使用 LLA；public waypoint 应保持 LLA，并在 Session 内相对 scene center 转换为 local Cartesian，不能泄漏内部坐标类型。
- L3 waypoint 时间采用相对 Session 起点秒数；固定 PRF 脉冲时刻由连续 `pulse_id / PRF` 生成，禁止超出 waypoint 覆盖范围外推。
- L3 BP 必须显式启用，并与 L1-RDA、L2 运动补偿互斥；当前不允许根据轨迹自动选择算法。
- public L3 BP 必须使用独立 `128x128` 尺寸门，不能沿用 RDA `1024x1024` 上限。
- waypoint、BP policy 和 L3/BP 输出摘要必须进入 replay，但不得进入 runtime patch，也不开放全图复矩阵。

## L3 BP 公开配置与 replay 发现

- public `SarWaypointConfig` 使用相对 Session 起点时间和 LLA 字段，保持与现有 SAR 外部输入一致。
- `enable_l3_bp_imaging` 默认 `false`；`l3_waypoints` 默认空列表，旧 payload 解码保持 L3/BP 关闭。
- cycle output replay 已保真 `kL3BpImage` 与 `has_l3_bp_image`，但 focused complex image 仍未开放。
- waypoint、L3 BP policy 和输出摘要 codec round-trip、public smoke、默认 Session replay 均通过。

## L3 BP Session 执行闭环发现

- Session 可按连续 `pulse_id / PRF` 将 public LLA waypoint 转为固定 PRF L3 脉冲轨迹，并使用实际轨迹生成 raw echo 与执行 BP。
- latest-N raw history 与实际轨迹跨周期保持同一 aperture；第二周期仅追加两脉冲，时间从 `0.45 s` 连续到 `0.50 s`。
- L3 BP 与 L1-RDA/L2 互斥，缺失 raw/range compression、无效 waypoint 结构、覆盖不足和超过 `128x128` 均有独立结构化拒绝。
- `sar.l3_trajectory`、`sar.bp_peak`、`sar.bp_traversal` 与 L3 输出摘要可完成两周期无 divergence replay。
- public L3 BP 已具备受控 Session 闭环，但当前证据仍不支持 Auto、runtime patch、全图复矩阵 replay、时变 PRF 或扩大 BP 尺寸门。
- 全仓无目标构建仍被非 SAR 的 JSBSim `FGFDMExec.h` 缺失阻断；SAR 独立构建与审批门不受影响。

## Phase 2 完成度审计发现

- 当前固定 PRF、小场景、点目标范围内，RDA/GBP/BP、L2 一阶补偿、L3 适用边界和 public L3 BP 已形成可重复闭环。
- 当前跨算法质量证据仍主要来自中心单点与同距离向多目标；缺少方位偏置、二维多目标、边界目标和硬件参数扫描。
- RDA/GBP/BP 分别具备回归与性能证据仍不足以批准 Auto；代表性矩阵、选择阈值、资源预算、降级规则和结构化选择诊断均未冻结。
- 下一方向选择参考场景矩阵扩展，优先补齐已批准主线的代表性证据，不提前增加 public API 或复杂算法。
- 首批矩阵冻结为 M1-M7，覆盖 L1/L2/L3、中心/偏置/二维多目标以及一阶补偿通过区和失效区。

## 首批参考场景矩阵发现

- L1 中心、距离偏置、方位偏置和二维多目标相对 GBP 的 NRMS 为 `0.042218/0.046455/0.075643/0.080137`，相关系数均高于 `0.996`。
- M1-M4 中 BP 与 GBP 复图逐样本一致；二维目标位置和 RCS 幅度顺序保持正确。
- L2 二维目标固定 seed 扰动经一阶补偿后，NRMS 从 `1.329265` 降至 `0.239074`，相关系数从 `0.116527` 升至 `0.971422`；结果接近当前门，不支持扩大适用声明。
- L3 `3 m` 二维目标补偿后 NRMS `0.228381`、相关系数 `0.973921`，保持当前通过区。
- L3 `12 m` 二维目标补偿后 NRMS `0.514096`、相关系数 `0.867853`，强化明确失效结论。
- M1-M7 补齐了二维目标代表性，但仍只覆盖固定硬件参数、固定 PRF 和中心附近小窗口；下一缺口是边界与参数适用性矩阵。

## 参考矩阵后续方向发现

- M1-M7 已证明中心附近二维目标布局不破坏当前 L1/L2/L3 结论，但没有量化 raw echo 裁剪、图像主瓣截断或单参数变化下的适用范围。
- 下一方向选择图像边界与采样/硬件参数适用性矩阵，比立即进入时变 PRF、高阶补偿或 Auto 更直接补齐当前参考证据缺口。
- 边界场景必须区分 `interior_pass`、`boundary_degraded`、`echo_clipped` 和 `invalid`，禁止把裁剪或退化场景通过阈值放宽归类为通过。
- 参数矩阵首批逐项扫描采样率、载频、PRF 和平台速度；每次只改变一个参数，失败结果用于冻结适用范围。

## 边界与参数适用性矩阵发现

- 图像网格边缘目标即使 RDA/GBP NRMS 与相关性仍通过，也必须因峰值/主瓣位于边缘归类为 `boundary_degraded`。
- 上侧 raw 边界场景出现 9 个脉冲、72 个样本裁剪，但 RDA/GBP 仍有 NRMS `0.014853`、相关系数 `0.999890`；跨算法一致不能替代输入完整性诊断。
- `80/100/120 MHz` 采样率与 `0.8/1.0/1.2 GHz` 载频首批档位均通过当前门，但不能据此外推完整支持范围。
- PRF `10 Hz`、速度 `2 m/s` 与 PRF `20 Hz`、速度 `4 m/s` 均对应 `v/PRF=0.2 m/pulse`，并得到相同 NRMS `0.177589`、相关系数 `0.984231`。
- `v/PRF=0.05/0.1 m/pulse` 当前门通过；证据说明退化与方位采样间距耦合，不能冻结为孤立 PRF 或速度阈值。
- 下一步必须审计方位采样充分性的物理/算法关系，再决定结构化诊断或算法修复。

## RDA 方位采样充分性审计发现

- 方位间距从 `0.05` 扫描到 `0.2 m/pulse` 时，RDA/GBP NRMS 单调从 `0.010554` 增至 `0.177589`；当前门在 `0.175 m/pulse` 开始失败。
- 失败点几何 Doppler Nyquist 裕量仍为 `18.35x/14.05x`，最大相邻传播相位步进仅 `0.149837/0.195693 rad`，远未达到混叠条件。
- 粗间距场景 none/linear/sinc RCMC 的 NRMS 为 `0.177713/0.177589/0.177455`，排除 RCMC 插值为主要原因。
- 每脉冲二阶方位相位曲率 `4*pi*(v/PRF)^2/(lambda*R_ref)` 可统一解释间距、载频和斜距变化。
- `80 MHz` 远斜距与 `0.8 GHz` 低载频等曲率，NRMS 为 `0.033714/0.033870`；`120 MHz` 近斜距与 `1.2 GHz` 高载频等曲率，NRMS 为 `0.050886/0.050602`。
- 下一方向选择增加解释性 RDA diagnostics；当前矩阵仍不足以批准警告阈值、结构化拒绝、算法修复或 Auto。

## RDA 方位相位曲率诊断实现发现

- `RdaDiagnostics` 已记录方位采样间距、每脉冲二阶相位曲率、最大几何 Doppler 和 Nyquist 裕量。
- 参数矩阵已改为直接核对生产 diagnostics；粗间距退化场景仍显示 Nyquist 裕量大于 `1`。
- `v/PRF` 等间距参数对产生相同采样间距和相位曲率；等曲率载频/参考斜距参数对由生产 diagnostics 得到相同曲率。
- 单脉冲输入不能通过现有 FFT 成像链；RDA 成像入口已明确拒绝单脉冲 aperture，单脉冲无穷裕量由独立内部诊断计算验证。
- Session `sar.rda_peak` 已记录四项指标，现有 replay 严格 diagnostics 比较无 divergence。
- 当前诊断仍只解释 broadside L1 RDA；下一步必须扩展孔径与目标方位布局矩阵，才能评估是否具备警告阈值证据。

## RDA 诊断后续决策发现

- 相同每脉冲相位曲率 `0.013982 rad` 下，中心目标 NRMS 随 aperture 从 `5` 脉冲的 `0.012145` 增至 `33` 脉冲的 `0.676300`，因此不能单独形成警告阈值。
- 候选孔径二次相位跨度 `curvature*((N-1)/2)^2` 可归并中心目标等效组合；三组等跨度 NRMS 差值分别为 `0.005636/0.017843/0.027830`。
- 相同采样诊断与 aperture 下，方位偏置目标的 NRMS 均高于中心目标，目标布局仍是独立质量影响量。
- 阶段 43 不批准质量警告、结构化拒绝、算法修复或 Auto；下一步只冻结孔径二次相位跨度解释性诊断契约。

## RDA 孔径二次相位跨度诊断契约发现

- 孔径二次相位跨度定义为 `azimuth_phase_curvature_rad_per_pulse2*((N-1)/2)^2`。
- 单脉冲跨度定义为 `0`，与独立采样诊断可验证但 RDA 成像入口拒绝单脉冲的边界一致。
- 指标进入内部 `RdaDiagnostics` 和 Session `sar.rda_peak`，不修改 replay schema。
- 目标方位偏置影响不由该指标覆盖；当前仍禁止警告阈值、结构化拒绝和 Auto。

## RDA 孔径二次相位跨度诊断实现发现

- `RdaDiagnostics` 已记录 `azimuth_quadratic_phase_span_rad`，由每脉冲曲率与实际 aperture 脉冲数计算。
- 单脉冲跨度为 `0`；Session `sar.rda_peak` 与 replay 已保真该字段。
- 阶段 43 三组等跨度组合由生产 diagnostics 得到相同跨度，中心目标 NRMS 差值保持在 `0.03` 内。
- 指标仍不能解释同配置下目标方位偏置产生的额外误差；下一步进入目标方位偏置几何决策门。

## RDA 目标方位偏置误差决策发现

- 等物理孔径、等目标偏置组合具有完全相同的目标偏置非线性相位残差，但不同采样密度的 NRMS 仍不同。
- 候选非线性相位残差最大仅 `0.004442 rad`，无法单独解释最高超过 `0.5` 的偏置额外 NRMS。
- `-0.4/-0.2/0/0.2/0.4 m` 偏置 NRMS 为 `0.260633/0.190284/0.159746/0.190284/0.260633`，证明影响对方向对称并随偏置幅值增加。
- 目标偏置反映 aperture 支撑相对目标的失衡，但质量同时依赖孔径二次相位跨度和采样密度，当前不能冻结单变量阈值。
- `RdaConfig` 不包含目标布局，多目标场景也不存在唯一偏置，因此不批准新增输入级全局 diagnostics、质量警告、结构化拒绝或 Auto。

## Phase 2 参考级成像闭环综合再审批发现

- 当前固定 PRF、点目标、显式算法路径已覆盖 L1 RDA、L2 一阶补偿、L3 BP、内部 GBP/BP 参考和摘要级 replay。
- RDA 与 BP 的适用域和尺寸门不同，GBP 仍是内部参考路径，现有 diagnostics 也不能形成通用质量阈值，因此 Auto 继续不批准。
- 当前跨算法证据仍以无噪声确定性点目标为主，无法证明质量指标和算法比较在不同 SNR 下的稳定性。
- 下一方向选择测试侧固定 seed 复高斯噪声与 SNR 鲁棒性矩阵；分布式杂波、辐射定标、时变 PRF 和真实动力学轨迹继续后置。

## 确定性噪声与 SNR 鲁棒性矩阵契约发现

- 测试噪声必须注入 raw pulse history，并由 RDA、GBP 与 BP 共同消费同一 noisy 输入。
- 为避免标准库实现相关序列，固定 seed 噪声使用自定义伪随机生成器与明确的复高斯变换。
- 候选噪声按总能量精确缩放到 requested SNR，使 realized SNR 可重复并可严格验证。
- 首批矩阵覆盖 M1 中心单点、M4 二维多目标和无噪声/30/20/10/0 dB；只评估趋势，不冻结通用阈值。
- 生产噪声模型、public 配置、replay、Auto、杂波和辐射定标继续后置。

## 确定性噪声与 SNR 鲁棒性矩阵实现发现

- 固定 seed noisy raw history 逐样本一致，不同 seed 不同，精确能量缩放使 realized SNR 与 requested SNR 一致。
- RDA、GBP 与 BP 在每个场景共同消费同一 noisy raw history；BP 与 GBP 在 noisy 输入下继续逐样本一致。
- seed 17 下 M1 的 RDA/GBP NRMS 随 `30/20/10/0 dB` 为 `0.042180/0.044864/0.070107/0.187519`；M4 为 `0.079274/0.079116/0.092969/0.201058`。
- seed 29 保持相同总体趋势；`0 dB` 明显越过当前无噪声参考门，但不能据此定义带噪场景算法失效。
- RDA clean/noisy 完整输出与 GBP clean/noisy 小窗口的绝对数值不可直接横向比较；统一 SNR 质量口径必须先冻结相同输出支持范围。

## SNR 矩阵后续决策发现

- 将 RDA clean/noisy 裁剪到与 GBP 相同 `9x9` 窗口后，比较支持范围偏差被消除。
- 统一窗口下 seed 17 的 M1 RDA clean/noisy NRMS 为 `0.006040/0.019105/0.060421/0.189517`，GBP 为 `0.000907/0.002871/0.009089/0.028854`。
- 两种算法对相同 raw 噪声的传播不同，但双 seed 与首批 SNR 档位仍不足以冻结通用质量阈值。
- 当前不扩展更多 seed/SNR 档位；下一参考级缺口选择测试侧确定性分布式杂波模型。

## 确定性分布式杂波参考模型契约发现

- 测试侧杂波应复用现有点目标 raw echo 生成链路，通过大量确定性散射点叠加形成，
  避免在契约阶段发明独立生产传播模型。
- 固定 seed 只在散射点位置、复幅相和稳定遍历顺序全部显式时才具有跨实现可重复性；
  不得依赖标准库随机分布序列。
- 首批信杂比应定义为目标与杂波 raw-history 总能量比，并在混合前分别测量和精确缩放。
- RDA、GBP 与 BP 必须共享同一份 target-plus-clutter raw history，否则跨算法比较会混入
  不同杂波 realization。
- 规则网格、M1/M4、双密度、双 seed 和 `30/20/10/0 dB` 足以建立首批趋势证据，但不足以
  批准生产杂波、绝对功率、辐射定标、质量阈值或 Auto。

## 确定性分布式杂波参考模型实现发现

- 现有点目标接口不接受独立初始复相位；测试 helper 通过逐散射点生成 raw history 后乘
  确定性复系数，既复用生产传播相位和延迟，又不修改生产接口。
- 候选散射系数去除样本均值后，规则网格不会引入未声明的相干直流背景。
- 杂波 raw history 精确能量缩放可使 realized SCR 与 requested SCR 严格一致。
- M1 sparse seed 17 从 `30 dB` 到 `0 dB` 时，RDA/clean NRMS 从 `0.014621` 增至
  `0.619501`，GBP/clean NRMS 从 `0.014217` 增至 `0.606510`。
- M4 dense seed 29 保持相同总体退化趋势，且 BP/GBP 对所有首批混合输入继续逐样本一致。
- 首批规则网格证据仍不足以批准相关杂波、生产杂波、通用 SCR 阈值、质量警告或 Auto。

## 分布式杂波后续决策发现

- 增加更多规则网格密度只能扩展当前趋势，不能回答空间相关模型如何定义。
- 随机位置和相关杂波需要先冻结位置分布、相关函数、相关长度与功率归一化；当前证据
  不足以选择这些参数。
- 确定性热噪声与确定性杂波已分别完成独立验收，且都注入 raw pulse history，因此
  SNR/SCR 二维矩阵是当前风险最低、证据连续性最好的下一方向。

## 阶段 55 联合 SNR/SCR 矩阵发现

- 以纯目标 raw-history 能量分别缩放噪声与杂波，可以避免 SNR 和 SCR 相互隐式耦合。
- 固定全部参数时目标、噪声、杂波和联合输入可重复；只改变一个 seed 不影响另一分量。
- 浮点复数加法的不同结合顺序不能保证 bitwise 相等，但当前联合输入逐样本差异不超过
  `1e-15`，满足数值顺序无关。
- M1 完整 `3x3` 和 M4 三档哨兵矩阵均保持 BP/GBP 逐样本一致，且未发生杂波回波裁剪。
- 当前证据只覆盖固定网格、固定 seed、小场景，不足以冻结联合质量阈值、相关杂波模型、
  生产噪声/杂波或 Auto。
- 后续建设范围明确只覆盖 `1.1.4.4 SAR雷达组件`。
- 二维矩阵只能用于联合退化趋势研究，不能自动批准通用阈值、生产杂波或 Auto。

## 阶段 56 联合矩阵后续决策发现

- 增加中间 SNR/SCR 档位只会提高趋势采样密度，不能形成通用算法有效性阈值。
- 相关杂波需要先定义位置分布、相关函数、相关长度和功率归一化，当前无法可靠选择。
- requested/realized SNR/SCR 是输入描述，不足以直接成为质量警告、拒绝或 Auto 条件。
- 长期方案中 `1.1.4.4.3.4 辐射定标模块`仍未闭环；现有 RCS、聚焦图像及确定性干扰
  能力已足以进入测试侧定标契约阶段。

## 阶段 57 辐射定标契约发现

- 长期方案的 `K_cal` 反推式包含发射功率、增益和波长，但 RCS 反演式省略这些量，
  两式不能直接组成一致闭环。
- 首批应冻结吸收当前确定性信号链增益的 `K_image = sigma/(|I|^2 R^4)`，不宣称真实
  硬件绝对功率语义。
- 每个单点按自身反推定标因子时残差理论上为零，因此不能使用残差平方倒数作为多点
  融合权重。
- 显式正权重加权平均可形成确定性、可审计的首批多点融合口径。
- 未归一化聚焦图像是绝对 RCS 定标的必要条件；显示归一化图像必须禁止进入定标。

## 阶段 58 辐射定标实现发现

- 内部图像响应定标因子可以稳定闭合当前确定性点目标 raw echo 与 GBP 聚焦链路。
- M1 定标因子应用于 M4 三个隔离目标时，离散像素与方位偏置引入可测但小于 `0.01 dB`
  的辐射误差；因此聚焦链路验收应记录 dB 残差，而非要求逐位恢复 RCS。
- 显式权重多点融合行为可预测；单点自校准残差保持接近零。
- 联合噪声/杂波会改变峰值 RCS 反演结果，但当前没有证据冻结定标有效性阈值。
- 当前内部闭环仍不具备完整系统功率、天线增益、损耗和真实标定器语义。

## 阶段 59 辐射定标后续决策发现

- public 点目标 RCS 输入不能替代显式标定器身份；自动选择普通目标会使定标来源不可审计。
- 摘要级 public 输出不包含未归一化聚焦像素功率，无法解释公开定标结果的输入来源。
- 在目标像素定位、功率提取和跨周期生命周期未冻结前，不应接入 public Session。
- 完整系统因子与有效性阈值仍缺少必要参数和矩阵证据。
- 下一步先建立内部显式标定观测契约，可以推进链路集成而不提前扩大 public/replay 表面。

## 阶段 60 显式标定观测契约发现

- 标定目标身份、图像像素和孔径来源必须显式绑定，否则定标结果无法审计。
- 自动使用整幅图像全局峰值会在多目标场景中错误选择高 RCS 普通目标，因此首批禁止。
- 首批指定像素峰值功率足以连接现有聚焦链路；主瓣积分和亚像素定位需要单独误差口径。
- 不持有图像指针的不可变观测值对象可以避免图像生命周期和隐式缓存问题。
- 任一无效观测导致整次多点定标失败，比静默跳过更符合可审计性要求。

## 阶段 61 显式标定观测实现发现

- 指定像素观测可以把聚焦图像来源与定标数学计算清晰分离，并保留身份和孔径审计信息。
- GBP 与 BP 在相同 M1 指定像素上的功率严格一致，延续了两条后向投影路径的一致性。
- 原子观测转换避免无效多点列表留下部分可用样本，减少调用方误用风险。
- 首批观测对象不持有图像，可避免图像生命周期和跨周期缓存耦合。
- 当前仍没有自动像素定位、主瓣积分或跨周期生命周期证据，不能直接扩大 public 表面。

## 阶段 62 显式标定观测后续决策发现

- 显式观测解决了图像像素到定标样本的可审计转换，但尚未定义谁在 Session 执行时提供
  该观测请求。
- 在没有显式请求前接入 Session 会重新引入普通目标自动选择问题。
- 自动像素定位和主瓣积分需要处理多目标重叠、局部窗口和算法响应差异，当前不是低风险
  接入前置项。
- 下一步先冻结内部请求、执行时机和失败语义，可在不扩大 public 表面的情况下验证管线。

## 阶段 63 内部显式校准请求契约发现

- 将请求路径与实际聚焦图像路径显式匹配，可以避免调用方误用另一算法图像的像素坐标。
- 无状态执行器和原子失败能验证 Session 风格执行边界，同时不引入跨周期缓存语义。
- 定标失败不应影响原聚焦结果，也不应触发算法回退或 Auto。
- 首批 RDA/GBP/BP 路径矩阵足以验证执行器；L2/L3 public 接入仍需单独审批。

## 阶段 64 内部校准执行器实现发现

- 显式路径匹配可以可靠阻止用 GBP/BP 像素坐标误读 RDA 图像等跨算法误用。
- 候选结果构建后一次性替换输出，使失败路径不留下部分残差或部分定标状态。
- M1 GBP/BP 定标因子严格一致；RDA 定标因子差异可作为解释性记录，但不形成质量阈值。
- 执行器已具备 Session 风格调用边界，但尚未定义 public 请求来源，因此不能直接公开。

## 阶段 65 内部校准执行器后续决策发现

- 没有显式请求来源时接入 `SarSession` 只会形成永远无输出的分支，不能视为有效能力。
- 自动从普通目标列表生成请求会违反已冻结的标定器身份与像素来源契约。
- 辐射定标内部线已达到当前可审计边界，继续扩展需要新的 public 请求或完整系统参数。
- 下一步应回到长期方案剩余聚焦能力，重新排序 CSA、Omega-K、Auto、自聚焦和时变 PRF。

## 阶段 66 剩余聚焦能力审计发现

- CSA 和 Omega-K 不能仅复用现有 FFT facade 即开始实现；必须先冻结二维频率、相位项、
  插值和独立参考真值。
- 显式脉冲时间只解决时变 PRF 几何采样，不解决 RDA 均匀慢时间 FFT 假设。
- 自聚焦需要独立残余相位误差模型，不能只以图像熵下降作为正确性证据。
- RDA、GBP、BP 当前证据足以支持内部建议器，但不足以直接开放 public Auto。
- 先实现只建议、不执行的确定性选择器，可以审计规则而不引入自动回退风险。

## 阶段 67 内部聚焦选择器契约发现

- 调用目的必须显式区分常规成像、独立参考和 L3 非直线成像，否则 GBP/BP 选择缺少语义。
- L2 未启用补偿时不能自动切换 BP；当前 BP 证据针对 L3 航路点，不是 L2 通用降级路径。
- 算法不可用或尺寸超限时拒绝，比备用算法静默切换更符合当前审批边界。
- 首批选择器不使用质量阈值和运行时耗时，避免把参考矩阵趋势误当作通用选择门。

## 阶段 68 内部聚焦选择器实现发现

- 显式轨迹等级与调用目的足以确定当前批准的 RDA/GBP/BP 建议路径。
- 算法不可用时拒绝而不选择备用算法，保持了当前路径审批边界。
- RDA `1024x1024` 与 GBP/BP `128x128` 尺寸门可直接形成确定性建议门。
- 当前建议器不依赖质量阈值或执行耗时，因此不能被误解为运行时质量自适应 Auto。

## 阶段 69 内部聚焦选择器后续决策发现

- 没有显式调用目的时，Session 无法区分常规成像与独立 GBP 参考请求。
- 只公开建议 diagnostics 而不改变执行路径，会扩大 schema/replay 表面但缺少直接价值。
- public Auto 仍需要自动执行、失败回退、跨周期稳定性和结构化警告独立契约。
- CSA 不依赖 Stolt 插值，适合作为下一条 Phase 3 聚焦算法契约线。

## 阶段 70 CSA 数学与参考真值契约发现

- 现有 CSA 设计只给出六步流程，没有完整 chirp-scaling、SRC/RCMC 和方位压缩相位式，
  不足以直接实现完整算法。
- 未 shift 频率轴必须与现有 FFT bin 顺序保持一致，否则相位函数会施加到错误频点。
- `D(f_a)` 和 `alpha(f_a)` 提供了可独立验证的 CSA 几何基础，也能提前诊断无效 Doppler
  频点。
- 完整 CSA 必须验证中间域固定结果；只比较最终峰值无法发现相位函数符号或轴顺序错误。

## 阶段 71 CSA 频率几何基础实现发现

- 现有 RDA 与 CSA 几何基础可以共享同一未 shift Doppler bin 约定，偶数长度 Nyquist
  bin 均位于正频率位置。
- 将结构有效与 Doppler 域有效分开表达，可以保留诊断信息，同时明确阻止无效域进入
  后续完整处理。
- 无效 Doppler bin 使用有限占位值并单独计数，比生成 NaN/Inf 更适合确定性记录和测试。
- 参考斜距属于已批准 L1 CSA 输入边界，但首批频率几何公式不使用该值；完整相位函数
  契约必须明确其使用位置。

## 阶段 72 CSA 频率几何后续决策发现

- 仓库检索未发现可复用的完整 CSA 相位函数、固定复数中间域真值或独立参考脚本。
- RDA/GBP/BP 最终图像参考不能替代 CSA 中间域验证，无法可靠发现相位符号和参考距离错误。
- 当前内部选择器只应建议已有完整审批证据的 RDA/GBP/BP，CSA 几何基础不足以批准执行路径。
- Omega-K 可先冻结传播色散有效域和 Stolt 映射边界，但完整算法同样必须等待独立真值。

## 阶段 73 Omega-K 数学与参考真值契约发现

- 现有设计只有 Omega-K 四步流程，没有参考相位、Stolt 查询方向或越界语义。
- 双程距离波数必须包含 `4*pi/c`；与单程波数混用会使传播色散与查询位置整体错误。
- 从均匀目标 `K_z` 反求源距离频率，可以独立验证查询几何而不提前批准复数谱插值。
- 色散无效点与 Stolt 越支持区查询是不同失败类别，必须分别诊断。

## 阶段 74 Omega-K Stolt 几何基础实现发现

- 均匀目标 `K_z` 网格在非零 Doppler 下会把高端距离频率查询推到原支持区之外，
  因此完整 Stolt 插值必须显式冻结裁剪、填零或目标网格收缩策略。
- 色散有效不代表 Stolt 查询位于支持区，两个诊断必须独立保留。
- 零 Doppler 经波数反算会产生微小浮点残差；显式复用原频率可保持严格零 shift 契约。
- 当前几何诊断足以验证映射方向和边界，但不足以批准复数插值幅相误差或完整聚焦。

## 阶段 75 Omega-K Stolt 几何后续决策发现

- 仓库没有复数 Stolt 插值固定结果、幅相误差阈值或独立 Omega-K 参考脚本。
- 越支持区策略会直接改变输出带宽和边缘能量，不能由几何基础隐式决定。
- Omega-K 几何基础不足以批准内部选择器执行路径，完整算法和 public 接入继续后置。
- 自聚焦下一步应先建立可控残余相位误差注入真值，而不是直接以图像熵下降批准 PGA。

## 阶段 76 自聚焦残余相位误差与参考真值契约发现

- 自聚焦不能把全局常量相位和线性方位位移宣称为可恢复残余相位误差。
- 对离散剖面做最小二乘常量/线性去除，比直接删除输入多项式的 `a0/a1` 更适用于后续
  任意估计剖面。
- 显式低阶注入真值可以先验证可观测性和校正方向，无需提前实现 PGA 或修改生产图像。
- 图像熵下降只能作为后续效果指标，不能替代相位剖面恢复误差真值。

## 阶段 77 自聚焦相位误差真值基础实现发现

- 离散最小二乘去除能对混合二次/三次剖面正确隔离常量与线性不可观测投影。
- 校正剖面直接定义为可观测残余的相反数，可形成后续估计器的明确误差真值。
- 当前模块只生成诊断剖面，不修改图像，因此不能被误解为已实现 PGA 或生产自聚焦。

## 阶段 78 自聚焦相位真值后续决策发现

- 相位注入真值解决了可观测性基线，但没有提供 PGA 梯度估计、unwrap 或迭代收敛证据。
- 图像熵下降不能区分正确相位恢复与过度锐化，生产自聚焦继续后置。
- 显式脉冲时刻已存在，但均匀 PRF FFT 成像仍缺少非均匀慢时间重采样前置契约。

## 阶段 79 时变 PRF 慢时间采样与重采样契约发现

- L3 几何已有严格递增显式脉冲时刻，可作为非均匀慢时间输入来源。
- RDA/CSA/Omega-K 当前均使用单一 `prf_hz` 构造均匀 Doppler 轴，不能直接消费时变 PRF。
- 使用首尾跨度构造名义轴可避免外推，并保证首尾样本严格保持。
- 复数仿射信号是线性重采样的独立解析真值；复指数只能验证误差趋势，不能要求精确恢复。

## 阶段 80 时变 PRF 慢时间重采样基础实现发现

- 首尾跨度名义轴可以在不外推的前提下为每个内部查询找到有效插值区间。
- 均匀输入严格退化与非均匀输入成功重采样是两个独立状态，诊断需要分别表达。
- 复数仿射解析真值验证了线性插值方向和权重，但尚不能批准带限回波的生产误差阈值。

## 阶段 81 时变 PRF 重采样后续接入决策发现

- 向量重采样正确不等于二维 raw-history 接入正确，矩阵轴和列独立性仍需单独冻结。
- 缺失脉冲和大时间间隙会改变线性插值可信度，当前没有审批门，不能直接接入 RDA。
- 二维内部基础可先复用相同时间映射逐列验证，不需要扩大 Session 或 public 表面。

## 阶段 82 二维 raw-history 慢时间重采样契约发现

- 所有距离列必须共享同一慢时间映射，否则逐列计算差异会破坏同一方位行的时间语义。
- 列独立性是二维重采样的核心验收条件，可防止误把距离方向作为插值轴。
- 复用现有 `ComplexMatrix` 可保持 row=方位、col=距离约定，不需要扩大 public 类型。

## 阶段 83 二维 raw-history 慢时间重采样实现发现

- 逐列复用向量重采样可以保持距离列完全独立，并共享同一慢时间诊断。
- 现有 `ComplexMatrix` 没有二维构造函数，内部模块必须显式设置尺寸并分配 values。
- 二维仿射真值验证了轴和列独立性，但仍不足以批准带限 SAR 回波质量或 RDA 接入。

## 阶段 84 二维慢时间重采样后续接入决策发现

- 固定参考点场景和 RDA/GBP 比较能力足以建立重采样质量矩阵。
- 在抖动参数矩阵给出通过区前，直接接入 RDA 会把未经审批的插值误差引入默认路径。
- 缺失脉冲不是普通小抖动，必须在后续单独定义最大间隙与拒绝语义。

## 阶段 85 时变 PRF 重采样质量矩阵契约发现

- 首尾固定的正弦抖动能保持名义轴端点不变，并形成确定性强度矩阵。
- raw-history 误差与最终 RDA 图像误差必须同时记录，避免聚焦链掩盖或放大插值误差。
- 首批应先记录趋势和测量值，再依据证据冻结数值通过阈值。

## 阶段 86 时变 PRF 重采样质量矩阵实现发现

- 当前固定单点场景下，raw-history 与 RDA 图像误差随抖动比例稳定增加。
- `A=0.35` 的图像 NRMS 仍约 `5.26e-4`，说明该场景对线性重采样较宽容，但不能外推
  为多目标、高 Doppler 或缺失脉冲的通用阈值。
- 派生归一化指标的零抖动结果存在机器精度舍入，验收应使用数值零而非位级零。

## 阶段 87 时变 PRF 质量矩阵后续决策发现

- 单点小场景质量趋势不能外推为多目标、高 Doppler 或缺失脉冲的通用通过阈值。
- 缺失脉冲会形成显著大间隙，必须先诊断和拒绝，不能由线性重采样静默跨越。
- 在缺失脉冲边界冻结前，RDA 和 Session 接入继续后置。

## 阶段 88 缺失脉冲与最大慢时间间隙诊断契约发现

- 使用首尾跨度反推名义间隔会被缺失脉冲污染，因此缺失诊断必须接收显式 expected interval。
- `1.5x` 门可以区分首批小抖动与接近单脉冲缺失的间隙，并在恰好边界时保守拒绝。
- 疑似缺失计数只能作为诊断，不能被用来自动插入或修复样本。

## 阶段 89 缺失脉冲与最大慢时间间隙诊断实现发现

- 显式 expected interval 使间隙诊断不受缺失脉冲造成的首尾跨度污染。
- `1.5x` 边界保守拒绝能阻止线性重采样静默跨越疑似缺失脉冲。
- 门禁包装入口可在不改变已审批基础函数行为的前提下增加拒绝语义。

## 阶段 90 缺失脉冲诊断后续接入决策发现

- 固定参考场景的 `prf_hz` 是可信 expected interval 来源，可用于缺失拒绝矩阵。
- Session 固定硬件 PRF 不等于已审批的时变 PRF 请求语义，生产接入仍需单独契约。
- 缺失 case 的关键验收不是图像质量，而是必须在插值和聚焦前停止。

## 阶段 91 缺失脉冲拒绝参考矩阵契约发现

- 缺失脉冲拒绝矩阵必须显式证明 RDA 未尝试，而不仅是重采样返回失败。
- 删除 raw-history 行和对应显式时刻可以构造真实约 `2x/3x` 间隙，不应排序或补点。
- 拒绝 case 没有有效图像，不能被纳入图像质量下降趋势。

## 阶段 92 缺失脉冲拒绝参考矩阵实现发现

- 门禁允许 baseline 与小抖动 case 完成重采样和 RDA，同时在缺失 case 前停止。
- 显式 `rda_attempted=false` 和空输出共同证明拒绝 case 没有进入聚焦链。
- 相邻缺失与分离缺失可以分别验证单个大间隙估计和多个拒绝间隙累计语义。

## 阶段 93 缺失脉冲拒绝矩阵后续决策发现

- 门禁正确停止缺失 case，但生产接入仍需要显式请求和结构化失败传播契约。
- RDA 默认路径直接接入会改变固定 PRF 审批行为，继续后置。
- 内部请求执行器可以组合现有门禁与重采样，而无需扩大 Session 或 public 表面。

## 阶段 94 内部慢时间重采样请求契约发现

- expected interval 必须是显式请求字段，执行器不能从缺失后的时间轴猜测。
- 结构无效与缺失间隙拒绝应使用不同原因，便于后续调用方审计。
- 原子输出要求拒绝时矩阵为空，防止调用方误用部分重采样结果。

## 阶段 95 内部慢时间重采样请求执行器实现发现

- 组合现有间隙诊断与 raw-history 重采样即可形成无状态内部执行边界，无需扩大 Session 或 public API。
- 在执行重采样前完成全部结构和缺失间隙检查，可自然保证拒绝时无部分输出。
- 结构化拒绝原因和原子输出已通过默认/Eigen 3.3.9 双环境验证；生产链接入仍需独立审批。

## 阶段 96 内部慢时间重采样执行器后续接入决策发现

- Session 仍按固定硬件 PRF 与连续 pulse id 构造 aperture，RDA 也只接收单一 `prf_hz`。
- public/schema/replay 缺少显式慢时间请求、expected interval、拒绝原因和重采样诊断，直接接入会改变已审批默认行为。
- 生产接入继续后置；下一方向补齐多目标、高 Doppler 与确定性随机抖动质量证据。

## 阶段 97 扩展时变 PRF 重采样质量矩阵契约发现

- 多目标与方位偏置单点可复用现有参考场景能力，直接检验重采样误差是否依赖目标布局和 Doppler 历史。
- 固定 seed 随机抖动必须独立于标准库随机实现，才能保持跨编译器确定性。
- 不同 seed 的质量不保证单调；首批矩阵应记录有限有效指标，而非猜测通用阈值。

## 阶段 98 扩展时变 PRF 重采样质量矩阵实现发现

- E1-E3/J0-J4 全部通过内部执行器；`0.15` 抖动最大间隙比约为 `1.106-1.141`，未触发缺失脉冲拒绝门。
- 多目标 E2 的误差高于单目标场景；seed 17 随机 `0.15` 下 raw-history/RDA 图像 NRMS 为 `0.000657/0.000447`。
- 小型参考矩阵结果稳定且确定，但仍不足以冻结真实硬件时变 PRF 的通用生产阈值。

## 阶段 99 扩展时变 PRF 质量矩阵后续决策发现

- 时变 PRF 分支已形成内部契约、重采样、质量、缺失拒绝和显式执行器闭环，但生产请求与 replay 语义仍缺失。
- `9x64` 小型 L1 矩阵不能外推到真实硬件规模、调度或通用阈值，生产接入继续后置。
- 下一方向回到完整 CSA 前置缺口：完整相位函数、处理顺序和独立中间域参考真值。

## 阶段 100 CSA 完整相位函数与中间域参考真值契约发现

- 现有设计只给出 CSA 高层六步流程，没有完整相位公式、符号约定或固定复数中间域真值。
- 参考包必须显式携带操作顺序和处理域；执行器不能依据相位核名称猜测 FFT 轴或顺序。
- 通用中间域真值执行器可先验证轴、符号、共轭、归一化与阶段误差，而不提前宣称完整 CSA 已实现。

## 阶段 101 CSA 中间域真值执行器实现发现

- 显式操作列表消除了依据相位核名称猜测 FFT 轴和处理顺序的歧义。
- 非对称复数矩阵往返可直接发现轴交换、符号和 inverse 归一化错误。
- 执行器已能定位首个真值偏差并保持最终输出原子性，但具体 CSA 相位公式仍需独立审计来源。

## 阶段 102 CSA 中间域真值执行器后续决策发现

- 合成操作参考包只能批准真值执行器，不能替代真实 CSA 相位公式参考包。
- 在三个完整相位函数和逐阶段固定复数真值进入仓库前，完整 CSA 聚焦继续后置。
- Omega-K Stolt 几何已经具备，复数插值是可独立推进且可建立解析真值的下一前置能力。

## 阶段 103 Omega-K 复数 Stolt 插值契约发现

- 未 shift 距离频率轴在内存中不单调，复数插值必须先按频率值构建排序视图，不能直接使用相邻列。
- 笛卡尔线性插值可对仿射复数谱提供解析精确真值；复指数只适合误差趋势证据。
- 首批任何越支持区查询均原子拒绝，避免在未审批情况下引入填零、裁剪或外推策略。

## 阶段 104 Omega-K 复数 Stolt 插值实现发现

- 按频率排序的索引视图可在保持未 shift 输出列顺序的同时正确选择线性插值邻点。
- 仿射复数谱逐样本精确恢复，证明逐行隔离、复数笛卡尔插值与输出顺序正确。
- 首次完整回归失败来自测试命中计数期望错误，插值数值真值未失败；修正为实际 `6/4` 后完整回归通过。
- 完整未 shift 目标网格在非零 Doppler 行会产生越支持区 Stolt 查询；当前原子拒绝语义正确，但完整 Omega-K 必须先冻结目标网格收缩或越界策略。

## 阶段 105 Omega-K 复数 Stolt 插值后续决策发现

- 插值器已能正确处理支持区内查询，但完整目标网格不能在所有 Doppler 行保持有效。
- 自动填零、边界裁剪或外推会改变幅相和输出尺寸，不能作为未审批的隐式行为。
- 下一步应先诊断所有方位行共同有效的目标距离频率列，再决定是否批准显式网格收缩。

## 阶段 106 Omega-K 共同支持窗口诊断实现发现

- 原未 shift 列掩码与按频率排序的连续窗口是两个不同口径，诊断必须同时保留。
- 完整多 Doppler 网格可形成共同有效窗口，但边界列会因任一方位行越界而被排除。
- 诊断提供了显式网格收缩依据，但输出尺寸、距离轴和裁剪后插值仍需独立审批。

## 阶段 107 Omega-K 共同支持窗口后续决策发现

- 最大连续共同窗口可作为唯一首批显式收缩选择，避免调用方任意拼接非连续列。
- 收缩结果必须返回严格递增目标频率轴和原始列索引，才能审计输出尺寸与网格变化。
- 收缩频率域输出仍不能定义空间域距离轴；二维 IFFT 和完整聚焦继续后置。

## 阶段 108 Omega-K 显式网格收缩请求实现发现

- 源谱列数与收缩目标查询列数是独立维度，插值器的验证、输出和循环边界必须全部使用正确口径。
- 最大连续共同窗口可让完整多 Doppler 几何在不填零、不外推的情况下完成支持区内插值。
- 收缩频率轴严格递增且保留原列索引，但尚不足以定义空间域距离采样轴。

## 阶段 109 Omega-K 显式网格收缩后续决策发现

- 连续均匀频率子集足以定义 inverse-range FFT 的周期性相对延迟轴与采样间隔。
- 收缩带宽会改变距离分辨率和无模糊延迟窗口，不能沿用原完整网格诊断。
- 没有参考相位和斜距原点时，相对延迟轴不能解释为绝对斜距或地理距离。

## 阶段 110 Omega-K 收缩距离频率网格与相对延迟轴实现发现

- 收缩频率轴保持均匀时，可直接定义周期性 inverse FFT 相对延迟采样轴。
- 有效带宽、频率间隔和样本数分别决定分辨能力、无模糊窗口和相对延迟采样间隔。
- FFT 往返只能验证数值约定，不能提供绝对斜距原点或 Omega-K 参考相位。

## 阶段 111 Omega-K 相对延迟轴后续决策发现

- 相对延迟轴和 inverse-range FFT 已足以形成内部原子执行边界。
- 距离时域矩阵仍位于方位频率域，不能被解释为完整聚焦图像。
- 没有参考相位与绝对斜距原点时，继续禁止方位 IFFT 后的物理图像声明。

## 阶段 112 Omega-K 相对延迟变换执行器实现发现

- 逐行 inverse FFT 可保持方位频率行完全独立，并与现有 FFT facade 往返一致。
- 原子执行边界避免调用方误用无效轴或部分距离时域矩阵。
- 输出仍是方位频率-相对延迟中间域，不能被命名或解释为聚焦 SAR 图像。

## 确定性噪声与杂波 SNR/SCR 二维矩阵契约发现

- SNR 与 SCR 必须共同以纯目标 raw-history 能量为参考；若噪声相对 `target + clutter`
  缩放，SNR 会随 SCR 隐式变化，反之亦然。
- 噪声和杂波应先作为独立分量生成与缩放，再通过复数加法形成最终输入；相同分量下
  注入顺序必须不改变结果。
- 噪声 seed 与杂波 seed 必须彼此隔离，单独改变任一 seed 时另一分量保持不变。
- M1 适合受控完整二维矩阵；M4 使用少量哨兵组合即可验证多目标趋势，避免首批矩阵
  无证据膨胀。
- 联合矩阵仍只提供测试侧趋势证据，不能批准生产模型、相关杂波、通用阈值或 Auto。
## Stage 113 - Omega-K relative-delay transform follow-up

- The inverse range-frequency result is deterministic and useful, but its
  coordinates remain relative to the reduced frequency grid.
- An azimuth inverse transform alone cannot supply the missing physical phase
  reference or absolute slant-range origin.
- Production Omega-K image integration is therefore deferred until a contract
  fixes reference phase, absolute range, azimuth coordinates, normalization,
  and an independent point-target truth.
## Stage 114 - Omega-K reference phase and absolute range contract

- Absolute slant range requires an explicit reference range and delay sign
  convention; it cannot be recovered from the reduced matrix shape.
- Reference phase sign, transform domain, azimuth coordinates, and every
  normalization factor must be request data rather than hidden conventions.
- Acceptance requires independently generated point-target truth. FFT
  round-trip or executor-generated expected data cannot establish physical
  image correctness.
## Stage 115 - Omega-K reference mapping executor

- Absolute range mapping can be implemented independently from final image
  formation when delay direction, propagation speed, and reference range are
  explicit request fields.
- Requiring a declared reference-phase convention prevents an identity
  intermediate from being mistaken for a compensated physical image.
- The implementation preserves the validated matrix and coordinates atomically;
  analytic phase compensation and final azimuth transformation remain outside
  the accepted boundary.
## Stage 116 - Omega-K reference mapping follow-up

- The mapping executor supplies enough validated coordinates to safely apply an
  explicit per-range phase vector.
- It does not supply enough independent physical truth to derive a production
  phase model or approve final image formation.
- The next executor is therefore limited to request-supplied finite phase
  values and an explicit multiplication sign.
## Stage 117 - Omega-K explicit reference phase compensation

- A request-supplied phase vector can be applied deterministically without
  introducing hidden geometry or Fourier conventions.
- Explicit positive/negative application signs make conjugation behavior
  testable and prevent silent convention changes.
- The compensated result remains an intermediate because its phase model has
  not yet been validated against independent point-target truth.
## Stage 118 - Omega-K explicit phase compensation follow-up

- The compensated intermediate is sufficient for a deterministic numerical
  azimuth inverse transform when output coordinates and normalization are
  explicit.
- The transform cannot establish physical image correctness by itself.
- Independent point-target truth remains the acceptance gate for absolute
  location, peak response, and sidelobe behavior.
## Stage 119 - Omega-K numerical azimuth inverse transform

- The existing FFT facade supports a deterministic inverse transform down each
  range column and documents its built-in inverse normalization.
- Reporting an additional request normalization prevents that physical scaling
  choice from becoming implicit.
- The resulting matrix is intentionally classified as a numerical image
  candidate pending independent point-target truth.
## Stage 120 - Omega-K point-target image acceptance contract

- A physically meaningful acceptance gate must use truth generated
  independently from the imaging executor.
- Peak location, phase, and magnitude are necessary but insufficient; range and
  azimuth PSLR/ISLR are also required to characterize focus quality.
- Targets outside common support must be rejected before metrics are evaluated,
  and edge cases require explicitly separate tolerances.
## Stage 121 - Omega-K point-target acceptance evaluator

- A valid candidate can fail quality thresholds without being a malformed
  request; the evaluator reports these outcomes separately.
- Explicit mainlobe widths make PSLR/ISLR measurement deterministic and expose
  the chosen quality convention.
- The synthetic unit fixture verifies evaluator mathematics but is not itself
  independent physical evidence for production image acceptance.
## Stage 122 - Omega-K independent physical truth ingestion

- Independence requires verifiable provenance and a digest, not merely a flag
  supplied to the evaluator.
- The ingestion layer must preserve truth values and tolerances exactly; it
  cannot derive expectations or relax thresholds.
- Repository synthetic fixtures can verify parsing but cannot authorize
  production physical image acceptance.
## Stage 123 - Omega-K versioned truth manifest parser

- Strict ordered parsing provides deterministic C++11-compatible manifest
  validation without adding a dormant JSON/YAML dependency.
- Digest syntax validation and payload integrity verification are distinct
  responsibilities and must be reported separately.
- The synthetic fixture remains explicitly non-physical and non-independent.
## Stage 124 - Omega-K truth payload digest verification

- Integrity verification must hash the exact acquired bytes without parsing or
  normalization.
- Digest matching is independent from provenance and physical correctness.
- A portable C++11 implementation avoids binding SAR truth ingestion to a
  platform-specific cryptography API.
## Stage 125 - Omega-K portable truth payload digest verifier

- Portable C++11 SHA-256 matches standard empty-string and `abc` vectors.
- Exact-byte hashing makes payload normalization or parser behavior irrelevant
  to integrity verification.
- A successful digest match remains only an integrity statement, not a
  provenance or physical-correctness statement.
## Stage 126 - Omega-K atomic truth ingestion gate

- Parsing success and digest match are both required before truth data can be
  published by ingestion.
- Integrity checks must not alter or upgrade the manifest's physical-evidence
  and independence classifications.
- A synthetic fixture can validate the gate while remaining ineligible for
  production physical image acceptance.
## Stage 127 - Omega-K atomic truth ingestion gate

- Combining strict parsing and digest verification prevents partially trusted
  truth data from escaping ingestion.
- Separate rejection reasons retain useful diagnostics without publishing
  partial trusted state.
- Integrity success does not upgrade synthetic or non-independent evidence.
## Stage 128 - Omega-K truth ingestion follow-up

- Missing external physical truth blocks final physical-image acceptance, not
  deterministic repository robustness work.
- A separate eligibility decision is required before ingested truth may enter
  physical point-target evaluation.
- Eligibility authorizes evaluation only; it cannot imply image-quality pass.
## Stage 129 - Omega-K physical truth evaluation eligibility gate

- Evaluation eligibility can be decided independently from image-quality pass.
- Digest comparison must be case-insensitive because manifest syntax accepts
  both uppercase and lowercase hexadecimal.
- The gate correctly keeps synthetic fixtures ineligible even after successful
  parsing and integrity verification.
## Stage 130 - Omega-K eligible truth evaluation orchestration

- Eligibility and quality acceptance must remain separate outcomes.
- Dataset identity binding prevents eligibility for one truth package from
  authorizing evaluation with another package.
- Truth and tolerances must pass from the ingested manifest to the evaluator
  without modification.
## Stage 131 - Omega-K eligible truth evaluation orchestrator

- Dataset identity binding closes the gap between eligibility and evaluation.
- A quality failure remains a valid evaluated outcome and is not confused with
  orchestration rejection.
- The repository-side path is ready to consume eligible truth, but no external
  or measured physical dataset is present.
## Stage 132 - Omega-K physical acceptance readiness

- Repository-side Omega-K acceptance prerequisites now form a complete path
  from strict truth ingestion through identity-bound quality evaluation.
- The remaining physical-acceptance blocker is external evidence, not another
  repository wrapper.
- Synthetic fixtures must remain synthetic; relabeling them would invalidate
  the acceptance model.
## Stage 133 - Next incomplete SAR capability

- Existing autofocus work establishes controlled residual phase-error truth but
  explicitly lacks PGA support selection, gradient estimation truth, unwrap,
  and stopping criteria.
- PGA support selection and phase-gradient truth can be developed without
  external Omega-K physical truth.
- Production image correction remains deferred until the estimator stages are
  independently validated.
## Stage 134 - PGA support and phase-gradient truth contract

- Peak-relative support with first-index tie breaking provides a deterministic
  initial support policy.
- Forward wrapped phase differences remove unobservable constant phase offsets
  from the truth target.
- Estimation, unwrap, iteration, and production correction remain separate
  future boundaries.
## Stage 135 - PGA support and phase-gradient truth executor

- Peak-relative support and first-index tie breaking are deterministic across
  repeated execution.
- Wrapped forward differences correctly remove constant phase offsets and bound
  gradients to the selected convention.
- The next safe boundary is a bounded estimator compared against this truth,
  not unwrap or production correction.
## Stage 136 - PGA bounded phase-gradient estimator decision

- Adjacent conjugate products provide a deterministic wrapped phase-difference
  estimate without phase unwrap.
- Unsupported gaps must not be bridged because doing so invents unavailable
  aperture evidence.
- Truth comparison should evaluate only explicitly valid adjacent pairs.
## Stage 137 - PGA adjacent-sample phase-gradient estimator

- Adjacent conjugate products recover wrapped phase differences without
  requiring explicit phase extraction or unwrap.
- Explicit pair validity prevents unsupported gaps and zero-amplitude samples
  from silently contributing estimates.
- Estimator correctness still needs a dedicated truth-comparison gate.
## Stage 138 - PGA gradient estimator truth comparison contract

- Only pairs valid in both estimator and truth may contribute to metrics.
- Invalid-pair zero placeholders must be ignored rather than rewarded.
- Wrapped maximum and RMS errors provide bounded estimator acceptance without
  requiring phase integration.
## Stage 139 - PGA gradient truth comparison evaluator

- Joint validity alignment prevents invalid placeholders from improving
  estimator metrics.
- Wrapped error correctly handles estimates near opposite phase branch
  boundaries.
- The bounded estimator line now has truth and an acceptance mechanism; phase
  integration remains a separate decision.

## SAR Phase 1 closeout

- The current SAR component has a coherent closeout boundary at Stage 139.
- VS2015 can build the isolated SAR core and run the bounded PGA smoke chain.
- A complete VS2015 repository build is not a valid acceptance expectation
  while imported JSBSim and GoogleTest require newer C++ library/compiler
  support.
- Omega-K physical acceptance still depends on external or measured truth;
  adding more repository-side wrappers would not remove that blocker.
- Configure-time source-tree CRLF/BOM rewriting is unsuitable as a default;
  it is now an explicit VS2015 fallback and the isolated SAR targets compile
  successfully without it.

## Stage 141 - Public external raw IQ input

- A complete external aperture can reuse the current RDA path without changing
  the internal focusing algorithm.
- External IQ cannot safely enter L2 or BP until per-pulse trajectory metadata
  is supplied and validated.
- Summary replay must reject external IQ rather than silently serialize an
  incomplete cycle input.

## Stage 142 - External raw IQ pulse trajectory and BP

- BP can safely consume one complete actual trajectory because its internal
  contract already operates on per-pulse platform states.
- External-IQ L2 motion compensation cannot reuse the same contract because it
  requires both ideal and actual trajectories.
- L1 RDA continues to use nominal configured geometry; supplied external pulse
  states are explicitly ignored rather than partially applied.

## Stage 143 - External raw IQ dual-trajectory L2 motion compensation

- The existing first-order compensator can safely consume external IQ when
  actual and ideal trajectories are both explicit and independently valid.
- Estimating the ideal trajectory from actual navigation data would introduce
  a separate navigation/modeling contract and is not implied by this stage.
- Keeping the two trajectory lists explicit prevents accidental use of the
  actual trajectory as its own zero-error reference.
