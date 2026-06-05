# 任务计划：SAR Phase 1 工程化建设与 Phase 2 参考级成像扩展

## 目标

基于当前 SAR 探索成果，将 SAR 模块从方案文档推进到可实施、可验证、可审批的 1Q 工程模块。Phase 1 只交付最小闭环：公共 API、LFM、匹配滤波、距离向脉冲压缩、点目标原始回波、L1 直线条带 RDA、脉冲环形缓冲区、基础质量指标和确定性测试。

## 当前基线

已新增/修订的方案文件：

- `sar_construction_scheme_complete.md`
- `SAR_MODULE_DESIGN.md`
- `SAR_PHASE1_ENGINEERING_CONTRACT.md`

当前结论：

- Phase 1 固定 RDA，不启用 Auto。
- GBP/BP/CSA/Omega-K、L2/L3、运动补偿、自聚焦、辐射定标、HDF5/GeoTIFF、GPU、机动/侦察/融合均不进入 Phase 1。
- 现有 `common/numerics::ZFFT1D` 是 O(n^2) 朴素 DFT，只能支撑小型单元测试，不能支撑正式 SAR 成像性能。

## 阶段总览

| 阶段 | 名称 | 状态 | 验收口径 |
|---|---|---|---|
| 0 | 规划与契约冻结 | complete | 三份 SAR 文档完成，Phase 1 边界明确 |
| 1 | 公共 API 与构建骨架 | complete | `#include "1q/sar/sar.hpp"` 可编译，CMake 分 engine/core |
| 2 | FFT 后端与数值基础 | complete | Eigen 3.4.0/3.3.9 正确性通过，SAR engine C++11 + Eigen 3.3.9 编译门通过 |
| 3 | 信号链路 | complete_current_platform | LFM、匹配滤波、距离压缩单元测试通过 |
| 4 | 几何、原始回波与缓冲区 | complete_current_platform | 点目标双程延迟、分数 PRF、pulse_id 连续性测试通过 |
| 5 | RDA 聚焦实现 | complete_current_platform | 固定点目标场景聚焦指标通过 |
| 6 | Session/Trace/Replay 集成 | complete_current_platform | `SarSession::StepWithResult()` 真实 raw/RDA 管线与摘要级 trace/replay 闭环通过 |
| 7 | CI 验收与审批包 | complete | SAR 专属 CI、1024x1024 性能/内存、C++11 兼容门和审批报告完成 |
| 8 | Phase 2 决策门 | complete | 已批准“参考级成像与算法对比闭环”，Auto 继续后置 |
| 9 | Phase 2A 参考质量闭环与 Sinc RCMC | complete_current_platform | 确定性参考场景、统一质量指标、linear/sinc 对照通过 |
| 10 | Phase 2B 小场景 GBP 与跨算法比较 | complete_current_platform | GBP 尺寸门、相位重参考、RDA/GBP 对比通过 |
| 11 | Phase 2C 扩展审批门 | complete | Auto 继续后置；下一方向建议 L2 轨迹与一阶运动补偿 |

## 阶段 0：规划与契约冻结

状态：`complete`

已完成：

- 修订 `sar_construction_scheme_complete.md`，统一 Phase 1 RDA 默认和算法编号。
- 修订 `SAR_MODULE_DESIGN.md`，补齐 1Q 公共 API 前置要求和阶段化计划。
- 新增 `SAR_PHASE1_ENGINEERING_CONTRACT.md`，冻结 Phase 1 工程契约。

验收结果：

- 旧冲突 `algorithm=5 -> RDA`、`4=GBP,5=RDA`、全图 `<1e-10` 等已清除。
- Phase 2+ 能力均列为非目标或暂缓。

## 阶段 1：公共 API 与构建骨架

状态：`complete`

目标：

建立 SAR 模块的公开入口、配置模型、会话门面和构建清单，形态对齐 `airborne_radar`、`electronic_surveillance_radar`、`electro_optical_sensor`。

任务：

1. 新增 `include/1q/sar/sar.hpp`。状态：`done`
2. 新增 `include/1q/sar/config/*`。状态：`done`
   - `sar_config.hpp`
   - `SarHardwareConfig.h`
   - `SarMissionConfig.h`
   - `SarPolicyConfig.h`
   - `SarEnvironmentConfig.h`
   - `SarSessionConfig.h`
   - `SarRuntimeConfigPatch.h`
3. 新增 `include/1q/sar/session/*`。状态：`done`
   - `SarCycleInput.h`
   - `SarCycleResult.h`
   - `SarSession.h`
   - `SarSessionFactory.h`
   - `SarTraceSession.h`
   - `SarReplaySession.h`
4. 新增 `src/sar/CMakeLists.txt`，分出 `SAR_ENGINE_SOURCES` 与 `SAR_CORE_SOURCES`。状态：`done`
5. 将 SAR 源文件接入 `src/CMakeLists.txt` 和安装清单。状态：`done`
6. 增加 public include contract 测试。状态：`done`
7. 运行配置、构建和合同测试验证。状态：`done`

验收：

- 只包含 `1q/sar/sar.hpp` 的 contract 测试可编译。
- SAR 公共头不直接暴露内部算法对象、Eigen 可变矩阵、裸指针或锁。
- 默认配置中不可触达 Phase 2+ 算法。

已通过验证：

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_contract_tests`
- `build/llvm-ninja-debug-local/bin/1q_contract_tests`
- `cmake -DSOURCE_DIR=/Users/aurora/Code/1q-sar-phase1 -P tests/contract/check_public_api_boundary.cmake`

风险：

- 命名空间需最终决策：`sar::...` 还是 `oneq::sar::...`。
- replay 已冻结为 Phase 1 摘要级；全图复数矩阵与外部 artifact 引用后置。

## 阶段 2：FFT 后端与数值基础

状态：`complete`

目标：

解决 SAR 成像的频域计算基础，避免用当前 O(n^2) `ZFFT1D` 承担大尺寸 RDA。

任务：

1. 评审可选 FFT 后端。状态：`done_for_initial_gate`
   - Conan/CMake 依赖引入。
   - 仓库内 vetted FFT。
   - Eigen FFT wrapper（若当前依赖实际可用）。
2. 定义并实现统一 FFT API。状态：`done_current_platform`
   - `Fft1D(input, inverse)`
   - `FftRows(matrix, inverse)`
   - `FftCols(matrix, inverse)`
3. 固定归一化约定。状态：`done_current_platform`
   - forward 不归一化。
   - inverse 除以 N。
4. 增加 round-trip、delta pulse、known sinusoid 测试。状态：`done_current_platform`
5. 明确 CI 初始图像尺寸上限。状态：`done_for_fft_facade_only`

验收：

- 1D FFT round-trip 误差满足严格复数容差。
- rows/cols 变换轴向正确。
- 大尺寸性能声明必须等后端批准后才能写入验收。

阻断条件：

- FFT 后端未定时，不进入 1024x1024 及以上 RDA 性能验收。

当前研究记录：

- `docs/sar_fft_backend_research.md`

当前未完成：

- 已通过 `sar_cxx11_compat` 使用 C++11 + Conan Eigen 3.3.9 编译全部 SAR engine 源文件。
- Windows/VS2015 不是 Phase 1 强制审批门。
- 已建立当前平台 1024x1024 二维 FFT、内部 RDA、真实点目标管线和 public Session 性能基准。
- 尚未冻结 vendored 后端备选。

## 阶段 3：信号链路

状态：`complete_current_platform`

目标：

完成 LFM、匹配滤波和距离向脉冲压缩的确定性实现。

任务：

1. 实现 `LfmWaveform`。状态：`done`
   - `T = BT / B`
   - `K = B / T`
   - `N_s = ceil(T * f_s)`
   - `s[n] = exp(j*2*pi*(f0*t + 0.5*K*t^2))`
2. 实现匹配滤波。状态：`done`
   - 时域 `h[n] = conj(s[N_s - 1 - n])`
   - 或频域 `H[k] = conj(S[k])`
   - 禁止重复共轭。
3. 实现距离压缩。状态：`done`
   - 线性卷积补零 `L_f >= N_x + N_s - 1`
   - Phase 1 生产路径返回与 range bins 对齐的 cropped 输出。
4. 实现 PSLR/ISLR/3dB 宽度基础指标。状态：`done`

验收：

- LFM 采样点数、调频斜率、瞬时频率单调性通过测试。
- 匹配滤波主峰位置和幅度稳定。
- 压缩脉冲距离分辨率按 `c/(2*f_s)` 换算。
- 三种主瓣估计方法至少先实现 3dB 与 IRW，20dB 可后置但接口预留。

已通过验证：

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*'`

当前未完成：

- 20dB 宽度估计已在后续阶段实现。
- 点目标原始回波和 `PulseRingBuffer` 已在阶段 4 完成当前平台实现，并已在后续阶段接入 `SarSession` 生产链路。

## 阶段 4：几何、原始回波与缓冲区

状态：`complete_current_platform`

目标：

建立 L1 直线条带点目标回波和快慢时间解耦机制。

任务：

1. 实现本地场景坐标。状态：`done`
   - x = azimuth
   - y = ground range
   - z = altitude
2. 实现 L1 平台轨迹。状态：`done`
   - 匀速直线。
   - 常 PRF。
   - pulse_id 单调递增。
3. 实现点目标回波。状态：`done`
   - `R[p,q] = norm(platform[p] - target[q])`
   - `tau[p,q] = 2R/c`
   - `n_delay = round(tau*f_s)`
   - 相位 `exp(-j*4*pi*R/lambda)`
   - 相对幅度 `sqrt(rcs)/R^2`
4. 实现 `PulseRingBuffer`。状态：`done`
   - duplicate pulse_id 拒绝。
   - range read 要求连续。
   - latest-N 要求连续。
   - overflow sticky flag。
   - fractional PRF carry。

验收：

- 单目标中心场景延迟和峰值位置正确。
- 三目标分离场景不发生错误合并。
- near-edge 场景有 clipping diagnostics。
- fractional PRF 场景不丢平均 PRF。

已通过验证：

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*'`

当前未完成：

- 生产链路已通过 `SarSession` 生成二维 pulse history 矩阵。
- `PulseRingBuffer` 已接入 `SarSession`，作为跨周期 latest-N aperture 累积机制。
- 尚未处理 ECEF/geodetic 场景输入。

## 阶段 5：RDA 聚焦实现

状态：`complete_current_platform`

目标：

完成 Phase 1 唯一聚焦算法：L1 直线条带 RDA。

任务：

1. 实现 RDA 阶段。状态：`done`
   - range compression
   - azimuth FFT
   - RCMC
   - azimuth matched filter
   - azimuth IFFT
   - focused complex image extraction
2. 固定参考参数。状态：`done`
   - `K_a = 2*v^2/(lambda*R_0)`
   - `H_az(f_a) = exp(j*pi*f_a^2/K_a)`
   - `delta_r_cm = lambda^2*R_0*f_a^2/(8*v^2)`
   - `delta_n_cm = delta_r_cm/delta_r`
3. RCMC 第一版允许 linear interpolation。状态：`done`
4. 记录诊断。状态：`done`
   - reference range
   - doppler rate
   - range bin spacing
   - out-of-bounds samples
   - linear/sinc RCMC 标记。

验收：

- `sar_point_single_center` 峰值位置误差在约定 bins 内。
- `sar_point_three_separated` 三目标可分辨且相对峰值顺序正确。
- RDA diagnostics 完整。
- 默认配置不可选择 GBP/BP/CSA/Omega-K/Auto。

已通过验证：

- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*'`

当前限制：

- 当前 RDA 验证限定 L1 broadside 点目标场景。
- 尚未建立真实二维性能基准。
- 已在后续阶段接入 `SarSession`、trace 和 replay。

## 阶段 6：Session/Trace/Replay 集成

状态：`complete_current_platform`

目标：

把算法链路接入 1Q 会话、运行时、trace 和 replay。

任务：

1. 实现 `SarSession::Step()` 和 `StepWithResult()`。状态：`done_current_platform`
2. 定义 `SarCycleInput`：
   - cycle dt
   - platform state
   - point targets
   - scene/window config
   - optional runtime controls
   状态：`done_basic_public_contract`
3. 定义 `SarCycleResult`：
   - executed/reused/failed 状态
   - focused image summary
   - metrics
   - diagnostics
   - degradation/error reason
   状态：`done_basic_public_contract`
4. 新增 flatbuffer schema：
   - `sar_replay.fbs`
   - `sar_session_replay.fbs`
   状态：`done_current_platform`
5. 实现 trace/replay codec 和 round-trip 测试。
   状态：`done_current_platform`

当前已完成：

- `SarSession::StepWithResult()` 已接入 LFM、点目标 raw echo、距离压缩和 L1 RDA。
- Session 使用 public `SarCycleInput` 的平台姿态与点目标，转换为 Phase 1 local Cartesian 场景。
- 启用 RDA 时增加运行时尺寸门：当前平台已验证并允许 `range_sample_count <= 1024` 且 `azimuth_pulse_count <= 1024`。
- 启用 RDA 时要求 raw echo generation 开启，避免空 raw history 进入内部 RDA。
- 无效 `dt_sec` 和运行时失败保持结构化错误，并在已有上一帧时复用上一帧输出。
- `SarTraceSession` 可写入 `session_config`、`cycle_input`、`runtime_config_patch`、`cycle_output` FlatBuffers replay events。
- `ReplaySarTrace()` 可读取 SAR trace，重建 Session，应用 runtime patch，重新执行 cycle input，并比对 `SarCycleResult` 摘要输出。

验收：

- 结构化失败不抛异常，不产生未定义输出。状态：`done_current_platform`
- `SarSession::StepWithResult()` 对小型点目标场景产生 raw/range/L1 image 完成标记。状态：`done_current_platform`
- trace/replay 对同一输入产生一致摘要。状态：`done_current_platform`
- replay 策略明确：Phase 1 当前平台记录摘要，不记录全图矩阵；全图矩阵或外部 artifact 引用后置审批。状态：`done_current_platform`

已通过验证：

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*:SarSessionPipelineTest.*'`
- `cmake --build --preset llvm-ninja-debug --target 1q_contract_tests`
- `build/llvm-ninja-debug-local/bin/1q_contract_tests`
- `cmake -DSOURCE_DIR=/Users/aurora/Code/1q-sar-phase1 -P tests/contract/check_public_api_boundary.cmake`
- `cmake --build --preset llvm-ninja-debug --target 1q_replay_fast_tests`
- `build/llvm-ninja-debug-local/bin/1q_replay_fast_tests`

当前限制：

- trace/replay 只保存 public `SarCycleResult` 摘要，不保存 focused complex image 全矩阵。
- `PulseRingBuffer` 已作为跨周期慢时间累积机制接入 Session；当前只覆盖小场景 Phase 1 路径。
- 当前 AppleClang 已验证 Conan Eigen 3.3.9；Windows/VS2015 仍未验证。
- 1024x1024 public Session 真实点目标端到端与峰值内存已测量；超过该尺寸仍不批准。

## 阶段 7：CI 验收与审批包

状态：`complete`

目标：

形成可交付的 SAR Phase 1 审批包和测试矩阵。

任务：

1. 增加测试标签：
   - `sar_unit`
   - `sar_contract`
   - `sar_integration`
   - `sar_performance`（当前覆盖 1024x1024 二维 FFT、内部 RDA、真实点目标内部管线和 public Session）
   状态：`done_current_platform`
2. 固定参考场景：
   - `sar_point_single_center`
   - `sar_point_three_separated`
   - `sar_point_near_edge`
   - `sar_ring_buffer_fractional_prf`
   状态：`done_current_platform`
3. 输出指标：
   - peak location error
   - range 3dB width
   - azimuth 3dB width
   - PSLR
   - ISLR
   - image entropy
   状态：`done_current_platform`
4. 编写 `docs/sar_phase1_acceptance_report.md`。状态：`done`

验收：

- contract/unit/integration 全部通过。状态：`done_current_platform`
- performance 若未启用，报告必须明确 FFT 后端未批准，不得伪造大尺寸性能。状态：`done`
- Phase 2+ 能力仍不可从默认配置触达。状态：`done_current_platform`

已通过验证：

- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*:SarSessionPipelineTest.*:SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`，结果：42/42 passed
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_performance --output-on-failure`，结果：1/1 CTest entry passed、内部 4/4 gtests passed；Debug 构建中 1024x1024 二维 FFT 约 344 ms、内部 RDA 合成场景约 996 ms、真实点目标内部管线约 1493 ms、public Session 约 1421-1496 ms
- `cmake --preset llvm-ninja-debug-eigen339` 与独立 Eigen 3.3.9 SAR 验证，FFT/信号链/RDA/Session 25/25 passed，`sar_performance` passed
- `ctest --test-dir build/llvm-ninja-debug-eigen339-local -L sar_cxx11_compat --output-on-failure`，结果：1/1 passed
- `build/llvm-ninja-debug-local/bin/1q_replay_fast_tests`
- `build/llvm-ninja-debug-local/bin/1q_contract_tests`

当前限制：

- 1024x1024 public Session 隔离运行峰值常驻内存约 137035776 bytes；超过 1024x1024 尚未批准。
- Windows/VS2015 不作为 Phase 1 强制审批门。
- 全图复数矩阵 replay 后置。

## 阶段 8：Phase 2 决策门

状态：`complete`

启动条件：

- 阶段 1-7 全部完成。
- Phase 1 acceptance report 通过。
- 用户明确批准扩展范围。

批准结论：

1. Phase 2 主线选择“参考级成像与算法对比闭环”。
2. Auto algorithm selector 继续后置，只有 RDA 与 GBP 分别完成独立质量和性能审批后才允许再次评审。
3. Phase 2 保持 Phase 1 的 public Session `1024x1024` 上限，不扩大默认运行边界。
4. 全图复数矩阵 replay、L2/L3、运动补偿、辐射定标和 HDF5/GeoTIFF 继续后置。

每个方向必须单独新增工程契约，不允许直接在 Phase 1 实现中暗加。

## 阶段 9：Phase 2A 参考质量闭环与 Sinc RCMC

状态：`complete_current_platform`

目标：

- 建立 RDA 与后续 GBP 共用的确定性参考场景和质量指标口径。
- 在保留 linear RCMC 回归路径的前提下新增显式 Sinc RCMC。
- 用质量改善和性能代价共同审批 Sinc RCMC，不以“测试通过”替代算法证据。

任务：

1. 新增内部图像质量评估组件。状态：`done_current_platform`
   - 峰值位置与幅度。
   - 距离向/方位向 3dB 宽度。
   - PSLR、ISLR、图像熵。
2. 新增归一化复图像比较和全局常数相位重参考能力。状态：`done_current_platform`
3. 将参考场景生成从单个测试夹具提升为可复用、确定性的测试支持层。状态：`done_current_platform`
4. 将 RCMC 配置从 bool 升级为显式 `none/linear/sinc` 内部枚举。状态：`done_current_platform`
5. 实现有限核 Sinc RCMC，并增加边界与诊断测试。状态：`done_current_platform`
6. 建立 linear/sinc 相同场景质量和性能对照。状态：`done_current_platform`
   - 已证明已知带限分数采样输入上 Sinc 误差低于 linear。
   - 已记录 1024x1024 独立 RCMC 性能代价。
   - 成像质量默认优越性需等待 GBP 独立参考真值，不提前批准；该门禁不阻断 Phase 2A 完成。

验收门：

- 指标对人工构造图像给出确定结果，边界和零能量输入明确拒绝或返回定义值。
- Sinc RCMC 不得静默成为 Phase 1 public Session 默认路径。
- 至少一个经批准的参考场景证明 Sinc 相对 linear 的改善；若无改善证据，不批准默认启用。
- SAR 专属 CI、C++11 + Eigen 3.3.9 编译门继续通过。

## 阶段 10：Phase 2B 小场景 GBP 与跨算法比较

状态：`complete_current_platform`

目标：

- 实现作为高精度参考算法的小场景 GBP。
- 使用 Phase 2A 的统一指标比较 RDA 与 GBP。

冻结边界：

- GBP 初始尺寸上限建议 `128x128`，提高上限必须经过性能与内存审批。
- GBP 只允许显式选择，不进入 public Session 默认路径，不作为自动降级目标。
- 相位重参考只允许消除全局常数相位差，不得掩盖相位斜坡、RCMC 错误或散焦。

已完成：

1. 新增内部 `FocusSmallSceneGbp()` 参考聚焦内核。
2. 冻结图像坐标：行=`x/azimuth`，列=`y/range`，固定图像平面 `z`。
3. 使用与 RDA 相同的 raw pulse history、匹配滤波和平台逐脉冲位置。
4. 逐脉冲距离压缩、线性距离插值和 `+4*pi*R/lambda` 相位补偿。
5. 严格拒绝任一维超过 `128` 的 GBP 图像。
6. 单点目标、三点目标幅度顺序、RDA/GBP 同场景比较均通过。
7. RDA/GBP 相同窗口比较结果：
   - 全局相位偏移约 `-0.047659 rad`。
   - 单位能量 NRMS 约 `0.042218`，批准阈值 `<0.1`。
   - 相干相关系数约 `0.999109`，批准阈值 `>0.99`。
8. `128x128` GBP Debug 参考场景约 `0.149 s`，通过当前平台独立性能门。

## 阶段 11：Phase 2C 扩展审批门

状态：`complete`

候选方向：

1. 在 RDA/GBP 双算法证据齐备后审批 Auto。
2. 转入 L2 轨迹误差与一阶运动补偿。
3. 继续后置的辐射定标或图像产品输出。

审批结论：

1. Auto 不批准，继续后置。
   - 当前只有 RDA 与严格受限 GBP，BP/CSA/Omega-K 未实现和审批。
   - 当前场景只覆盖 L1 broadside 点目标。
   - Auto 选择阈值、降级规则和结构化诊断尚未冻结。
   - GBP 不进入 public Session，也不得作为自动降级目标。
2. Sinc 继续作为内部显式路径，不成为 public Session 默认值。
   - 相对 GBP 的单点参考场景中，Sinc NRMS `0.042107`，linear NRMS `0.042218`。
   - 改善可测但极小，而 Sinc 独立 RCMC 代价约为 linear 的 7.4 倍。
3. 下一扩展方向建议 L2 轨迹误差与一阶运动补偿，但必须另行批准并新增工程契约。

## 当前待决策问题

| 编号 | 问题 | 影响 | 建议 |
|---|---|---|---|
| D1 | FFT 后端 | 阻断大尺寸 RDA | 优先研究 Conan/Eigen FFT 可行性 |
| D2 | SAR 命名空间 | 影响所有公开头 | 对齐现有模块，优先使用 `sar::{config,session,...}` |
| D3 | replay 图像存储策略 | 影响 schema 与文件体积 | Phase 1 先存摘要，必要时外部 artifact |
| D4 | CI 图像尺寸 | 影响耗时和稳定性 | 先用小场景，性能标签单独启用 |
| D5 | RCMC 插值 | 影响精度和复杂度 | Phase 1 linear 已冻结；Phase 2A 实施显式 sinc 对照 |
| D6 | Phase 2 主线 | 决定扩展顺序 | 已批准参考级成像与算法对比闭环 |
| D7 | Auto 启用时机 | 影响算法选择可信度 | 继续后置到 RDA/GBP 双算法独立审批完成 |

## 错误与风险记录

| 风险 | 当前处理 |
|---|---|
| 用当前 O(n^2) DFT 实现大图 RDA | 阶段 2 设为前置门 |
| 自动选择在算法未成熟前进入默认路径 | Phase 1 明确禁止 |
| 辐射定标被误认为已有 RCS helper 可支撑 | Phase 1 拒绝，Phase 4 单独契约 |
| 机动/侦察行为混入 SAR 模块验收 | 审批清单和本计划明确排除 |
| 在第二算法未成熟前启用 Auto | Phase 2A/2B 禁止 Auto，Phase 2C 单独审批 |
| Sinc RCMC 仅以单测通过宣称改善 | 必须提供相同参考场景的质量与性能对照 |
| GBP 复杂度导致运行阻塞 | 初始冻结严格小场景尺寸门，超限明确拒绝 |
| 在 GBP 真值前宣称 Sinc 成像质量默认优于 linear | 当前只批准插值精度证据和性能测量，默认切换继续门禁 |

## 遇到的错误

| 错误 | 尝试次数 | 解决方案 |
|---|---:|---|
| GBP `LocalPoint{x,y,z}` 在 C++11 兼容门下编译失败 | 1 | 改为默认构造后逐字段赋值，保持 C++11 |
| RDA/GBP 仅相位对齐的 NRMS 被全局累积增益放大 | 1 | 两幅图分别单位能量归一化后计算形状 NRMS |

## 下一步

当前 Phase 2 计划已完成。下一步需单独批准 L2 轨迹误差与一阶运动补偿工程契约；批准前不开始实现。
