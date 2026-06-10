# 任务计划：SAR 工程化建设、参考级成像与 L3 BP

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
| 12 | L2 工程契约冻结 | complete | L2 连续扰动轨迹和一阶运动补偿边界明确 |
| 13 | L2 连续扰动轨迹 | complete_current_platform | 固定种子、零扰动退化和连续位置轨迹通过 |
| 14 | 一阶运动补偿闭环 | complete_current_platform | 包络/相位补偿相对理想参考产生可测改善 |
| 15 | L2 后续扩展决策门 | complete | 选择 L2 与一阶运动补偿受控接入 public Session |
| 16 | L2 Session 接入契约 | complete | 默认关闭、replay 保真和强制补偿边界明确 |
| 17 | L2 公开配置与 replay | complete_current_platform | 配置、schema、codec 和 round-trip 通过 |
| 18 | L2 Session 执行闭环 | complete_current_platform | Session 非零轨迹、补偿诊断、replay 和回归通过 |
| 19 | L2 后续扩展决策门 | complete | 选择 L3 航路点轨迹；外部轨迹和多参考点继续后置 |
| 20 | L3 航路点轨迹契约 | complete | 显式时间航路点、显式脉冲时刻和退化边界明确 |
| 21 | L3 航路点轨迹几何 | complete_current_platform | 确定性插值、显式脉冲时刻和严格退化测试通过 |
| 22 | L3 raw echo 与成像退化基线 | complete_current_platform | L3 经 L1-RDA 显著退化，L3-GBP 建立参考 |
| 23 | L3 后续处理决策门 | complete | 选择先审计现有一阶补偿对 L3 的适用性 |
| 24 | L3 一阶补偿适用性审计 | complete_current_platform | 一阶补偿显著改善，空间残余非零但当前很小 |
| 25 | L3 后续算法决策门 | complete | 不批准二阶补偿/BP；选择先建立适用边界矩阵 |
| 26 | L3 一阶补偿适用边界矩阵 | complete_current_platform | `0-6 m` 当前门通过，`12 m` 明确失效 |
| 27 | L3 失效区后续决策门 | complete | 选择共享核心的 L3 BP 路径，二阶补偿继续后置 |
| 28 | L3 BP 工程契约 | complete | 冻结 BP/GBP 共享核心、遍历顺序和验收边界 |
| 29 | L3 BP 内部闭环 | complete_current_platform | 逐样本一致、L3 失效区质量和性能通过 |
| 30 | L3 BP 后续接入决策门 | complete | 选择受控 public Session 接入契约，Auto 继续后置 |
| 31 | L3 BP Session 接入契约 | complete | 冻结 waypoint、时间基准、replay、算法和尺寸门 |
| 32 | L3 BP 公开配置与 replay | complete_current_platform | waypoint/policy/output schema、codec 和 round-trip 通过 |
| 33 | L3 BP Session 执行闭环 | complete_current_platform | L3 raw echo、BP 聚焦、诊断、replay 和回归通过 |
| 34 | Phase 2 完成度审计 | complete | 当前小场景闭环完成；代表性参考矩阵仍不足，Auto 继续后置 |
| 35 | 参考场景矩阵扩展契约 | complete | 冻结 L1/L2/L3、二维目标与统一比较矩阵 |
| 36 | 参考场景矩阵实现 | complete_current_platform | M1-M7 矩阵、质量结果和双环境回归通过 |
| 37 | 参考矩阵后续决策门 | complete | 选择图像边界与采样/硬件参数适用性矩阵 |
| 38 | 边界与参数矩阵契约 | complete | 冻结边界分类、B1-B4、P1-P4 与验收门 |
| 39 | 边界与参数矩阵实现 | complete_current_platform | B1-B4/P1-P4 分类、适用边界和双环境回归通过 |
| 40 | 方位采样充分性决策门 | complete | 排除 Nyquist/RCMC 主因，选择相位曲率解释性诊断 |
| 41 | RDA 方位相位曲率诊断契约 | complete | 冻结采样间距、相位曲率、Nyquist 裕量与 replay |
| 42 | RDA 方位相位曲率诊断实现 | complete_current_platform | diagnostics、Session、replay 和双环境回归通过 |
| 43 | RDA 诊断后续决策门 | complete | 每脉冲曲率不足以形成警告阈值，选择孔径二次相位跨度诊断 |
| 44 | RDA 孔径二次相位跨度诊断契约 | complete | 冻结新解释性诊断，不增加警告、拒绝或 Auto |
| 45 | RDA 孔径二次相位跨度诊断实现 | complete_current_platform | RdaDiagnostics、Session、replay 和双环境回归通过 |
| 46 | RDA 目标方位偏置误差决策门 | complete | 不批准单一偏置诊断、警告、拒绝或 Auto |
| 47 | Phase 2 参考级成像闭环综合再审批 | complete | 固定 PRF 点目标闭环通过；选择 SNR 鲁棒性矩阵 |
| 48 | 确定性噪声与 SNR 鲁棒性矩阵契约 | complete | 冻结测试侧噪声定义、矩阵和审批边界 |
| 49 | 确定性噪声与 SNR 鲁棒性矩阵实现 | complete_current_platform | helper、M1/M4 双 seed 矩阵和双环境回归通过 |
| 50 | SNR 矩阵后续决策门 | complete | 统一窗口口径；选择确定性分布式杂波参考模型 |
| 51 | 确定性分布式杂波参考模型契约 | complete | 冻结测试侧杂波定义与审批边界 |
| 52 | 确定性分布式杂波参考模型实现 | complete_current_platform | helper、M1/M4 双密度双 seed 矩阵通过 |
| 53 | 分布式杂波后续决策门 | complete | 选择确定性噪声与杂波 SNR/SCR 二维矩阵契约 |
| 54 | 确定性噪声与杂波 SNR/SCR 二维矩阵契约 | complete | 冻结联合输入、矩阵和审批边界 |
| 55 | 确定性噪声与杂波 SNR/SCR 二维矩阵实现 | complete_current_platform | 联合 helper、首批矩阵与验收报告通过 |
| 56 | 联合 SNR/SCR 矩阵后续决策门 | complete | 关闭矩阵扩展线；选择辐射定标闭环契约 |
| 57 | 辐射定标闭环工程契约 | complete | 冻结内部定标、RCS 反演与显式权重融合口径 |
| 58 | 辐射定标内部闭环实现 | complete_current_platform | 内部标量模块、聚焦闭环与双环境验收通过 |
| 59 | 辐射定标后续接入决策门 | complete | public 接入后置；选择显式标定观测契约 |
| 60 | 显式标定观测工程契约 | complete | 冻结显式身份、像素功率、斜距与生命周期 |
| 61 | 显式标定观测内部实现 | complete_current_platform | 观测构建、原子转换和双环境验收通过 |
| 62 | 显式标定观测后续决策门 | pending | 审计 Session 内部接入与像素功率扩展 |

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

## 阶段 12：L2 工程契约冻结

状态：`complete`

已批准：

- 用户以“继续”批准进入 L2 轨迹误差与一阶运动补偿方向。
- 新增 `SAR_L2_MOTION_COMPENSATION_CONTRACT.md`。
- L2 冻结为固定种子零均值速度扰动，经时间积分形成连续位置轨迹。
- 一阶运动补偿冻结为相对显式参考点的包络平移与载频相位校正。
- public Session 默认 L1、Auto 门禁和现有尺寸上限保持不变。

## 阶段 13：L2 连续扰动轨迹

状态：`complete_current_platform`

任务：

1. 新增 L2 轨迹配置和轨迹误差诊断。
2. 实现固定种子速度扰动和连续位置积分。
3. 验证零扰动严格退化为 L1。
4. 验证相同种子逐点一致、不同种子产生不同轨迹。
5. 验证非零扰动轨迹位置连续且误差非零。

已完成：

- `PlatformPulseState` 扩展三轴速度。
- 新增 `PerturbedStripmapTrackConfig` 和 `TrajectoryErrorDiagnostics`。
- 固定种子三轴高斯速度扰动经逐脉冲积分生成连续位置轨迹。
- 全零扰动显式返回 L1 轨迹和零诊断，严格满足退化契约。
- 相同种子逐点一致，不同种子产生不同轨迹。

## 阶段 14：一阶运动补偿闭环

状态：`complete_current_platform`

任务：

1. 实现相对参考点的逐脉冲斜距误差诊断。
2. 实现 linear 包络平移。
3. 实现载频相位校正。
4. 构建理想、L2 未补偿和 L2 已补偿同场景比较。
5. 证明补偿后 NRMS 降低、相关系数提高。
6. 形成独立审批报告。

已完成：

- 新增内部 `ApplyFirstOrderMotionCompensation()`。
- 使用实际/理想轨迹相对显式参考点的斜距差执行 linear 包络平移和载频相位校正。
- 诊断覆盖最大/RMS 斜距误差、最大包络平移、补偿脉冲数和越界样本数。
- 零轨迹误差输入保持 raw history 逐样本一致。
- 确定性 L2 场景：
  - 未补偿 NRMS `1.312794`，相关系数 `0.138285`。
  - 补偿后 NRMS `0.249779`，相关系数 `0.968805`。
- `1024x1024` raw history 一阶补偿 Debug 核心处理约 `0.052741 s`。
- 新增 `docs/sar_l2_motion_compensation_acceptance_report.md`。

## 阶段 15：L2 后续扩展决策门

状态：`complete`

候选方向：

1. 将 L2 与一阶运动补偿受控接入 public Session。
2. 扩展多参考点/空间变化一阶补偿。
3. 转入 L3 轨迹与二阶残余误差校正。

当前建议：

- 优先受控接入 public Session，但必须新增公开配置、replay 契约、结构化诊断和默认关闭门。
- Auto 继续后置。

决策结论：

- 跟随任务执行，选择受控接入 public Session。
- 多参考点补偿与 L3 继续后置。

## 阶段 16：L2 Session 接入契约

状态：`complete`

- 新增 `SAR_L2_SESSION_INTEGRATION_CONTRACT.md`。
- public policy 新增默认关闭的 `enable_l2_motion_compensation`。
- mission 配置提供三轴速度扰动标准差和固定种子。
- 参考点固定为 local `(0, nominal_slant_range_m, 0)`。
- 启用 L2 时必须在 RDA 前强制执行一阶补偿，不允许 public L2 未补偿成像。
- L2 配置进入 session config replay，但不进入 runtime patch。

## 阶段 17：L2 公开配置与 replay

状态：`complete_current_platform`

任务：

1. 新增公开 mission/policy 配置字段，默认关闭。
2. 更新 FlatBuffers session config schema 和生成头。
3. 更新 codec 与 round-trip 测试。
4. 验证旧默认行为和 public API contract。

已完成：

- policy 新增默认关闭的 `enable_l2_motion_compensation`。
- mission 新增三轴速度扰动标准差和固定 seed。
- FlatBuffers session config schema 在尾部追加字段，旧 payload 缺失字段解码为关闭/零值。
- codec、round-trip、public smoke、replay 和 SAR CI 通过。

## 阶段 18：L2 Session 执行闭环

状态：`complete_current_platform`

任务：

1. Session 同时维护当前 aperture 的理想与实际轨迹。
2. 使用实际 L2 轨迹生成 raw echo。
3. 在 RDA 前强制执行一阶运动补偿。
4. 输出结构化 L2 轨迹和补偿诊断。
5. 验证默认 L1、L2 零扰动、L2 非零扰动和 replay。

已完成：

- Session 同步维护 latest-N 理想轨迹、实际轨迹和 raw pulse history。
- 使用实际 L2 轨迹生成 raw echo，并在 RDA 前强制执行一阶运动补偿。
- 零扰动严格退化为 L1 输出摘要；非零扰动与跨周期 aperture 对齐通过。
- L2 session config replay、结构化诊断和非法配置拒绝通过。
- 新增 `docs/sar_l2_session_integration_acceptance_report.md`。

## 阶段 19：L2 后续扩展决策门

状态：`complete`

候选方向：

1. 多参考点/空间变化一阶运动补偿。
2. L3 轨迹与二阶残余误差校正。
3. 外部逐脉冲轨迹 public 输入与 replay 契约。

当前建议：

- 选择 L3 航路点轨迹，先建设内部确定性几何与退化证据。
- 外部逐脉冲轨迹属于当前方案明确排除的 L4 范围，继续后置。
- 多参考点补偿缺少空间参考面与独立验收真值，继续后置。
- 二阶补偿必须等待 L3 成像退化和独立参考被量化后另行审批。

决策结论：

- 跟随任务执行，选择 L3 航路点轨迹方向。
- 首批只实现几何层与显式脉冲时刻，不接入 public Session。

## 阶段 20：L3 航路点轨迹契约

状态：`complete`

- 新增 `SAR_L3_WAYPOINT_TRAJECTORY_CONTRACT.md`。
- L3 冻结为显式时间航路点之间的分段线性轨迹。
- 显式脉冲时刻支持固定和时变 PRF；本阶段不实现调度器。
- 直线航路点与固定 PRF 必须严格退化为 L1。
- public Session、L3 成像、二阶补偿、自聚焦和 Auto 继续后置。

## 阶段 21：L3 航路点轨迹几何

状态：`complete_current_platform`

任务：

1. 新增内部 L3 航路点和轨迹生成配置。
2. 实现显式脉冲时刻上的分段线性位置和航段速度。
3. 验证固定 PRF 直线轨迹严格退化为 L1。
4. 验证转角命中、非均匀脉冲时刻和非法输入拒绝。
5. 完成默认与 Eigen 3.3.9/C++11 回归。

已完成：

- 新增内部 `Waypoint`、`WaypointTrackConfig` 和 `GenerateWaypointTrack()`。
- 显式脉冲时刻支持固定与非均匀采样，位置按航段线性插值。
- 固定 PRF 直线航路点与 L1 逐点一致。
- 转角航段速度切换、重复生成和非法时间契约拒绝通过。
- 新增 `docs/sar_l3_waypoint_trajectory_acceptance_report.md`。

## 阶段 22：L3 raw echo 与成像退化基线

状态：`complete_current_platform`

任务：

1. 使用相同点目标与 L1/L3 轨迹分别生成 raw echo。
2. 将 L3 raw echo 输入现有 L1-RDA，量化非直线轨迹退化。
3. 使用支持任意逐脉冲位置的 GBP 建立 L3 参考聚焦结果。
4. 对比 L1-RDA、L3 未适配 RDA 和 L3-GBP 的峰值位置、NRMS 与相关性。
5. 不接入 public Session，不实现二阶补偿或 Auto。

已完成：

- 在已审批 `9x9` 参考场景中生成折线 L3 raw echo。
- L1-RDA 相对 L1-GBP：NRMS `0.042218`，相关系数 `0.999109`。
- L3 raw echo 经 L1-RDA 相对 L3-GBP：NRMS `0.501878`，相关系数 `0.874059`。
- L3-GBP 仍将目标聚焦到预期邻域。
- 新增 `docs/sar_l3_imaging_degradation_baseline_report.md`。

## 阶段 23：L3 后续处理决策门

状态：`complete`

候选方向：

1. 审计现有单参考点一阶运动补偿对 L3 的适用性。
2. 实现 L3 专用 BP。
3. 直接进入二阶残余误差补偿。

决策结论：

- 先审计现有一阶补偿；它是判断 BP/二阶补偿必要性的最低风险证据门。
- 未证明一阶补偿不足前，不批准新增 BP 或二阶补偿。

## 阶段 24：L3 一阶补偿适用性审计

状态：`complete_current_platform`

任务：

1. 对 L3 raw echo 应用现有单参考点一阶运动补偿。
2. 分别与 L1 理想 RDA 和 L3-GBP 比较补偿前后质量。
3. 增加至少一个偏离参考点的目标，检查空间变化残余误差。
4. 根据证据决定继续使用一阶补偿、转入多参考点/BP，或批准二阶补偿研究。

已完成：

- 相对 L3-GBP，未补偿 NRMS `0.501878`、相关系数 `0.874059`。
- 一阶补偿后 NRMS `0.185881`、相关系数 `0.982724`。
- 偏离参考点目标最大空间变化残余斜距误差约 `0.000180 m`。
- 当前证据支持内部小场景继续使用一阶补偿，不支持批准二阶补偿或 BP。
- 新增 `docs/sar_l3_first_order_compensation_audit.md`。

## 阶段 25：L3 后续算法决策门

状态：`complete`

决策结论：

- 不批准二阶补偿、多参考点或 BP；当前没有明确失效证据支撑复杂度。
- 下一步建立 L3 一阶补偿适用边界矩阵，扫描转弯幅度与目标相对参考点偏移。
- public Session、时变 PRF 成像和 Auto 继续后置。

## 阶段 26：L3 一阶补偿适用边界矩阵

状态：`complete_current_platform`

任务：

1. 扫描多档孔径末端横向偏移。
2. 扫描目标相对补偿参考点的距离偏移。
3. 记录补偿前后 NRMS、相关性和最大空间残余斜距误差。
4. 冻结当前小场景一阶补偿通过区与明确失效区。
5. 依据失效区证据决定是否进入多参考点、BP 或二阶补偿。

已完成：

- 横向偏移 `0/1/3/6 m` 当前门通过；`12 m` 时 NRMS `0.386100`、相关系数 `0.925463`，明确失效。
- `12 m` 转弯下，目标距离单元从 `20` 偏移到 `12` 时，最大空间残余斜距误差从 `0` 增至 `0.007119 m`。
- 质量随转弯幅度不严格单调，禁止以单一几何阈值替代质量验收。
- 新增 `docs/sar_l3_first_order_applicability_matrix_report.md`。

## 阶段 27：L3 失效区后续决策门

状态：`complete`

决策结论：

- 选择 L3 专用 BP 路径，二阶补偿与多参考点继续后置。
- GBP 与 BP 数学聚焦结果相同，主要区别为像素优先与脉冲优先遍历。
- BP 必须复用共享后向投影核心，禁止复制一套独立数学实现。
- 相同输入下 BP 与 GBP 必须逐像素一致，L3 失效区中 BP 必须优于一阶补偿 RDA。

## 阶段 28：L3 BP 工程契约

状态：`complete`

任务：

1. 冻结 BP 与 GBP 的共享距离压缩、插值、传播相位和网格契约。
2. 冻结 BP 按脉冲累加到像素、GBP 按像素遍历脉冲的实现边界。
3. 冻结逐像素一致性、L3 失效区质量和独立性能验收门。
4. public Session、Auto 和尺寸扩展继续后置。

已完成：

- 新增 `SAR_L3_BP_CONTRACT.md`。
- GBP/BP 共享距离压缩、插值、相位、网格和尺寸门。
- 唯一区别冻结为像素优先与脉冲优先遍历顺序。
- 相同输入复图必须逐样本一致。

## 阶段 29：L3 BP 内部闭环

状态：`complete_current_platform`

任务：

1. 提取 GBP/BP 共享后向投影核心。
2. 新增脉冲优先 BP 内部入口。
3. 验证 L1/L3 场景 GBP/BP 逐样本一致。
4. 验证 BP 在 `12 m` L3 失效区优于一阶补偿 RDA。
5. 增加 `128x128` BP 独立性能门并完成双环境回归。

已完成：

- GBP/BP 共用后向投影核心，仅遍历顺序不同。
- L1/L3 相同输入 BP 与 GBP 复图逐样本一致。
- `12 m` L3 失效区中 BP 相对 GBP 为零误差，优于一阶补偿 RDA。
- `128x128` BP Debug 约 `0.177757 s`，通过独立性能门。
- 新增 `docs/sar_l3_bp_acceptance_report.md`。

## 阶段 30：L3 BP 后续接入决策门

状态：`complete`

决策结论：

- 下一步冻结 L3 航路点与 BP 的受控 public Session 接入契约。
- public L3 必须默认关闭、显式选择 BP、进入 session config replay，并严格受 `128x128` 门禁。
- Auto 继续后置；当前不允许根据轨迹自动切换算法。
- 时变 PRF 调度、二阶补偿、自聚焦继续后置。

## 阶段 31：L3 BP Session 接入契约

状态：`complete`

任务：

1. 冻结 public waypoint 配置与 session 起始时间基准。
2. 冻结固定 PRF 脉冲时刻生成与航路点覆盖拒绝行为。
3. 冻结显式 BP policy、L2/L3 互斥、`128x128` 尺寸门和默认关闭行为。
4. 冻结 waypoint/session config replay；runtime patch 继续禁止。
5. 冻结输出摘要和结构化诊断，不开放全图 replay。

已完成：

- 新增 `SAR_L3_BP_SESSION_INTEGRATION_CONTRACT.md`。
- public waypoint 使用相对 Session 起点时间与 LLA，内部转换为 local Cartesian。
- public L3 固定 PRF，显式选择 BP，与 L1-RDA/L2 互斥。
- 冻结独立 `128x128` 尺寸门、输出阶段、诊断和 replay 边界。

## 阶段 32：L3 BP 公开配置与 replay

状态：`complete_current_platform`

任务：

1. 新增 public waypoint、L3 BP policy 和输出摘要字段。
2. 更新 session/cycle FlatBuffers schema 和生成头。
3. 更新 codec、round-trip 和 public smoke。
4. 验证旧 payload/default 行为保持 L3/BP 关闭。

已完成：

- public mission 新增 `SarWaypointConfigList l3_waypoints`。
- public policy 新增默认关闭的 `enable_l3_bp_imaging`。
- output 新增 `kL3BpImage` 和 `has_l3_bp_image`。
- FlatBuffers schema、生成头、codec、round-trip 和 public smoke 通过。
- waypoint 与 BP policy 未进入 runtime patch。

## 阶段 33：L3 BP Session 执行闭环

状态：`complete_current_platform`

任务：

1. Session 按固定 PRF 和 waypoint 覆盖生成 L3 脉冲轨迹。
2. 使用 L3 轨迹生成 raw echo 并执行 BP。
3. 实现互斥、覆盖范围和 `128x128` 结构化拒绝。
4. 输出 L3/BP diagnostics，并完成摘要级 replay。

验收结果：

- 默认与 Conan Eigen 3.3.9 全部 `Sar*` 单测：各 73/73 passed。
- 默认与 Conan Eigen 3.3.9 SAR replay-fast：各 10/10 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。
- 验收报告：`docs/sar_l3_bp_session_integration_acceptance_report.md`。

## 阶段 34：Phase 2 完成度审计

状态：`complete`

任务：

1. 按“参考级成像与算法对比闭环”目标逐项审计现有证据。
2. 核对 RDA/GBP/BP、L2/L3 适用边界、public Session 与 replay 的完成度和剩余缺口。
3. 保持 Auto 后置，基于缺口证据选择下一扩展方向。

审计结论：

- 当前固定 PRF、小场景、点目标范围内的参考级成像与算法对比闭环已完成。
- RDA、GBP、BP、L2 一阶补偿、L3 适用边界和 public L3 BP 均有独立证据。
- 方位偏置、二维多目标、边界目标和参数扫描证据不足，不能批准通用 Auto。
- 下一方向选择参考场景矩阵扩展。
- 新增 `docs/sar_phase2_reference_closure_audit.md`。

## 阶段 35：参考场景矩阵扩展契约

状态：`complete`

已完成：

- 新增 `SAR_REFERENCE_SCENARIO_MATRIX_CONTRACT.md`。
- 冻结 M1-M7 首批场景，覆盖 L1/L2/L3、中心/偏置/二维多目标与通过区/失效区。
- 冻结统一 raw、轨迹、网格、质量指标和跨算法比较口径。
- Auto、public API、尺寸扩展、时变 PRF 和全图 replay 继续后置。

## 阶段 36：参考场景矩阵实现

状态：`complete_current_platform`

任务：

1. 提取可复用的矩阵场景描述与执行 helper。
2. 实现 M1-M7 确定性质量矩阵测试。
3. 记录质量结果与暴露的适用边界。
4. 完成双环境回归、审批报告和下一决策门。

验收结果：

- 新增二维局部坐标确定性目标 helper 与独立 `SarReferenceScenarioMatrixTest`。
- M1-M4 L1 RDA/GBP NRMS 均 `<0.1`、相关系数均 `>0.99`，BP/GBP 逐样本一致。
- M5 L2 二维目标补偿后 NRMS `0.239074`、相关系数 `0.971422`。
- M6 `3 m` L3 二维目标当前门通过；M7 `12 m` 补偿后仍明确失效。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*` 单测各 80/80 passed。
- 验收报告：`docs/sar_reference_scenario_matrix_acceptance_report.md`。

## 阶段 37：参考矩阵后续决策门

状态：`complete`

候选方向：

1. 图像边界与采样/硬件参数适用性矩阵。
2. 时变 PRF 与更真实 L3 轨迹。
3. 多参考点或二阶补偿。
4. 重新审计 Auto。

决策结论：

- 选择图像边界与采样/硬件参数适用性矩阵。
- M1-M7 已补齐二维布局，但仍只覆盖中心附近小窗口与单组硬件参数。
- 时变 PRF、二阶补偿、多参考点和 Auto 继续后置。

## 阶段 38：边界与参数矩阵契约

状态：`complete`

已完成：

- 新增 `SAR_REFERENCE_BOUNDARY_PARAMETER_MATRIX_CONTRACT.md`。
- 冻结 `interior_pass / boundary_degraded / echo_clipped / invalid` 分类。
- 冻结 B1-B4 距离/方位边界矩阵与 P1-P4 单参数扫描。
- 冻结通过档位沿用既有 L1 门，失败档位保留真实边界。

## 阶段 39：边界与参数矩阵实现

状态：`complete_current_platform`

任务：

1. 增加 raw echo 裁剪可审计矩阵 helper。
2. 实现 B1-B4 边界分类测试。
3. 实现 P1-P4 单参数扫描并记录适用范围。
4. 完成双环境回归、审批报告和下一决策门。

验收结果：

- 新增 raw history 裁剪脉冲/目标/样本汇总。
- B1-B4 明确区分内部通过、图像边缘退化和 raw echo 裁剪。
- P1 `80-120 MHz` 与 P2 `0.8-1.2 GHz` 首批档位通过。
- P3/P4 证明退化与方位采样间距 `platform_velocity / PRF` 耦合：
  - `0.05/0.1 m/pulse` 当前门通过。
  - `0.2 m/pulse` NRMS `0.177589`、相关系数 `0.984231`，当前门失败。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*` 单测各 86/86 passed。
- 验收报告：`docs/sar_boundary_parameter_matrix_acceptance_report.md`。

## 阶段 40：方位采样充分性决策门

状态：`complete`

任务：

1. 分离 `v/PRF`、载频、斜距、孔径和多普勒采样的工程关系。
2. 判断当前 `0.2 m/pulse` 失败属于预期采样边界、RDA 实现边界或参考网格比较边界。
3. 选择结构化诊断、算法修复或更宽矩阵，不直接冻结经验硬阈值。

审计结论：

- `0.175/0.2 m/pulse` 质量失败时几何 Doppler Nyquist 裕量仍为 `18.35x/14.05x`，不是混叠失败。
- none/linear/sinc RCMC 在粗间距下均失败，RCMC 不是主因。
- NRMS 随每脉冲二阶方位相位曲率
  `4*pi*(v/PRF)^2/(lambda*R_ref)` 变化；等曲率参数对得到近似相同 NRMS。
- 选择增加解释性 diagnostics，不批准结构化拒绝阈值或算法修复。
- 新增 `docs/sar_azimuth_sampling_audit.md`。

## 阶段 41：RDA 方位相位曲率诊断契约

状态：`complete`

已完成：

- 新增 `SAR_RDA_AZIMUTH_PHASE_CURVATURE_DIAGNOSTIC_CONTRACT.md`。
- 冻结方位采样间距、每脉冲二阶相位曲率、最大几何多普勒与 Nyquist 裕量公式。
- 冻结内部 RDA diagnostics 与 Session `sar.rda_peak` 摘要级 replay。
- 不增加警告、拒绝、Auto 或 public 配置。

## 阶段 42：RDA 方位相位曲率诊断实现

状态：`complete_current_platform`

已完成：

1. 扩展 `RdaDiagnostics`，实现采样间距、相位曲率、最大几何 Doppler 和 Nyquist 裕量。
2. 提取独立内部采样诊断计算，保留单脉冲无穷裕量定义但不扩大 RDA FFT 输入范围。
3. 扩展 RDA 单测和耦合参数矩阵，直接核对生产 diagnostics。
4. 在 Session `sar.rda_peak` 中记录新指标并通过 replay 严格比较。
5. 新增 `docs/sar_rda_phase_curvature_diagnostics_acceptance_report.md`。
6. 默认与 Conan Eigen 3.3.9 全部 `Sar*` 各 92/92 通过，审批门通过。

## 阶段 43：RDA 诊断后续决策门

状态：`complete`

已完成：

1. 扩展 `5/9/17/33` aperture 脉冲数、`0.1/0.2 m/pulse` 和中心/偏置目标矩阵。
2. 证明相同每脉冲曲率下，质量随 aperture 长度显著变化。
3. 证明孔径二次相位跨度可归并中心目标等效组合，但目标偏置仍是独立影响量。
4. 决定不批准质量风险警告阈值、结构化拒绝或 Auto。
5. 新增 `docs/sar_rda_diagnostic_followup_decision.md`。
6. 默认与 Conan Eigen 3.3.9 全部 `Sar*` 各 93/93 通过，审批门通过。

## 阶段 44：RDA 孔径二次相位跨度诊断契约

状态：`complete`

已完成：

1. 新增 `SAR_RDA_APERTURE_PHASE_SPAN_DIAGNOSTIC_CONTRACT.md`。
2. 冻结 `azimuth_quadratic_phase_span_rad` 定义、输出和 replay 语义。
3. 明确该指标只解释 aperture 中心参考点二次相位跨度，不覆盖目标布局误差。
4. 继续禁止质量警告、结构化拒绝和 Auto。

## 阶段 45：RDA 孔径二次相位跨度诊断实现

状态：`complete_current_platform`

已完成：

1. 扩展 `RdaDiagnostics` 与采样诊断计算。
2. 扩展 Session `sar.rda_peak` 和 replay 验证。
3. 使用阶段 43 等跨度组合验证生产 diagnostics。
4. 新增 `docs/sar_rda_aperture_phase_span_acceptance_report.md`。
5. 默认与 Conan Eigen 3.3.9 全部 `Sar*` 各 93/93 通过，审批门通过。

## 阶段 46：RDA 目标方位偏置误差决策门

状态：`complete`

已完成：

1. 通过等物理孔径和等归一化偏置矩阵分离目标偏置、aperture 长度和采样间距影响。
2. 证明目标偏置影响对方向对称并随幅值增加。
3. 证明目标偏置非线性相位残差不足以单独解释额外误差。
4. 不批准生产 diagnostics、质量警告、结构化拒绝或 Auto。
5. 新增 `docs/sar_rda_target_azimuth_offset_decision.md`。
6. 默认与 Conan Eigen 3.3.9 全部 `Sar*` 各 95/95 通过，审批门通过。

## 阶段 47：Phase 2 参考级成像闭环综合再审批

状态：`complete`

已完成：

1. 汇总 RDA/GBP/BP、L2/L3、边界、参数与 RDA 解释性诊断证据。
2. 区分已审批能力、明确适用边界与仍缺失的证据。
3. 继续拒绝 Auto、尺寸扩展、时变 PRF、真实动力学和全图 replay。
4. 下一扩展方向选择测试侧确定性噪声与 SNR 鲁棒性参考矩阵。
5. 新增 `docs/sar_phase2_reference_imaging_reapproval_report.md`。

## 阶段 48：确定性噪声与 SNR 鲁棒性矩阵契约

状态：`complete`

已完成：

1. 冻结固定 seed 复高斯噪声定义和 SNR 计算口径。
2. 冻结首批 L1 RDA/GBP/BP 参考场景与质量指标矩阵。
3. 明确只修改测试支持层，不增加 public 配置、replay 或生产算法分支。
4. 禁止由首批 SNR 矩阵直接生成 Auto、警告或结构化拒绝。
5. 新增 `SAR_REFERENCE_SNR_MATRIX_CONTRACT.md`。

## 阶段 49：确定性噪声与 SNR 鲁棒性矩阵实现

状态：`complete_current_platform`

已完成：

1. 实现测试支持层固定 seed 复高斯噪声 helper 与 diagnostics。
2. 实现 M1/M4 双 seed、多 SNR 参考矩阵。
3. 验证 clean/noisy 算法比较、BP/GBP 一致性和趋势。
4. 完成双环境审批门与审批报告。
5. 新增 `docs/sar_reference_snr_matrix_acceptance_report.md`。
6. 默认与 Conan Eigen 3.3.9 全部 `Sar*` 各 97/97 通过，审批门通过。

## 阶段 50：SNR 矩阵后续决策门

状态：`complete`

已完成：

1. 统一 RDA/GBP clean/noisy 比较的输出支持范围。
2. 判断是否需要更多 seed、SNR 档位或场景。
3. 下一步选择测试侧确定性分布式杂波参考模型。
4. 不直接批准通用 SNR 阈值、警告、拒绝或 Auto。
5. 新增 `docs/sar_reference_snr_followup_decision.md`。

## 阶段 51：确定性分布式杂波参考模型契约

状态：`complete`

已完成：

1. 新增 `SAR_DETERMINISTIC_DISTRIBUTED_CLUTTER_CONTRACT.md`。
2. 冻结规则网格散射点、固定 seed、显式 PRNG、复幅相和遍历顺序定义。
3. 冻结基于 target/clutter raw-history 总能量的 requested/realized SCR 定义。
4. 冻结 M1/M4、双网格密度、双 seed 与无杂波/30/20/10/0 dB 首批矩阵。
5. 明确只修改测试支持层，不增加生产杂波、公用配置、绝对功率或辐射定标语义。
6. 禁止直接生成质量阈值、警告、拒绝或 Auto。

## 阶段 52：确定性分布式杂波参考模型实现

状态：`complete_current_platform`

已完成：

1. 在 `tests/support` 实现确定性散射点网格和杂波 raw-history helper。
2. 增加确定性、SCR 能量缩放、无裁剪和算法共享输入测试。
3. 实现 M1/M4、`3x3/5x5`、双 seed 与 `30/20/10/0 dB` 首批矩阵。
4. 验证 BP/GBP 在混合 raw history 下继续逐样本一致。
5. 新增 `docs/sar_deterministic_distributed_clutter_acceptance_report.md`。

## 阶段 53：分布式杂波后续决策门

状态：`complete`

已完成：

1. 复核规则网格、双密度、双 seed 与首批 SCR 趋势证据。
2. 判断当前没有足够证据冻结随机位置、空间相关函数或相关长度。
3. 选择确定性噪声与分布式杂波 SNR/SCR 二维矩阵作为下一阶段。
4. 继续禁止直接批准生产杂波、通用 SCR 阈值、质量警告或 Auto。
5. 新增 `docs/sar_distributed_clutter_followup_decision.md`。

## 阶段 54：确定性噪声与杂波 SNR/SCR 二维矩阵契约

状态：`complete`

已完成：

1. 新增 `SAR_REFERENCE_SNR_SCR_MATRIX_CONTRACT.md`。
2. 冻结噪声与杂波独立 seed、以纯目标能量为共同参考的独立缩放和加法顺序无关。
3. 冻结 M1 `3x3` 二维矩阵与 M4 哨兵矩阵。
4. 冻结联合 diagnostics、算法共享输入和 BP/GBP 一致性要求。
5. 明确只修改测试支持层，不增加生产/public/replay 能力。
6. 禁止直接生成质量阈值、警告、拒绝或 Auto。

## 阶段 55：确定性噪声与杂波 SNR/SCR 二维矩阵实现

状态：`complete_current_platform`

已完成：

1. 在 `tests/support` 实现独立目标、噪声、杂波分量及联合输入 helper。
2. 增加独立 seed、能量缩放、注入顺序无关和算法共享输入测试。
3. 实现 M1 完整二维矩阵与 M4 哨兵矩阵。
4. 新增 `docs/sar_reference_snr_scr_matrix_acceptance_report.md`。
5. 默认环境完整 CTest `25/25`、参考矩阵 `13/13` 通过。
6. Conan Eigen 3.3.9 联合矩阵 `2/2` 与 `sar_cxx11_compat` 通过。

## 阶段 56：联合 SNR/SCR 矩阵后续决策门

状态：`complete`

决策结论：

1. 首批矩阵已覆盖确定性和总体退化趋势，不继续扩展中间 SNR/SCR 档位。
2. 相关杂波缺少可批准的分布、相关函数和相关长度，继续后置。
3. 当前没有证据冻结联合质量阈值、警告、拒绝或 Auto。
4. 下一方向选择 `1.1.4.4.3.4 辐射定标模块`的测试侧闭环契约。
5. 新增 `docs/sar_joint_snr_scr_followup_decision.md`。

## 阶段 57：辐射定标闭环工程契约

状态：`complete`

已完成：

1. 新增 `SAR_RADIOMETRIC_CALIBRATION_CONTRACT.md`。
2. 冻结图像响应定标因子、RCS 反演和辐射误差公式。
3. 澄清完整雷达方程系统因子与首批内部定标因子的边界。
4. 冻结显式正权重多点融合，禁止零残差倒数权重。
5. 冻结 M1/M4 聚焦链路和受控联合干扰首批验收矩阵。
6. public API、Session、replay、有效性阈值与完整系统因子继续后置。

## 阶段 58：辐射定标内部闭环实现

状态：`complete_current_platform`

已完成：

1. 在 `src/sar/calibration` 实现内部标量定标、融合、反演和误差评估。
2. 增加单点、多点、无效输入和尺度不变性单元测试。
3. 增加 M1/M4 未归一化聚焦图像闭环与联合干扰哨兵测试。
4. 新增 `docs/sar_radiometric_calibration_acceptance_report.md`。
5. 默认完整 CTest `25/25`、双环境定标测试各 `5/5`、Eigen 3.3.9 C++11 门通过。

## 阶段 59：辐射定标后续接入决策门

状态：`complete`

决策结论：

1. public Session 缺少显式标定目标选择和未归一化图像功率提取语义，直接接入后置。
2. 完整系统因子缺少发射功率、系统损耗和处理增益一致口径，继续后置。
3. 当前没有证据冻结 SNR、边缘或残差有效性阈值。
4. 下一方向选择显式标定观测工程契约。
5. 新增 `docs/sar_radiometric_calibration_followup_decision.md`。

## 阶段 60：显式标定观测工程契约

状态：`complete`

已完成：

1. 新增 `SAR_EXPLICIT_CALIBRATION_OBSERVATION_CONTRACT.md`。
2. 冻结显式观测身份、已知 RCS、指定像素、斜距、孔径范围和权重。
3. 冻结未归一化指定像素峰值功率提取，禁止自动全局峰值选择。
4. 冻结不可变值对象和显式重新定标生命周期。
5. public API、Session schema、replay、主瓣积分和有效性阈值继续后置。

## 阶段 61：显式标定观测内部实现

状态：`complete_current_platform`

已完成：

1. 在内部 calibration 模块实现观测构建、验证和样本转换。
2. 增加身份、像素、图像归一化声明、孔径范围和无效列表测试。
3. 增加 M1 GBP/BP 与 M4 显式像素观测闭环。
4. 新增 `docs/sar_explicit_calibration_observation_acceptance_report.md`。
5. 默认完整 CTest `25/25`、双环境定标观测测试各 `6/6`、Eigen 3.3.9 C++11 门通过。

## 阶段 62：显式标定观测后续决策门

状态：`pending`

任务：

1. 审计显式观测、定标核心和 Session 内部聚焦链路的接入条件。
2. 决定 Session 内部受控接入、像素定位或主瓣积分功率的优先级。
3. public API、schema、replay 和有效性阈值继续门禁。

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
| L2 零扰动递推积分与 L1 直接公式产生约 `1e-14 m` 浮点差 | 1 | 全零扰动配置显式返回 L1 轨迹与零诊断，保持严格退化契约 |
| 单脉冲完整 RDA 测试进入不受支持的 FFT 成像链并异常退出 | 1 | 成像入口明确拒绝单脉冲 aperture；独立诊断函数验证无穷 Nyquist 裕量 |
| 审计阶段状态的 shell 命令引号未闭合 | 1 | 改用无嵌套反引号的独立检索命令 |
| zsh 展开未引用的 gtest `*` 过滤器导致测试未启动 | 1 | 引用完整 `--gtest_filter` 参数后双环境通过 |
| shell 检索命令中的反引号被 zsh 当作命令替换 | 1 | 后续检索避免在双引号参数内嵌 Markdown 反引号 |

## 下一步

执行阶段 62：完成显式标定观测后续决策门。
