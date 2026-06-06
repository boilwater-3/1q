# 进度：SAR Phase 1 工程化计划

## 2026-06-04

### 会话目标

用户要求使用 `planning-with-files-zh`，基于当前 SAR 探索成果制定各阶段详细计划。

### 已执行

1. 读取技能文件 `/Users/aurora/.agents/skills/planning-with-files-zh/SKILL.md`。
2. 读取现有 `task_plan.md`、`findings.md`、`progress.md`。
3. 确认旧规划文件属于 flight_dynamic/JSBSim 阶段 13，不适用于当前 SAR 任务。
4. 运行 `session-catchup.py`，无输出。
5. 查看 `git diff --stat`，无已跟踪文件差异。
6. 查看 `git status --short`，当前 SAR 文档均为未跟踪文件。
7. 将根目录规划文件切换为 SAR Phase 1 工程化规划。

### 创建/修改文件

已改写：

- `task_plan.md`
- `findings.md`
- `progress.md`

当前 SAR 文档：

- `sar_construction_scheme_complete.md`
- `SAR_MODULE_DESIGN.md`
- `SAR_PHASE1_ENGINEERING_CONTRACT.md`

### 当前计划状态

| 阶段 | 状态 |
|---|---|
| 阶段 0：规划与契约冻结 | complete |
| 阶段 1：公共 API 与构建骨架 | pending |
| 阶段 2：FFT 后端与数值基础 | pending |
| 阶段 3：信号链路 | pending |
| 阶段 4：几何、原始回波与缓冲区 | pending |
| 阶段 5：RDA 聚焦实现 | pending |
| 阶段 6：Session/Trace/Replay 集成 | pending |
| 阶段 7：CI 验收与审批包 | pending |
| 阶段 8：Phase 2 决策门 | pending |

### 关键发现

- Phase 1 只能批准 RDA 最小闭环。
- FFT 后端是正式实现前置门。
- 现有 `common/rcs` 不支撑 SAR 辐射定标闭环。
- 机动/侦察/融合不属于 SAR Phase 1。

### 错误记录

无工具错误。

### 下一步建议

从阶段 1 开始落 SAR 公共 API 与 CMake 骨架；同时在阶段 2 前置研究 FFT 后端。阶段 2 未完成前，不承诺大尺寸 RDA 性能。

## 2026-06-04 后续执行

### 会话目标

用户要求新建工作树后切换到新工作树，从阶段 1 开始落 SAR 公共 API 与 CMake 骨架，同时并行研究阶段 2 FFT 后端。

### 已执行

1. 新建工作树 `/Users/aurora/Code/1q-sar-phase1`，分支 `codex/sar-phase1`。
2. 将 SAR 方案文档和规划文件复制到新工作树。
3. 新增 SAR 公共配置头、会话输入输出头、会话门面、工厂、trace/replay 门面。
4. 新增 `src/sar/CMakeLists.txt` 和最小 `SarSession`/trace/replay 实现。
5. 将 SAR core object target 接入 `src/CMakeLists.txt`。
6. 将 SAR 公开头加入 `tests/contract/check_public_api_boundary.cmake` 白名单。
7. 将 SAR include 与最小用法加入 `tests/contract/public_headers_smoke_test.cpp`。
8. 核查 FFT 依赖现状，并新增 `docs/sar_fft_backend_research.md`。

### 当前计划状态

| 阶段 | 状态 |
|---|---|
| 阶段 0：规划与契约冻结 | complete |
| 阶段 1：公共 API 与构建骨架 | complete |
| 阶段 2：FFT 后端与数值基础 | current_platform_complete_cross_platform_pending |
| 阶段 3：信号链路 | complete_current_platform |
| 阶段 4：几何、原始回波与缓冲区 | complete_current_platform |
| 阶段 5：RDA 聚焦实现 | complete_current_platform |
| 阶段 6：Session/Trace/Replay 集成 | pending |
| 阶段 7：CI 验收与审批包 | pending |
| 阶段 8：Phase 2 决策门 | pending |

### 已验证

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_contract_tests`
- `build/llvm-ninja-debug-local/bin/1q_contract_tests`，结果：72/72 passed。
- `cmake -DSOURCE_DIR=/Users/aurora/Code/1q-sar-phase1 -P tests/contract/check_public_api_boundary.cmake`

### 当前剩余门禁

- 阶段 2 已在 macOS AppleClang + Conan Eigen 3.4.0 下实现并验证内部 FFT facade。
- Windows/VS2015 + Eigen 3.3.9 尚未验证。
- 1024x1024 级别 RDA 性能基准尚未建立。

### 阶段 2 已验证

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*'`，结果：7/7 passed。

## 2026-06-04 阶段 3 执行

### 已执行

1. 新增 `src/sar/signal/SarWaveform.h`。
2. 新增 `src/sar/signal/SarWaveform.cpp`。
3. 新增 `tests/unit/sar_signal_chain_test.cpp`。
4. 将 `SarWaveform.cpp` 接入 `SAR_ENGINE_SOURCES`。
5. 实现 LFM 生成、时域匹配滤波器、FFT 线性卷积、距离压缩和基础脉冲质量指标。

### 已验证

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*'`，结果：14/14 passed。

### 当前剩余门禁

- 20dB 宽度估计未实现。
- 阶段 4 已完成当前平台点目标原始回波和 `PulseRingBuffer`。
- SAR Session 仍是阶段 1 骨架，尚未接入真实信号链路输出。

## 2026-06-04 阶段 4 执行

### 已执行

1. 新增 `src/sar/geometry/SarGeometry.h/.cpp`。
2. 新增 `src/sar/echo/SarEcho.h/.cpp`。
3. 新增 `src/sar/runtime/PulseRingBuffer.h/.cpp`。
4. 新增 `tests/unit/sar_echo_geometry_buffer_test.cpp`。
5. 将 geometry/echo/runtime 源文件接入 `SAR_ENGINE_SOURCES`。

### 已验证

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*'`，结果：22/22 passed。

### 当前剩余门禁

- 生产链路尚未生成二维 pulse history 矩阵；当前 RDA 单元测试使用测试夹具生成矩阵。
- 阶段 5 已完成当前平台 L1 broadside 点目标 RDA 最小闭环。
- 尚未把真实信号链路接入 `SarSession`。

## 2026-06-04 阶段 5 执行

### 已执行

1. 新增 `src/sar/imaging/SarRda.h/.cpp`。
2. 新增 `tests/unit/sar_rda_test.cpp`。
3. 将 `SarRda.cpp` 接入 `SAR_ENGINE_SOURCES`。
4. 实现 range compression、azimuth FFT、linear RCMC、azimuth matched filter、azimuth IFFT 和 focused image extraction。
5. 输出 RDA diagnostics。

### 已验证

- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*'`，结果：26/26 passed。

### 当前剩余门禁

- `SarSession` 尚未接入真实信号链路和 RDA 输出。
- trace/replay 仍未实现 SAR schema 和 codec。
- 1024x1024 性能基准尚未建立。

## 2026-06-04 阶段 6 执行

### 已执行

1. 修改 `src/sar/session/SarSession.cpp`，将 `SarSession::StepWithResult()` 接入真实 Phase 1 信号/RDA 管线。
2. 新增 `tests/unit/sar_session_pipeline_test.cpp`。
3. 将 public header smoke test 中的 SAR 最小用法改为小尺寸 RDA 场景，避免公共合约测试触发未批准的大尺寸成像。
4. 增加 Session RDA 运行时尺寸门：`range_sample_count <= 256` 且 `azimuth_pulse_count <= 64`。
5. 增加 RDA 对 raw echo generation 的前置校验，错误码为 `rda_requires_raw_echo`。
6. 记录 RDA peak diagnostics，保持 public API 只暴露摘要和阶段标记，不暴露内部 complex matrix。

### 已验证

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*:SarSessionPipelineTest.*'`，结果：30/30 passed。
- `cmake --build --preset llvm-ninja-debug --target 1q_contract_tests`
- `build/llvm-ninja-debug-local/bin/1q_contract_tests`，结果：72/72 passed。
- `cmake -DSOURCE_DIR=/Users/aurora/Code/1q-sar-phase1 -P tests/contract/check_public_api_boundary.cmake`
- `git diff --check`

### 当前计划状态

| 阶段 | 状态 |
|---|---|
| 阶段 0：规划与契约冻结 | complete |
| 阶段 1：公共 API 与构建骨架 | complete |
| 阶段 2：FFT 后端与数值基础 | current_platform_complete_cross_platform_pending |
| 阶段 3：信号链路 | complete_current_platform |
| 阶段 4：几何、原始回波与缓冲区 | complete_current_platform |
| 阶段 5：RDA 聚焦实现 | complete_current_platform |
| 阶段 6：Session/Trace/Replay 集成 | complete_current_platform |
| 阶段 7：CI 验收与审批包 | pending |
| 阶段 8：Phase 2 决策门 | pending |

### 当前剩余门禁

- `PulseRingBuffer` 已作为跨周期慢时间累积机制接入 Session；仍限当前小场景 Phase 1 路径。
- Windows/VS2015 + Eigen 3.3.9 尚未验证。
- 1024x1024 性能基准尚未建立，Session RDA 尺寸门不能放开。

## 2026-06-05 阶段 6 Trace/Replay 执行

### 已执行

1. 新增 `schemas/replay/sar_replay.fbs` 和 `schemas/replay/sar_session_replay.fbs`。
2. 使用 Conan flatbuffers 包内 `flatc` 生成：
   - `src/sar/session/generated/sar_replay_generated.h`
   - `src/sar/session/generated/sar_session_replay_generated.h`
3. 新增 `src/sar/session/SarReplayFlatbufferCodec.h/.cpp`，覆盖 SAR public cycle input、output frame、cycle result、session config 和 runtime patch。
4. 扩展 `SarTraceSession`，支持 `ReplayTraceWriter`，写入 `session_config`、`cycle_input`、`runtime_config_patch` 和 `cycle_output`。
5. 扩展 `SarReplaySession`，新增 `ReplaySarTrace(trace_dir)`，按 trace 重建 Session、应用 runtime patch、执行 input 并比对 `SarCycleResult` 摘要。
6. 新增 `tests/unit/sar_replay_codec_roundtrip_test.cpp` 和 `tests/unit/sar_replay_session_test.cpp`。
7. 将 SAR replay tests 接入 `1q_replay_fast_tests`，并将 SAR codec 接入 `SAR_CORE_SOURCES`。

### 已验证

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_replay_fast_tests`
- `build/llvm-ninja-debug-local/bin/1q_replay_fast_tests '--gtest_filter=SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`，结果：9/9 passed。
- `build/llvm-ninja-debug-local/bin/1q_replay_fast_tests`，结果：71/71 passed。
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*:SarSessionPipelineTest.*:SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`，结果：39/39 passed。
- `cmake --build --preset llvm-ninja-debug --target 1q_contract_tests`
- `build/llvm-ninja-debug-local/bin/1q_contract_tests`，结果：72/72 passed。
- `cmake -DSOURCE_DIR=/Users/aurora/Code/1q-sar-phase1 -P tests/contract/check_public_api_boundary.cmake`
- `git diff --check`

### 当前计划状态

| 阶段 | 状态 |
|---|---|
| 阶段 0：规划与契约冻结 | complete |
| 阶段 1：公共 API 与构建骨架 | complete |
| 阶段 2：FFT 后端与数值基础 | current_platform_complete_cross_platform_pending |
| 阶段 3：信号链路 | complete_current_platform |
| 阶段 4：几何、原始回波与缓冲区 | complete_current_platform |
| 阶段 5：RDA 聚焦实现 | complete_current_platform |
| 阶段 6：Session/Trace/Replay 集成 | complete_current_platform |
| 阶段 7：CI 验收与审批包 | pending |
| 阶段 8：Phase 2 决策门 | pending |

### 当前剩余门禁

- Trace/replay 当前为 public 摘要级，不保存 focused complex image 全矩阵。
- `PulseRingBuffer` 已作为跨周期慢时间累积机制接入 Session；仍限当前小场景 Phase 1 路径。
- Windows/VS2015 + Eigen 3.3.9 尚未验证。
- 1024x1024 性能基准尚未建立，Session RDA 尺寸门不能放开。

## 2026-06-05 阶段 7 CI 与审批包执行

### 已执行

1. 在 `tests/CMakeLists.txt` 中新增 SAR 专属 CTest 入口：
   - `sar_unit::1q_unit_tests`
   - `sar_replay::1q_replay_fast_tests`
   - `sar_integration::1q_unit_tests`
   - `sar_contract::1q_contract_tests`
2. 为上述入口设置标签：
   - `sar_unit`
   - `sar_replay`
   - `sar_integration`
   - `sar_contract`
   - `sar_ci`
3. 将 `public_api_boundary_guard` 也标记为 `sar_contract;sar_ci`。
4. 新增 `docs/sar_phase1_acceptance_report.md`。

### 当前审批结论

- 当前平台、小场景、点目标 SAR Phase 1 工程基线可进入进一步审批。
- 不批准大尺寸性能、全平台支持、全图 replay、生产图像质量或 Phase 2 算法能力。

### 已验证

- `cmake --preset llvm-ninja-debug`
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`，结果：4/4 passed。
- `build/llvm-ninja-debug-local/bin/1q_replay_fast_tests '--gtest_filter=SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`，结果：9/9 passed。
- `build/llvm-ninja-debug-local/bin/1q_contract_tests --gtest_filter=PublicHeadersSmokeTest.SarPublicSurfaceSupportsMinimalUsage`，结果：1/1 passed。
- `git diff --check`

### 当前剩余门禁

- `sar_performance` 尚未启用。
- Windows/VS2015 + Eigen 3.3.9 尚未验证。
- 1024x1024 性能基准尚未建立。

## 2026-06-05 Session 跨周期 PulseRingBuffer 接入

### 已执行

1. 修改 `src/sar/session/SarSession.cpp`，在 `SarSession::Impl` 内维护：
   - `runtime::PulseRingBuffer raw_pulse_buffer`
   - `next_pulse_id`
   - `pulse_fraction_carry`
2. `SarSession::StepWithResult()` 的 raw echo 阶段改为：
   - 首帧或缓冲不足时补足一个 aperture 窗口。
   - 后续周期按 `dt_sec * PRF + pulse_fraction_carry` 追加新 pulse。
   - 从 `PulseRingBuffer::ReadLatest(azimuth_pulse_count)` 读取连续 latest-N pulse history。
3. 新增 `SarSessionPipelineTest.RawPulseHistoryUsesCrossCycleRingBuffer`。

### 已验证

- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests 1q_replay_fast_tests`
- `build/llvm-ninja-debug-local/bin/1q_unit_tests '--gtest_filter=SarFftBackendTest.*:SarSignalChainTest.*:SarGeometryTest.*:SarEchoTest.*:PulseRingBufferTest.*:SarRdaTest.*:SarSessionPipelineTest.*:SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`，结果：40/40 passed。
- `build/llvm-ninja-debug-local/bin/1q_replay_fast_tests '--gtest_filter=SarReplayCodecRoundtripTest.*:SarReplaySessionTest.*'`，结果：9/9 passed。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`，结果：4/4 passed。

### 当前剩余门禁

- `sar_performance` 尚未启用。
- Windows/VS2015 + Eigen 3.3.9 尚未验证。
- 1024x1024 性能基准尚未建立。

## 2026-06-05 Focused Image Entropy 指标

### 已执行

1. 在 `RdaDiagnostics` 中新增 `image_entropy_nats`。
2. 新增 `EstimateImageEntropyNats()`，按归一化像素功率计算 Shannon entropy：
   - `p_i = |z_i|^2 / sum(|z|^2)`
   - `H = -sum(p_i * ln(p_i))`
3. `FocusStripmapRda()` 在 focused image 生成后计算 entropy。
4. `SarSession` 的 `sar.rda_peak` diagnostics 增加 `image_entropy_nats`。
5. 新增 `SarRdaTest.ImageEntropyUsesNormalizedPowerDistribution`。

### 已验证

- SAR 过滤测试：41/41 passed。
- SAR replay 测试：9/9 passed。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`：4/4 passed。

### 当前剩余门禁

- `sar_performance` 尚未启用。
- azimuth 3dB 宽度指标尚未实现。
- Windows/VS2015 + Eigen 3.3.9 尚未验证。
- 1024x1024 性能基准尚未建立。

## 2026-06-05 Range 20dB 宽度指标

### 已执行

1. 在 `PulseQualityMetrics` 中新增 `width_20db_bins`。
2. `EstimatePulseQuality()` 使用 `peak_magnitude * 0.1` 幅度阈值估计连续 20dB 宽度。
3. 扩展 `SarSignalChainTest.PulseQualityMetricsCapturePeakWidthAndSidelobes` 验证 20dB 宽度。

### 已验证

- SAR 过滤测试：41/41 passed。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`：4/4 passed。
- `git diff --check`。

### 当前剩余门禁

- `sar_performance` 尚未启用。
- azimuth 3dB 宽度指标尚未实现。
- Windows/VS2015 + Eigen 3.3.9 尚未验证。
- 1024x1024 性能基准尚未建立。

## 2026-06-05 二维聚焦输出、方位 3dB 与 FFT 性能烟测

### 已执行

1. 移除将方位 IFFT 结果求和并压入中心行的 `ExtractFocusedAzimuth()` 非物理投影，focused image 现在保留完整二维复数响应。
2. 新增 `EstimateAzimuthWidth3dbBins()`，在全图峰值所在距离列上按连续半功率样本估计方位向 3dB 宽度。
3. 将方位向 3dB 宽度写入 RDA diagnostics 和 Session `sar.rda_peak` 摘要。
4. 新增 `sar_performance` 独立入口，覆盖 1024x1024 二维 FFT facade 和内部 RDA 合成场景当前平台基准。

### 已验证

- `SarRdaTest.*:SarSessionPipelineTest.*`：11/11 passed。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_performance --output-on-failure`：1/1 CTest entry、2/2 internal gtests passed。
- 1024x1024 二维 FFT facade Debug 核心变换当前观测约 344 ms。
- 1024x1024 内部 RDA 合成场景 Debug 核心处理当前观测约 943 ms。

### 当前剩余门禁

- `sar_performance` 不代表 1024x1024 Session 真实点目标端到端性能和峰值内存获批。
- Windows/VS2015 + Conan Eigen 3.3.9 尚未验证。

## 2026-06-05 Conan Eigen 3.3.9 与 1024x1024 Public Session 门

### 已执行

1. Conan 新增项目级 `eigen_version=auto|3.3.9|3.4.0` 选项，bootstrap 安装指纹同步纳入该选项。
2. 新增 `llvm-ninja-debug-eigen339` configure/build/test preset。
3. 新增 1024x1024 真实点目标内部管线和 public `SarSession::StepWithResult()` 性能测试。
4. 当前平台 Session RDA 尺寸门由 256x64 提升到已验证的 1024x1024；默认 2048x1024 仍被拒绝。

### 已验证

- AppleClang + Conan Eigen 3.3.9：SAR FFT/信号链/RDA/Session 25/25 passed，`sar_performance` passed。
- 当前 Conan Eigen 3.4.0：`sar_performance` 1/1 CTest entry、4/4 internal gtests passed。
- 1024x1024 真实点目标内部管线 Debug 当前观测约 1493 ms。
- 1024x1024 public Session Debug 当前观测约 1421-1496 ms。
- 隔离 public Session 最大常驻内存当前观测为 137035776 bytes。

### 当前剩余门禁

- Windows/VS2015 尚未验证。
- 超过 1024x1024 的性能与峰值内存尚未批准。

## 2026-06-05 Phase 1 最终决策冻结

### 用户决策

1. Windows/VS2015 不作为 Phase 1 强制审批门；以 C++11 + Eigen 3.3.9 本机验证为准。
2. 正式冻结 1024x1024 为 Phase 1 当前平台上限，超过该尺寸继续门禁。
3. 全图复数矩阵 replay 后置，Phase 1 保持摘要级 replay。

### 已执行与验证

- 新增 `sar_cxx11_compat`，使用 `-std=c++11` 和 Conan Eigen 3.3.9 编译全部 SAR engine 源文件。
- `ctest --test-dir build/llvm-ninja-debug-eigen339-local -L sar_cxx11_compat --output-on-failure`：1/1 passed。
- Phase 1 工程契约、计划、研究记录和审批报告已同步冻结上述决策。

## 2026-06-05 Phase 2 方向批准与启动

### 用户决策

1. Phase 2 选择“参考级成像与算法对比闭环”。
2. Auto algorithm selector 继续后置。

### 已执行

1. 将 `task_plan.md` 阶段 8 标记为完成。
2. 新增 Phase 2A/2B/2C 分阶段计划和独立审批门。
3. 冻结 Phase 2 当前非目标：
   - Auto 正式运行。
   - L2/L3 与运动补偿。
   - 辐射定标。
   - HDF5/GeoTIFF。
   - 全图复数矩阵 replay。
   - 放宽 Phase 1 public Session `1024x1024` 上限。

### 当前执行点

- Phase 2A 第一批：新增内部图像质量评估组件和确定性测试。

## 2026-06-05 Phase 2A 第一批实现

### 已执行

1. 新增 `SAR_PHASE2_REFERENCE_IMAGING_CONTRACT.md`，冻结 Phase 2A/2B 范围和 Auto 门禁。
2. 新增内部 `SarImageQuality`：
   - 峰值位置与幅度。
   - 距离向/方位向 3dB 宽度。
   - PSLR、ISLR、图像熵。
   - 全局常数相位重参考后的归一化 RMS 误差和相干相关系数。
3. RDA 已直接使用统一质量组件生成现有 diagnostics。
4. 新增可复用确定性点目标参考场景测试支持层。
5. RCMC 内部配置升级为显式 `none/linear/sinc`，public Session 继续固定 linear。
6. 新增有限核 Lanczos-Sinc RCMC、边界计数和诊断测试。
7. 新增 1024x1024 Sinc RCMC 独立性能测量。

### 已验证

- 聚焦/Session 过滤测试：17/17 passed。
- `ctest -L sar_ci`：4/4 passed。
- Eigen 3.3.9 聚焦/Session 过滤测试：17/17 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- 现有 `sar_performance`：passed。
- 1024x1024 独立 RCMC Debug 测量：
  - linear：约 0.024230 s。
  - Sinc 半宽 4：约 0.178153 s。

### 当前审批结论

- Sinc 实现、插值精度优势和性能代价已有证据。
- Sinc 尚未获批成为 public Session 默认路径；真实成像质量优越性等待 GBP 独立参考真值。
- Phase 2A 继续进行，下一工程步骤为 Phase 2B 前置：冻结 GBP 坐标、相位和尺寸契约并实现最小小场景内核。

### 最终回归

- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`：4/4 passed。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_performance --output-on-failure`：1/1 passed，内部 5/5 performance tests passed。
- `ctest --test-dir build/llvm-ninja-debug-eigen339-local -L sar_cxx11_compat --output-on-failure`：1/1 passed。
- `git diff --check`：passed。

## 2026-06-06 L2 工程契约冻结

### 用户批准

- 用户以“继续”批准进入 L2 轨迹误差与一阶运动补偿方向。

### 已执行

1. 新增 `SAR_L2_MOTION_COMPENSATION_CONTRACT.md`。
2. 将 L2 冻结为固定种子速度扰动经时间积分形成的连续位置轨迹。
3. 冻结一阶运动补偿的包络平移、载频相位校正、诊断和验收口径。
4. 新增阶段 12-14 计划。

### 阶段状态

- 阶段 12 L2 工程契约冻结完成。
- 阶段 13 L2 连续扰动轨迹已启动。

## 2026-06-06 阶段 13 L2 连续扰动轨迹

### 已执行

1. `PlatformPulseState` 新增 `velocity_y_mps` 和 `velocity_z_mps`。
2. 新增固定种子三轴速度扰动配置和轨迹误差诊断。
3. 实现速度积分形成的连续 L2 位置轨迹。
4. 全零扰动显式退化为 L1。

### 验证结果

- `SarGeometryTest.*:SarEchoTest.*:SarGbpTest.*:SarRdaTest.*`：20/20 passed。
- `ctest -L sar_ci`：4/4 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 13 完成当前平台审批。
- 阶段 14 一阶运动补偿闭环已启动。

## 2026-06-06 阶段 14 一阶运动补偿闭环

### 已执行

1. 新增内部 `SarMotionCompensation`。
2. 实现相对显式参考点的逐脉冲包络平移和载频相位校正。
3. 新增补偿诊断、零误差恒等测试和 L2 聚焦改善测试。
4. 新增 `1024x1024` 一阶补偿性能测试。
5. 新增 `docs/sar_l2_motion_compensation_acceptance_report.md`。

### 质量证据

- 最大参考斜距误差：约 `1.300414 m`。
- 未补偿：NRMS `1.312794`，相关系数 `0.138285`。
- 补偿后：NRMS `0.249779`，相关系数 `0.968805`。
- `1024x1024` raw history 一阶补偿：约 `0.052741 s`。

### 最终回归

- 默认构建 L2/补偿/GBP/质量/RDA/Session 过滤测试：29/29 passed。
- Eigen 3.3.9 L2/补偿/GBP/质量/RDA/Session 过滤测试：29/29 passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 14 完成当前平台审批。
- 阶段 15 等待选择 public Session 接入、多参考点补偿或 L3。

## 2026-06-06 阶段 15-16 L2 Session 接入决策与契约

### 决策

- 跟随任务执行，选择将 L2 与一阶运动补偿受控接入 public Session。
- 默认关闭；多参考点补偿与 L3 后置。

### 已执行

1. 新增 `SAR_L2_SESSION_INTEGRATION_CONTRACT.md`。
2. 冻结公开配置、参考点、强制补偿、replay 和诊断契约。
3. 新增阶段 17-18 执行计划。

### 阶段状态

- 阶段 15 决策门完成。
- 阶段 16 接入契约完成。
- 阶段 17 公开配置与 replay 已启动。

## 2026-06-06 阶段 17 L2 公开配置与 replay

### 已执行

1. mission 新增三轴 L2 速度扰动标准差与固定 seed。
2. policy 新增默认关闭的 `enable_l2_motion_compensation`。
3. 更新 FlatBuffers session config schema、生成头和 codec。
4. 扩展 round-trip 和 public smoke 测试。

### 验证结果

- `SarReplayCodecRoundtripTest.*`：5/5 passed。
- SAR replay 过滤测试：9/9 passed。
- public SAR smoke：passed。
- `ctest -L sar_ci`：4/4 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 17 完成当前平台审批。
- 阶段 18 Session 执行闭环已启动。

## 2026-06-06 阶段 18 L2 Session 执行闭环

### 已执行

1. Session 新增 latest-N 理想/实际轨迹同步缓冲。
2. 使用实际 L2 轨迹生成 raw echo，并在 RDA 前强制执行一阶运动补偿。
3. 增加 L2 非法配置拒绝、轨迹与补偿结构化诊断。
4. 增加默认关闭、零扰动严格退化、非零扰动、跨周期对齐和 L2 replay 测试。
5. 新增 `docs/sar_l2_session_integration_acceptance_report.md`。

### 最终回归

- 默认构建 SAR 聚焦过滤测试：42/42 passed。
- Eigen 3.3.9 SAR 聚焦过滤测试：42/42 passed。
- 默认与 Eigen 3.3.9 SAR replay 过滤测试：各 9/9 passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 18 完成当前平台审批。
- 阶段 19 L2 后续扩展决策门已就绪。
- 在完成下一方向选择前，Auto、多参考点、L3、二阶补偿和外部逐脉冲轨迹输入继续后置。

## 2026-06-06 阶段 19-20 L3 方向决策与契约

### 决策

- 跟随任务执行，选择 L3 航路点轨迹方向。
- 外部逐脉冲轨迹属于 L4 范围，继续后置。
- 多参考点和二阶补偿缺少当前可审批真值，继续后置。

### 已执行

1. 审计两份建设方案中的 L3/L4、运动补偿和时变 PRF 边界。
2. 新增 `SAR_L3_WAYPOINT_TRAJECTORY_CONTRACT.md`。
3. 冻结显式时间航路点、显式脉冲时刻、分段线性插值和 L1 严格退化契约。
4. 新增阶段 21 L3 几何执行计划。

### 阶段状态

- 阶段 19 决策门完成。
- 阶段 20 L3 航路点轨迹契约完成。
- 阶段 21 L3 航路点轨迹几何已启动。

## 2026-06-06 阶段 21 L3 航路点轨迹几何

### 已执行

1. 新增内部航路点、显式脉冲时刻配置和 L3 轨迹生成器。
2. 实现航段线性位置插值与航段常速度输出。
3. 增加 L1 严格退化、转角命中、非均匀脉冲时刻、重复生成和非法输入拒绝测试。
4. 新增 `docs/sar_l3_waypoint_trajectory_acceptance_report.md`。

### 最终回归

- 默认构建 L3/echo/buffer/补偿/GBP/质量/RDA/Session 过滤测试：42/42 passed。
- Eigen 3.3.9 同过滤测试：42/42 passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 21 完成当前平台审批。
- 阶段 22 L3 raw echo 与成像退化基线已启动。

## 2026-06-06 阶段 22-23 L3 成像退化基线与后续决策

### 已执行

1. 使用折线 L3 实际轨迹生成单点 raw echo。
2. 对比 L1-RDA/L1-GBP 与 L3-RDA/L3-GBP。
3. 记录 `33x9` 扩大场景的既有跨算法基线不一致，不用于 L3 验收。
4. 新增 `docs/sar_l3_imaging_degradation_baseline_report.md`。

### 质量证据

- L1-RDA 相对 L1-GBP：NRMS `0.042218`，相关系数 `0.999109`。
- L3 raw echo 经 L1-RDA 相对 L3-GBP：NRMS `0.501878`，相关系数 `0.874059`。
- L3-GBP 峰值位于预期目标像素邻域。

### 最终回归

- 默认与 Eigen 3.3.9 L3/GBP/补偿/RDA/Session 过滤测试：各 32/32 passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 22 完成当前平台审批。
- 阶段 23 决策选择先审计现有一阶补偿对 L3 的适用性。
- 阶段 24 L3 一阶补偿适用性审计已启动。

### 阶段状态

- Phase 2A 已完成当前平台审批。
- Sinc 作为显式内部实验路径获批；public Session 默认仍为 linear。
- Phase 2B 已启动。
- 由于 Phase 1 与 Phase 2A 变更在当前工作树中已经交织且此前未提交，本次将其作为当前 SAR 已验收基线统一提交；后续严格按每个阶段完成后独立提交。

## 2026-06-05 Phase 2B 小场景 GBP 与跨算法比较

### 已执行

1. 新增内部 `SarGbp` 小场景参考聚焦内核。
2. 冻结 GBP local Cartesian 网格、距离压缩、双程时延插值和相位补偿契约。
3. 新增严格 `128x128` 尺寸门，超限明确拒绝。
4. 新增单点、三点目标和 RDA/GBP 同场景比较测试。
5. 将跨算法 NRMS 修正为两幅图分别单位能量归一化后的形状误差。
6. 新增 `128x128` GBP 独立性能测试。

### 遇到并解决的问题

- C++11 不接受带默认成员初始化的 `LocalPoint{x,y,z}` 三参数聚合构造，已改为逐字段赋值。
- 仅相位对齐的 NRMS 被算法全局增益差异放大，已改为单位能量形状 NRMS。

### 验证结果

- `SarGbpTest.*`：4/4 passed。
- RDA/GBP 相同窗口：相位偏移约 `-0.047659 rad`，NRMS 约 `0.042218`，相干相关系数约 `0.999109`。
- `128x128` GBP Debug 参考场景：约 `0.149 s`。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 CTest entry passed。
- Eigen 3.3.9 聚焦/Session 过滤测试：22/22 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- Phase 2B 已完成当前平台审批。
- public Session 未增加 GBP 或 Auto 入口。
- Phase 2C 审批门已启动。

## 2026-06-05 Phase 2C 扩展审批门

### 已执行

1. 使用 GBP 参考真值补充 linear/Sinc RCMC 三方比较。
2. 新增 `docs/sar_phase2_acceptance_report.md`。
3. 审计 Auto 正式运行前置条件。

### 证据

- Linear 相对 GBP：NRMS `0.042218`，相干相关系数 `0.999109`。
- Sinc 相对 GBP：NRMS `0.042107`，相干相关系数 `0.999114`。
- Sinc 改善极小，独立 RCMC 性能代价约为 linear 的 7.4 倍。

### 审批结论

- Phase 2 当前计划完成。
- Auto 继续后置，不增加 public 入口。
- Sinc 保持内部显式路径，public Session 默认保持 linear。
- 下一方向建议 L2 轨迹误差与一阶运动补偿，必须另行批准。

### 最终回归

- 默认构建 GBP/质量/RDA/Session 过滤测试：23/23 passed。
- Eigen 3.3.9 GBP/质量/RDA/Session 过滤测试：23/23 passed。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`：4/4 passed。
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_performance --output-on-failure`：1/1 passed。
- `ctest --test-dir build/llvm-ninja-debug-eigen339-local -L sar_cxx11_compat --output-on-failure`：1/1 passed。
- `git diff --check`：passed。
