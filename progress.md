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

## 2026-06-11 阶段 95：内部慢时间重采样请求执行器实现

- 实现无状态内部执行器，组合显式请求验证、最大慢时间间隙诊断与二维 raw-history 重采样。
- 为结构无效、缺失脉冲间隙和重采样失败提供独立拒绝原因；拒绝时保持输出为空。
- 新增 4 个单元测试，覆盖成功、缺失拒绝、结构拒绝、输入不变与确定性。
- 默认/Eigen 3.3.9 定向测试均为 4/4 passed；默认完整 CTest 为 25/25 passed。
- Eigen 3.3.9 构建与 `sar_cxx11_compat` 通过。
- 阶段 95 完成；下一步执行阶段 96 后续接入决策门。

## 2026-06-11 阶段 96：内部慢时间重采样执行器后续接入决策门

- 审计 Session、RDA 与 replay/schema 的现有边界。
- 确认 Session/RDA 固定 PRF 语义和 public/schema/replay 缺少显式请求与拒绝传播。
- 决定暂不接入生产链，保持内部显式无状态执行器。
- 下一方向选择扩展时变 PRF 重采样质量矩阵，覆盖多目标、高 Doppler 与确定性随机小抖动。

## 2026-06-11 阶段 97：扩展时变 PRF 重采样质量矩阵契约

- 冻结 E1-E3 场景：M1 中心单点、M4 多目标和方位偏置高 Doppler 单点。
- 冻结 J0-J4：均匀、周期抖动和双 seed 确定性随机小抖动。
- 冻结执行器间隙门、确定性、raw-history/RDA 图像质量和双环境验收要求。
- 阶段 97 完成；下一步实现扩展矩阵。

## 2026-06-11 阶段 98：扩展时变 PRF 重采样质量矩阵实现

- 新增 E1-E3/J0-J4 扩展矩阵测试，覆盖多目标、高 Doppler、周期与双 seed 随机小抖动。
- 15 个 case 均通过执行器间隙门、重采样和 RDA；均匀基线严格退化。
- 默认/Eigen 3.3.9 定向测试各 1/1 passed；默认完整 CTest 25/25 passed。
- Eigen 3.3.9 构建和 C++11 门通过；阶段 98 完成。

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

## 2026-06-06 阶段 24-25 L3 一阶补偿审计与后续决策

### 已执行

1. 对 L3 raw echo 应用现有单参考点一阶运动补偿。
2. 使用 L3-GBP 比较补偿前后成像质量。
3. 量化偏离参考点目标的空间变化残余斜距误差。
4. 新增 `docs/sar_l3_first_order_compensation_audit.md`。

### 质量证据

- 未补偿：NRMS `0.501878`，相关系数 `0.874059`。
- 一阶补偿后：NRMS `0.185881`，相关系数 `0.982724`。
- 偏离参考点最大空间残余斜距误差：约 `0.000180 m`。

### 最终回归

- 默认与 Eigen 3.3.9 L3/GBP/补偿/RDA/Session 过滤测试：各 33/33 passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 24 完成当前平台审批。
- 阶段 25 不批准二阶补偿、BP 或多参考点；选择先建立适用边界矩阵。
- 阶段 26 L3 一阶补偿适用边界矩阵已启动。

## 2026-06-06 阶段 26-27 L3 适用边界矩阵与失效区决策

### 已执行

1. 扫描孔径末端横向偏移 `0/1/3/6/12 m`。
2. 扫描 `12 m` 转弯下目标距离单元 `20/18/16/12` 的空间残余。
3. 识别当前一阶补偿通过区和明确失效区。
4. 审计 GBP 与 BP 的数学及实现职责差异。
5. 新增 `docs/sar_l3_first_order_applicability_matrix_report.md`。

### 关键证据

- `0/1/3/6 m` 当前门通过。
- `12 m`：补偿后 NRMS `0.386100`，相关系数 `0.925463`，明确失效。
- `12 m` 转弯下最远目标偏移的最大残余斜距误差：`0.007119 m`。

### 最终回归

- 默认与 Eigen 3.3.9 L3/GBP/补偿/RDA/Session 过滤测试：各 34/34 passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 26 完成当前平台审批。
- 阶段 27 选择共享后向投影核心上的 L3 BP 路径。
- 阶段 28 L3 BP 工程契约已启动。

## 2026-06-06 阶段 28 L3 BP 工程契约

### 已执行

1. 新增 `SAR_L3_BP_CONTRACT.md`。
2. 冻结 GBP/BP 共享数学核心与唯一遍历顺序差异。
3. 冻结 L1/L3 逐样本一致性、L3 失效区质量和独立性能门。

### 阶段状态

- 阶段 28 L3 BP 工程契约完成。
- 阶段 29 L3 BP 内部闭环已启动。

## 2026-06-06 阶段 29-30 L3 BP 内部闭环与接入决策

### 已执行

1. 提取 GBP/BP 共享后向投影核心。
2. 新增脉冲优先 BP 内部入口和遍历顺序诊断。
3. 验证 L1/L3 下 GBP/BP 复图逐样本一致。
4. 验证 BP 在 `12 m` L3 一阶补偿失效区优于补偿后 RDA。
5. 新增 `128x128` BP 独立性能测试。
6. 新增 `docs/sar_l3_bp_acceptance_report.md`。

### 性能证据

- `128x128` GBP Debug：约 `0.183516 s`。
- `128x128` BP Debug：约 `0.177757 s`。

### 最终回归

- 默认与 Eigen 3.3.9 L3/GBP/BP/补偿/RDA/Session 过滤测试：各 35/35 passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 29 完成当前平台审批。
- 阶段 30 选择冻结 L3 BP public Session 接入契约，Auto 继续后置。
- 阶段 31 L3 BP Session 接入契约已启动。

## 2026-06-06 阶段 31 L3 BP Session 接入契约

### 已执行

1. 新增 `SAR_L3_BP_SESSION_INTEGRATION_CONTRACT.md`。
2. 冻结 public waypoint 的 LLA、相对 Session 起点时间和固定 PRF 契约。
3. 冻结 L3 BP 与 L1-RDA/L2 互斥、独立 `128x128` 门和结构化拒绝。
4. 冻结输出阶段、diagnostics、session config replay 和 runtime patch 禁止边界。

### 阶段状态

- 阶段 31 L3 BP Session 接入契约完成。
- 阶段 32 L3 BP 公开配置与 replay 已启动。

## 2026-06-06 阶段 32 L3 BP 公开配置与 replay

### 已执行

1. public mission 新增 `SarWaypointConfig` 与 waypoint 列表。
2. public policy 新增默认关闭的 `enable_l3_bp_imaging`。
3. output 新增 `kL3BpImage` 与 `has_l3_bp_image`。
4. 更新 session/cycle FlatBuffers schema、生成头和 codec。
5. 扩展 round-trip 与 public smoke。

### 验证结果

- 默认与 Eigen 3.3.9 replay/Session 过滤测试：各 18/18 passed。
- 默认与 Eigen 3.3.9 replay fast 过滤测试：各 9/9 passed。
- public SAR smoke：passed。
- `ctest -L sar_ci`：4/4 passed。
- `ctest -L sar_performance`：1/1 passed。
- Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 32 完成当前平台审批。
- 阶段 33 L3 BP Session 执行闭环已启动。

## 2026-06-06 阶段 33 L3 BP Session 执行闭环

### 已执行

1. Session 按 public LLA waypoint 和连续 `pulse_id / PRF` 生成 L3 实际轨迹。
2. 使用 L3 实际轨迹生成 raw echo，并通过共享后向投影核心执行 `pulse_major` BP。
3. 新增 L1/L2 互斥、raw/range 前置条件、waypoint 结构、覆盖范围和 `128x128` 尺寸结构化拒绝。
4. 新增 `sar.l3_trajectory`、`sar.bp_peak`、`sar.bp_traversal` diagnostics。
5. 新增跨周期 aperture 对齐和两周期 L3 BP trace replay 测试。
6. 新增 `docs/sar_l3_bp_session_integration_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 全部 `Sar*` 单测：各 73/73 passed。
- 默认与 Conan Eigen 3.3.9 SAR replay-fast：各 10/10 passed。
- 默认与 Conan Eigen 3.3.9 `ctest -L sar_ci`：各 4/4 passed。
- 默认 `ctest -L sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `ctest -L sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 独立限制

- 全仓无目标构建在既有 `flight_dynamic` 测试处被缺失的 JSBSim `FGFDMExec.h` 阻断；SAR 独立目标与门禁均通过。

### 阶段状态

- 阶段 33 完成当前平台审批。
- Auto、runtime patch、全图复矩阵 replay、时变 PRF 和 BP 尺寸扩展继续后置。
- 阶段 34 Phase 2 完成度审计已启动。

## 2026-06-06 阶段 34-35 Phase 2 完成度审计与参考矩阵契约

### 已执行

1. 按参考成像、算法对比、L2/L3 适用边界、public Session、replay 和性能逐项审计现有证据。
2. 明确当前闭环仅批准固定 PRF、小场景、点目标范围，不外推为通用质量声明。
3. 明确 Auto 仍缺代表性参考矩阵、选择阈值、资源预算、降级规则和结构化选择诊断。
4. 选择参考场景矩阵扩展作为下一方向。
5. 新增 `docs/sar_phase2_reference_closure_audit.md`。
6. 新增 `SAR_REFERENCE_SCENARIO_MATRIX_CONTRACT.md`，冻结 M1-M7 首批矩阵。

### 阶段状态

- 阶段 34 完成。
- 阶段 35 契约冻结完成。
- 阶段 36 参考场景矩阵实现已启动。

## 2026-06-06 阶段 36 首批参考场景矩阵实现

### 已执行

1. 在参考场景支持层新增二维局部坐标目标构建 helper。
2. 新增独立 `SarReferenceScenarioMatrixTest`，实现 M1-M7。
3. M1-M4 验证 L1 中心、距离偏置、方位偏置和二维多目标的 RDA/GBP/BP 对比。
4. M5 验证 L2 二维目标零扰动退化与固定 seed 补偿改善。
5. M6/M7 复核 L3 二维目标 `3 m` 通过区与 `12 m` 失效区。
6. 新增 `docs/sar_reference_scenario_matrix_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 M1-M7：各 7/7 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*` 单测：各 80/80 passed。
- 默认与 Conan Eigen 3.3.9 `ctest -L sar_ci`：各 4/4 passed。
- 默认 `ctest -L sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `ctest -L sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 36 完成当前平台审批。
- 阶段 37 参考矩阵后续决策门已启动。

## 2026-06-06 阶段 37-38 边界与参数矩阵决策及契约

### 决策结论

- 选择图像边界与采样/硬件参数适用性矩阵作为下一方向。
- 时变 PRF、真实动力学 L3、多参考点、二阶补偿和 Auto 继续后置。

### 已执行

1. 新增 `SAR_REFERENCE_BOUNDARY_PARAMETER_MATRIX_CONTRACT.md`。
2. 冻结四类边界结果：`interior_pass / boundary_degraded / echo_clipped / invalid`。
3. 冻结 B1-B4 距离/方位边界矩阵。
4. 冻结 P1-P4 采样率、载频、PRF 和平台速度单参数扫描。
5. 冻结通过档位沿用既有门、失败档位保留真实边界。

### 阶段状态

- 阶段 37 完成。
- 阶段 38 契约冻结完成。
- 阶段 39 边界与参数矩阵实现已启动。

## 2026-06-06 阶段 39 边界与参数适用性矩阵

### 已执行

1. 扩展参考 raw history helper，可选汇总裁剪脉冲、目标和样本数。
2. 实现 B1-B4，区分内部通过、图像边缘退化和 raw echo 裁剪。
3. 实现 P1-P4，扫描采样率、载频、PRF 和平台速度。
4. 保留 PRF/速度退化档位，不放宽当前质量门。
5. 新增 `docs/sar_boundary_parameter_matrix_acceptance_report.md`。

### 关键证据

- raw echo 裁剪场景仍可得到 NRMS `0.014853`、相关系数 `0.999890`，证明裁剪诊断必须独立门禁。
- `v/PRF=0.2 m/pulse` 的两组配置均得到 NRMS `0.177589`、相关系数 `0.984231` 并失败。
- `v/PRF=0.05/0.1 m/pulse` 当前门通过。

### 验证结果

- 默认与 Conan Eigen 3.3.9 M1-M7 + B1-B4 + P1-P4：各 13/13 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*` 单测：各 86/86 passed。
- 默认与 Conan Eigen 3.3.9 `ctest -L sar_ci`：各 4/4 passed。
- 默认 `ctest -L sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `ctest -L sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 39 完成当前平台审批。
- 阶段 40 方位采样充分性决策门已启动。

## 2026-06-06 阶段 40-41 方位采样审计与诊断契约

### 已执行

1. 连续扫描 `0.05-0.2 m/pulse` 方位采样间距。
2. 计算最大几何 Doppler Nyquist 裕量和相邻传播相位步进。
3. 隔离 none/linear/sinc RCMC，排除插值主因。
4. 使用等曲率载频/斜距参数对验证相位曲率解释。
5. 新增 `docs/sar_azimuth_sampling_audit.md`。
6. 新增 `SAR_RDA_AZIMUTH_PHASE_CURVATURE_DIAGNOSTIC_CONTRACT.md`。

### 决策结论

- 当前粗间距质量失败不是几何 Doppler Nyquist 混叠。
- 每脉冲二阶方位相位曲率可解释现有参数矩阵。
- 下一步只增加解释性 diagnostics，不增加警告、拒绝或 Auto。

### 阶段状态

- 阶段 40 完成。
- 阶段 41 契约冻结完成。
- 阶段 42 RDA 方位相位曲率诊断实现已启动。

## 2026-06-06 阶段 42 RDA 方位相位曲率诊断实现

### 已执行

1. 扩展 `RdaDiagnostics`，增加方位采样间距、相位曲率、最大几何 Doppler 和 Nyquist 裕量。
2. 提取 `ComputeRdaSamplingDiagnostics()`，支持独立验证单脉冲无穷裕量定义。
3. 在 Session `sar.rda_peak` 中追加四项诊断，保持摘要级 replay。
4. 扩展 RDA 单测与方位采样审计矩阵，直接核对生产 diagnostics。
5. 新增 `docs/sar_rda_phase_curvature_diagnostics_acceptance_report.md`。

### 遇到并解决的问题

- 使用单脉冲完整 RDA 链验证无穷裕量时，现有 FFT 成像链异常退出。
- 解决方案：RDA 成像入口明确拒绝单脉冲 aperture；独立验证采样诊断定义，不扩大现有 RDA FFT 成像输入范围。

### 验证结果

- 默认与 Conan Eigen 3.3.9 聚焦诊断测试：各 15/15 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*`：各 92/92 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 42 完成当前平台审批。
- 阶段 43 RDA 诊断后续决策门已启动。

## 2026-06-06 阶段 43 RDA 诊断后续决策门

### 已执行

1. 新增 aperture `5/9/17/33`、间距 `0.1/0.2 m/pulse` 和中心/偏置目标矩阵。
2. 直接记录生产 RDA diagnostics、RDA/GBP NRMS 和相关系数。
3. 验证孔径二次相位跨度对中心目标等效组合的归并能力。
4. 新增 `docs/sar_rda_diagnostic_followup_decision.md`。

### 决策结论

- 每脉冲相位曲率不足以形成跨 aperture 质量警告阈值。
- 孔径二次相位跨度是更完整的解释性指标候选。
- 目标方位布局仍产生独立误差，因此当前不批准质量警告、结构化拒绝或 Auto。

### 验证结果

- 默认与 Conan Eigen 3.3.9 决策矩阵：各 1/1 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*`：各 93/93 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 43 完成。
- 阶段 44 RDA 孔径二次相位跨度诊断契约已启动。

## 2026-06-06 阶段 44 RDA 孔径二次相位跨度诊断契约

### 已执行

1. 新增 `SAR_RDA_APERTURE_PHASE_SPAN_DIAGNOSTIC_CONTRACT.md`。
2. 冻结 `azimuth_quadratic_phase_span_rad` 公式、Session message 和 replay 语义。
3. 冻结单脉冲跨度为 `0`。
4. 明确目标方位偏置影响不由该指标覆盖。

### 阶段状态

- 阶段 44 契约冻结完成。
- 阶段 45 RDA 孔径二次相位跨度诊断实现已启动。

## 2026-06-06 阶段 45 RDA 孔径二次相位跨度诊断实现

### 已执行

1. 扩展 `RdaDiagnostics`，增加 `azimuth_quadratic_phase_span_rad`。
2. 使用实际 aperture 脉冲数计算跨度，单脉冲定义为 `0`。
3. Session `sar.rda_peak` 追加新指标，replay 严格比较通过。
4. 阶段 43 决策矩阵改为直接核对生产 diagnostics。
5. 新增 `docs/sar_rda_aperture_phase_span_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 聚焦诊断测试：各 14/14 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*`：各 93/93 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 45 完成当前平台审批。
- 阶段 46 RDA 目标方位偏置误差决策门已启动。

## 2026-06-06 阶段 46 RDA 目标方位偏置误差决策门

### 已执行

1. 新增等物理孔径、等归一化目标偏置矩阵，分离孔径、采样密度与目标布局。
2. 计算目标偏置相对 broadside 中心切线的非线性相位残差。
3. 新增正负目标偏置对称性矩阵。
4. 新增 `docs/sar_rda_target_azimuth_offset_decision.md`。

### 决策结论

- 目标偏置额外误差对方向对称，并随偏置幅值增加。
- 非线性相位残差不能单独解释额外误差；质量还依赖孔径相位跨度和采样密度。
- 不新增生产 diagnostics、质量警告、结构化拒绝或 Auto。

### 验证结果

- 默认与 Conan Eigen 3.3.9 目标偏置决策测试：各 2/2 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*`：各 95/95 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 46 决策完成。
- 阶段 47 Phase 2 参考级成像与算法对比闭环综合再审批已启动。

## 2026-06-06 阶段 47 Phase 2 参考级成像闭环综合再审批

### 已执行

1. 汇总 L1 RDA、L2 一阶补偿、L3 BP、GBP/BP 参考、边界参数矩阵和 RDA diagnostics 证据。
2. 复核 public Session、replay、尺寸门和 Auto 前置条件。
3. 新增 `docs/sar_phase2_reference_imaging_reapproval_report.md`。

### 审批结论

- 当前固定 PRF、点目标、显式算法路径完成综合再审批。
- Auto、尺寸扩展、时变 PRF、真实动力学、全图 replay、杂波和辐射定标继续后置。
- 下一方向批准测试侧确定性噪声与 SNR 鲁棒性参考矩阵，不新增生产或 public 能力。

### 阶段状态

- 阶段 47 完成。
- 阶段 48 确定性噪声与 SNR 鲁棒性矩阵契约已启动。

## 2026-06-06 阶段 48 确定性噪声与 SNR 鲁棒性矩阵契约

### 已执行

1. 新增 `SAR_REFERENCE_SNR_MATRIX_CONTRACT.md`。
2. 冻结测试侧固定 seed 复高斯噪声、精确能量缩放和 realized SNR 定义。
3. 冻结 M1/M4、无噪声/30/20/10/0 dB 与双 seed 首批矩阵。
4. 明确 noisy raw history 由 RDA、GBP 与 BP 共同消费。
5. 明确不增加生产/public/replay/Auto/杂波/辐射定标能力。

### 阶段状态

- 阶段 48 契约冻结完成。
- 阶段 49 确定性噪声与 SNR 鲁棒性矩阵实现已启动。

## 2026-06-06 阶段 49 确定性噪声与 SNR 鲁棒性矩阵实现

### 已执行

1. 在测试支持层实现固定 seed 复高斯噪声 helper 与精确能量缩放 diagnostics。
2. 新增 M1/M4、双 seed、`30/20/10/0 dB` 鲁棒性矩阵。
3. 验证同 seed 严格重复、不同 seed 差异和 requested/realized SNR 一致。
4. 验证 RDA/GBP/BP 共同消费 noisy raw history，BP 与 GBP 继续逐样本一致。
5. 新增 `docs/sar_reference_snr_matrix_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 噪声 helper/SNR 矩阵测试：各 2/2 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*`：各 97/97 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 49 完成当前平台审批。
- 阶段 50 SNR 矩阵后续决策门已启动。

## 2026-06-06 阶段 50 SNR 矩阵后续决策门

### 已执行

1. 将 RDA clean/noisy 比较裁剪到与 GBP 相同的小场景窗口。
2. 复核双 seed、多 SNR 的退化趋势。
3. 新增 `docs/sar_reference_snr_followup_decision.md`。

### 决策结论

- 统一窗口比较口径获批。
- 当前不增加更多 seed 或 SNR 档位，不冻结质量阈值。
- 下一方向选择测试侧确定性分布式杂波参考模型。

### 阶段状态

- 阶段 50 完成。
- 阶段 51 确定性分布式杂波参考模型契约已启动。

## 2026-06-10 阶段 51 确定性分布式杂波参考模型契约

### 已执行

1. 新增 `SAR_DETERMINISTIC_DISTRIBUTED_CLUTTER_CONTRACT.md`。
2. 冻结规则网格散射点、固定 seed、显式 PRNG、复幅相和稳定遍历顺序。
3. 冻结 target/clutter raw-history 总能量定义的 requested/realized SCR。
4. 冻结 M1/M4、双网格密度、双 seed 与无杂波/30/20/10/0 dB 首批矩阵。
5. 冻结测试支持层边界，继续禁止生产杂波、公用配置、绝对功率、辐射定标、
   质量阈值、警告、拒绝和 Auto。

### 阶段状态

- 阶段 51 契约完成。
- 阶段 52 确定性分布式杂波参考模型实现待启动。

## 2026-06-10 阶段 52 确定性分布式杂波参考模型实现

### 已执行

1. 在 `tests/support/sar_reference_scene.h` 新增规则网格确定性杂波 helper 与 diagnostics。
2. 逐散射点复用现有点目标 raw echo 链路，并使用固定 seed 零均值复系数叠加杂波。
3. 按 target/clutter raw-history 总能量精确缩放 requested SCR。
4. 新增确定性、不同 seed、无裁剪、算法共享输入和 BP/GBP 一致性测试。
5. 新增 M1/M4、`3x3/5x5`、双 seed、`30/20/10/0 dB` 首批矩阵。
6. 新增 `docs/sar_deterministic_distributed_clutter_acceptance_report.md`。

### 关键证据

- M1 sparse seed 17：RDA/clean NRMS 从 `30 dB` 的 `0.014621` 增至 `0 dB` 的
  `0.619501`；GBP 从 `0.014217` 增至 `0.606510`。
- M4 dense seed 29：RDA/clean NRMS 从 `0.019897` 增至 `0.362255`；GBP 从
  `0.019850` 增至 `0.361540`。
- 新增杂波 helper 与矩阵测试：2/2 passed。

### 阶段状态

- 阶段 52 完成当前平台实现。
- 阶段 53 分布式杂波后续决策门待启动。

## 2026-06-10 阶段 53 分布式杂波后续决策门

### 已执行

1. 复核规则网格、双密度、双 seed 和首批 SCR 趋势证据。
2. 复核随机位置、相关杂波、更多密度、SNR/SCR 二维矩阵和生产杂波候选方向。
3. 新增 `docs/sar_distributed_clutter_followup_decision.md`。

### 决策结论

- 当前没有足够证据冻结随机位置分布、空间相关函数或相关长度，相关杂波继续后置。
- 下一阶段选择确定性噪声与分布式杂波 SNR/SCR 二维矩阵契约。
- 生产杂波、绝对功率、辐射定标、通用阈值、质量警告、拒绝和 Auto 继续禁止。

### 阶段状态

- 阶段 53 决策完成。
- 阶段 54 SNR/SCR 二维矩阵契约待启动。

## 2026-06-10 阶段 54 确定性噪声与杂波 SNR/SCR 二维矩阵契约

### 已执行

1. 新增 `SAR_REFERENCE_SNR_SCR_MATRIX_CONTRACT.md`。
2. 冻结纯目标、噪声和杂波三份独立 raw-history 分量。
3. 冻结 SNR/SCR 均以纯目标能量为共同参考，禁止相互隐式耦合。
4. 冻结噪声与杂波独立 seed、联合加法顺序无关和算法共享输入语义。
5. 冻结 M1 `3x3` 二维矩阵与 M4 哨兵矩阵。
6. 继续禁止生产噪声/杂波、相关杂波、阈值、警告、拒绝和 Auto。

### 阶段状态

- 阶段 54 契约完成。
- 阶段 55 SNR/SCR 二维矩阵实现待启动。

## 2026-06-10 阶段 55 确定性噪声与杂波 SNR/SCR 二维矩阵实现

### 执行内容

1. 增加测试侧联合配置、独立分量输出和联合 diagnostics helper。
2. 验证固定参数可重复、噪声/杂波 seed 隔离和注入顺序数值一致。
3. 完成 M1 `3x3` 完整矩阵与 M4 clean/20 dB/0 dB 哨兵矩阵。
4. 验证 requested/realized SNR/SCR、无裁剪和所有联合输入下 BP/GBP 逐样本一致。
5. 新增 `docs/sar_reference_snr_scr_matrix_acceptance_report.md`。

### 验证结果

- Windows Debug 编译通过。
- 默认环境联合矩阵 `2/2`、全部参考矩阵 `13/13` 通过。
- Conan Eigen 3.3.9 联合矩阵 `2/2` 与 `sar_cxx11_compat` 通过。
- 默认 Windows Debug 完整 CTest `25/25` 通过。

### 当前状态

- 阶段 55 完成当前平台审批。
- 下一步进入阶段 56 联合 SNR/SCR 矩阵后续决策门。
- 后续范围只覆盖 `1.1.4.4 SAR雷达组件`。

## 2026-06-10 阶段 56 联合 SNR/SCR 矩阵后续决策门

### 决策结论

- 首批矩阵已充分覆盖确定性和总体联合退化趋势，不继续增加中间档位。
- 相关杂波缺少可批准的统计模型，联合质量诊断缺少有效性阈值证据，均继续后置。
- 下一方向选择 `1.1.4.4.3.4 辐射定标模块`，先冻结测试侧闭环契约。
- 新增 `docs/sar_joint_snr_scr_followup_decision.md`。

### 阶段状态

- 阶段 56 完成。
- 阶段 57 辐射定标闭环工程契约待启动。

## 2026-06-10 阶段 57 辐射定标闭环工程契约

### 已执行

1. 新增 `SAR_RADIOMETRIC_CALIBRATION_CONTRACT.md`。
2. 冻结首批图像响应定标因子，不将其误声明为完整真实系统雷达方程常数。
3. 冻结 RCS 反演、辐射误差和显式正权重多点融合。
4. 识别并禁止单点零残差倒数权重。
5. 冻结聚焦链路与受控联合干扰验收矩阵。

### 阶段状态

- 阶段 57 契约完成。
- 阶段 58 辐射定标内部闭环实现待启动。

## 2026-06-10 阶段 58 辐射定标内部闭环实现

### 执行内容

1. 新增内部标量辐射定标模块，支持单点、多点融合、RCS 反演和 dB 误差评估。
2. 增加尺度不变性、显式权重融合和无效输入测试。
3. 使用 M1 未归一化 GBP 图像建立定标因子并反演 M4 隔离目标。
4. 记录 `20 dB SNR + 20 dB SCR` 联合干扰下的辐射误差。
5. 新增 `docs/sar_radiometric_calibration_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 定标测试各 `5/5` 通过。
- 默认 Windows Debug 完整 CTest `25/25` 通过。
- Conan Eigen 3.3.9 `sar_cxx11_compat` 通过。

### 阶段状态

- 阶段 58 完成当前平台审批。
- 阶段 59 辐射定标后续接入决策门待启动。

## 2026-06-10 阶段 59 辐射定标后续接入决策门

### 决策结论

- public Session 尚无显式标定目标选择、未归一化像素功率和定标生命周期语义。
- 完整系统因子和有效性阈值证据不足，继续后置。
- 下一方向选择显式标定观测工程契约。
- 新增 `docs/sar_radiometric_calibration_followup_decision.md`。

### 阶段状态

- 阶段 59 完成。
- 阶段 60 显式标定观测工程契约待启动。

## 2026-06-10 阶段 60 显式标定观测工程契约

### 已执行

1. 新增 `SAR_EXPLICIT_CALIBRATION_OBSERVATION_CONTRACT.md`。
2. 冻结显式标定身份、指定像素功率、中心脉冲斜距和孔径 pulse id 范围。
3. 冻结不可变观测值对象和显式重新定标生命周期。
4. 禁止从普通目标列表或全局峰值隐式选择标定器。

### 阶段状态

- 阶段 60 契约完成。
- 阶段 61 显式标定观测内部实现待启动。

## 2026-06-10 阶段 61 显式标定观测内部实现

### 执行内容

1. 扩展内部定标模块，增加显式观测请求、观测值和样本转换。
2. 实现指定未归一化像素功率提取与严格输入验证。
3. 保证多点观测转换原子性，无效列表不部分覆盖输出。
4. 使用 M1 GBP/BP 显式观测进入定标核心并保持 M4 闭环。
5. 新增 `docs/sar_explicit_calibration_observation_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 定标观测测试各 `6/6` 通过。
- 默认 Windows Debug 完整 CTest `25/25` 通过。
- Conan Eigen 3.3.9 `sar_cxx11_compat` 通过。

### 阶段状态

- 阶段 61 完成当前平台审批。
- 阶段 62 显式标定观测后续决策门待启动。

## 2026-06-10 阶段 62 显式标定观测后续决策门

### 决策结论

- 下一方向选择内部显式校准请求与 Session 执行边界。
- 自动像素定位与主瓣积分功率缺少独立误差口径，继续后置。
- public diagnostics、schema、runtime patch 和 replay 继续后置。
- 新增 `docs/sar_explicit_calibration_observation_followup_decision.md`。

### 阶段状态

- 阶段 62 完成。
- 阶段 63 Session 内部显式校准请求契约待启动。

## 2026-06-10 阶段 63 Session 内部显式校准请求契约

### 已执行

1. 新增 `SAR_INTERNAL_SESSION_CALIBRATION_REQUEST_CONTRACT.md`。
2. 冻结内部显式请求、无状态执行器和 RDA/GBP/BP 路径匹配。
3. 冻结原子失败、结构化失败原因和不影响原聚焦结果语义。
4. public API、SarSession、schema、trace、replay 与有效性阈值继续后置。

### 阶段状态

- 阶段 63 契约完成。
- 阶段 64 Session 内部显式校准执行器实现待启动。

## 2026-06-10 阶段 64 Session 内部显式校准执行器实现

### 执行内容

1. 实现内部无状态校准执行器与结构化失败原因。
2. 实现路径匹配、原子失败、多请求融合和残差摘要。
3. 完成 M1 RDA/GBP/BP 显式路径执行矩阵。
4. 新增 `docs/sar_internal_calibration_executor_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 执行器与定标测试各 `7/7` 通过。
- 默认 Windows Debug 完整 CTest `25/25` 通过。
- Conan Eigen 3.3.9 `sar_cxx11_compat` 通过。

### 阶段状态

- 阶段 64 完成当前平台审批。
- 阶段 65 内部校准执行器后续接入决策门待启动。

## 2026-06-10 阶段 65 内部校准执行器后续接入决策门

### 决策结论

- `SarSession` 尚无可审计的显式校准请求来源，直接接入继续后置。
- 内部执行器保持独立参考能力，不生成隐式请求或无输出死分支。
- 下一方向转入 SAR 剩余聚焦能力审计。
- 新增 `docs/sar_internal_calibration_executor_followup_decision.md`。

### 阶段状态

- 阶段 65 完成。
- 阶段 66 SAR 剩余聚焦能力审计待启动。

## 2026-06-10 阶段 66 SAR 剩余聚焦能力审计

### 决策结论

- CSA/Omega-K、自聚焦与时变 PRF 成像的关键数学和真值前置条件不足，继续后置。
- RDA、GBP、BP 已具备明确适用边界、尺寸门和质量/性能证据。
- 下一方向选择内部确定性聚焦算法选择器，首批只建议、不执行。
- 新增 `docs/sar_remaining_focusing_capability_audit.md`。

### 阶段状态

- 阶段 66 完成。
- 阶段 67 内部确定性聚焦选择器契约待启动。

## 2026-06-10 阶段 67 内部确定性聚焦选择器契约

### 已执行

1. 新增 `SAR_INTERNAL_FOCUSING_SELECTOR_CONTRACT.md`。
2. 冻结 L1/L2 RDA、独立 GBP 参考和 L3 BP 的显式建议规则。
3. 冻结尺寸门、可用性门、目的/轨迹一致性和结构化拒绝原因。
4. 明确选择器只建议、不执行、不回退，public Auto 继续后置。

### 阶段状态

- 阶段 67 契约完成。
- 阶段 68 内部确定性聚焦选择器实现待启动。

## 2026-06-10 阶段 68 内部确定性聚焦选择器实现

### 执行内容

1. 新增内部确定性聚焦算法建议器。
2. 实现 L1/L2 RDA、独立 GBP 参考和 L3 BP 建议规则。
3. 实现算法可用性、尺寸门和前置条件结构化拒绝。
4. 新增 `docs/sar_internal_focusing_selector_acceptance_report.md`。

### 验证结果

- 默认与 Conan Eigen 3.3.9 选择器测试各 `4/4` 通过。
- 默认 Windows Debug 完整 CTest `25/25` 通过。
- Conan Eigen 3.3.9 `sar_cxx11_compat` 通过。

### 阶段状态

- 阶段 68 完成当前平台审批。
- 阶段 69 内部聚焦选择器后续接入决策门待启动。

## 2026-06-10 阶段 69 内部聚焦选择器后续接入决策门

### 决策结论

- public Session 缺少显式调用目的，建议器接入与 public Auto 继续后置。
- 建议器保持独立内部参考能力，不扩大 schema/replay 表面。
- 下一方向选择 CSA 数学与参考真值工程契约。
- 新增 `docs/sar_internal_focusing_selector_followup_decision.md`。

### 阶段状态

- 阶段 69 完成。
- 阶段 70 CSA 数学与参考真值工程契约待启动。

## 2026-06-10 阶段 70 CSA 数学与参考真值工程契约

### 已执行

1. 新增 `SAR_CSA_MATH_REFERENCE_CONTRACT.md`。
2. 冻结未 shift 二维频率轴、`D(f_a)`、`alpha(f_a)` 和有效域诊断。
3. 明确现有高层流程不足以批准完整 CSA 相位函数。
4. 冻结完整 CSA 实现前的独立中间域参考真值要求。

### 阶段状态

- 阶段 70 契约完成。
- 阶段 71 CSA 频率几何基础实现待启动。

## 2026-06-10 阶段 71 CSA 频率几何基础实现

### 已执行

1. 新增内部 `SarCsaGeometry`，生成未 shift 距离/方位频率轴。
2. 实现波长、`D(f_a)`、`alpha(f_a)` 与有效域诊断。
3. 对无效 Doppler bin 计数并保持输出有限，不允许无提示 NaN/Inf。
4. 新增奇偶轴、对称性、边界、非法输入与确定性测试。
5. 新增 `docs/sar_csa_frequency_geometry_acceptance_report.md`。

### 验证结果

- 默认 `SarCsaGeometryTest.*`：`4/4` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `SarCsaGeometryTest.*`：`4/4` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 71 完成当前平台审批。
- 完整 CSA 相位函数继续后置。
- 阶段 72 CSA 频率几何后续决策门待启动。

## 2026-06-10 阶段 72 CSA 频率几何后续决策门

### 决策结论

- 仓库没有完整 CSA 相位函数、固定复数中间域结果或独立参考脚本。
- 完整 CSA 聚焦与内部选择器接入继续后置。
- CSA 频率几何基础保持独立内部诊断能力。
- 下一方向选择 Omega-K 数学与参考真值工程契约。
- 新增 `docs/sar_csa_frequency_geometry_followup_decision.md`。

### 阶段状态

- 阶段 72 完成。
- 阶段 73 Omega-K 数学与参考真值工程契约待启动。

## 2026-06-10 阶段 73 Omega-K 数学与参考真值工程契约

### 已执行

1. 新增 `SAR_OMEGA_K_MATH_REFERENCE_CONTRACT.md`。
2. 冻结 `K_r`、`K_x`、`K_z` 和传播色散有效域。
3. 冻结均匀目标 `K_z` 网格反求源距离频率的 Stolt 查询映射。
4. 冻结无效色散点、越支持区查询、shift 和独立真值要求。
5. 明确首批只批准几何诊断，不批准复数插值或完整聚焦。

### 阶段状态

- 阶段 73 契约完成。
- 阶段 74 Omega-K Stolt 几何基础实现待启动。

## 2026-06-10 阶段 74 Omega-K Stolt 几何基础实现

### 已执行

1. 新增内部 `SarOmegaKGeometry`，计算双程波数、传播色散和 Stolt 查询位置。
2. 分别诊断色散无效点与越支持区查询，并保持所有输出有限。
3. 新增未 shift 轴、零 Doppler、对称性、单调趋势、失败分类和确定性测试。
4. 修正零 Doppler 波数反算产生的微小 shift，使其成为精确零不变量。
5. 新增 `docs/sar_omega_k_stolt_geometry_acceptance_report.md`。

### 验证结果

- 默认 `SarOmegaKGeometryTest.*`：`5/5` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `SarOmegaKGeometryTest.*`：`5/5` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 74 完成当前平台审批。
- 完整 Omega-K 聚焦继续后置。
- 阶段 75 Omega-K Stolt 几何后续决策门待启动。

## 2026-06-10 阶段 75 Omega-K Stolt 几何后续决策门

### 决策结论

- 仓库缺少复数 Stolt 插值真值、越支持区策略和完整参考相位。
- 完整 Omega-K 聚焦与内部选择器接入继续后置。
- Stolt 几何保持独立内部诊断能力。
- 下一方向选择自聚焦残余相位误差与参考真值工程契约。
- 新增 `docs/sar_omega_k_stolt_geometry_followup_decision.md`。

### 阶段状态

- 阶段 75 完成。
- 阶段 76 自聚焦残余相位误差与参考真值工程契约待启动。

## 2026-06-10 阶段 76 自聚焦残余相位误差与参考真值工程契约

### 已执行

1. 新增 `SAR_AUTOFOCUS_PHASE_ERROR_REFERENCE_CONTRACT.md`。
2. 冻结归一化孔径坐标和低阶多项式相位误差真值。
3. 冻结常量/线性不可观测分量的最小二乘去除。
4. 冻结可观测残余、真值校正、误差指标和验收矩阵。
5. 明确 PGA、图像熵优化和生产接入继续后置。

### 阶段状态

- 阶段 76 契约完成。
- 阶段 77 自聚焦相位误差真值基础实现待启动。

## 2026-06-10 阶段 77 自聚焦相位误差真值基础实现

### 已执行

1. 新增内部 `SarAutofocusPhaseTruth`，生成低阶相位误差剖面。
2. 使用离散最小二乘去除常量/线性不可观测分量。
3. 输出可观测残余、反向校正剖面和结构化统计。
4. 新增 `docs/sar_autofocus_phase_truth_acceptance_report.md`。

### 验证结果

- 默认与 Eigen 3.3.9 定向测试各 `4/4` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 77 完成当前平台审批。
- 阶段 78 自聚焦相位真值后续决策门待启动。

## 2026-06-10 阶段 78 自聚焦相位真值后续决策门

### 决策结论

- 仓库缺少 PGA 支持区、梯度估计真值、unwrap 和停止准则。
- PGA、生产图像校正与 Session 接入继续后置。
- 相位误差真值保持独立内部诊断能力。
- 下一方向选择时变 PRF 慢时间采样与重采样工程契约。
- 新增 `docs/sar_autofocus_phase_truth_followup_decision.md`。

### 阶段状态

- 阶段 78 完成。
- 阶段 79 时变 PRF 慢时间采样与重采样工程契约待启动。

## 2026-06-10 阶段 79 时变 PRF 慢时间采样与重采样工程契约

### 已执行

1. 新增 `SAR_VARIABLE_PRF_RESAMPLING_CONTRACT.md`。
2. 冻结首尾跨度定义的名义均匀慢时间轴与 PRF。
3. 冻结间隔偏差、时间轴偏差和均匀容差诊断。
4. 冻结支持区内复数线性重采样与仿射解析真值。
5. 明确非均匀生产成像、NUFFT 和 Session 接入继续后置。

### 阶段状态

- 阶段 79 契约完成。
- 阶段 80 时变 PRF 慢时间重采样基础实现待启动。

## 2026-06-10 阶段 80 时变 PRF 慢时间重采样基础实现

### 已执行

1. 新增内部 `SarSlowTimeResampling`，生成名义均匀慢时间轴。
2. 实现间隔/时间轴非均匀度诊断与复数线性重采样。
3. 保持首尾样本，拒绝重复、逆序和非有限输入。
4. 新增 `docs/sar_slow_time_resampling_acceptance_report.md`。

### 验证结果

- 默认与 Eigen 3.3.9 定向测试各 `4/4` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

### 阶段状态

- 阶段 80 完成当前平台审批。
- 阶段 81 时变 PRF 重采样后续接入决策门待启动。

## 2026-06-10 阶段 81 时变 PRF 重采样后续接入决策门

### 决策结论

- 缺少带限回波误差阈值、缺失脉冲策略和 RDA/GBP 独立质量矩阵。
- RDA、Session 和 public 接入继续后置。
- 下一方向选择二维 raw-history 慢时间重采样契约。
- 新增 `docs/sar_slow_time_resampling_followup_decision.md`。

### 阶段状态

- 阶段 81 完成。
- 阶段 82 二维 raw-history 慢时间重采样契约待启动。

## 2026-06-10 阶段 82 二维 raw-history 慢时间重采样契约

### 已执行

1. 新增 `SAR_RAW_HISTORY_SLOW_TIME_RESAMPLING_CONTRACT.md`。
2. 冻结矩阵轴、共享慢时间映射、首尾行保持和列独立性。
3. 冻结多列复数仿射解析真值与非法矩阵拒绝语义。
4. 明确 RDA 接入和带限回波质量继续后置。

### 阶段状态

- 阶段 82 契约完成。
- 阶段 83 二维 raw-history 慢时间重采样实现待启动。

## 2026-06-10 阶段 83 二维 raw-history 慢时间重采样实现

### 已执行

1. 在 `SarSlowTimeResampling` 增加二维 `ComplexMatrix` 重采样入口。
2. 保持矩阵轴、尺寸、首尾行和列独立性。
3. 新增 `docs/sar_raw_history_slow_time_resampling_acceptance_report.md`。

### 验证结果

- 默认与 Eigen 3.3.9 定向测试各 `3/3` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。

### 阶段状态

- 阶段 83 完成当前平台审批。
- 阶段 84 二维慢时间重采样后续接入决策门待启动。

## 2026-06-10 阶段 84 二维慢时间重采样后续接入决策门

### 决策结论

- 当前具备参考场景、RDA/GBP 和图像比较指标，可建立质量矩阵。
- 当前没有抖动幅度、raw-history 误差和聚焦质量通过区证据。
- RDA、Session 和 public 接入继续后置。
- 下一方向选择时变 PRF 重采样质量参考矩阵契约。
- 新增 `docs/sar_raw_history_resampling_followup_decision.md`。

### 阶段状态

- 阶段 84 完成。
- 阶段 85 时变 PRF 重采样质量参考矩阵契约待启动。

## 2026-06-10 阶段 85 时变 PRF 重采样质量参考矩阵契约

### 已执行

1. 新增 `SAR_VARIABLE_PRF_RESAMPLING_QUALITY_MATRIX_CONTRACT.md`。
2. 冻结首尾固定的确定性正弦抖动与严格递增约束。
3. 冻结 raw-history NRMS、RDA 图像指标和时间轴诊断。
4. 冻结零、小、中、大抖动矩阵与趋势验收。

### 阶段状态

- 阶段 85 契约完成。
- 阶段 86 时变 PRF 重采样质量参考矩阵实现待启动。

## 2026-06-10 阶段 86 时变 PRF 重采样质量参考矩阵实现

### 已执行

1. 新增确定性抖动参考矩阵，重建实际轨迹 raw-history 并执行二维重采样。
2. 记录 raw-history NRMS 与重采样后 RDA 图像 NRMS/相关趋势。
3. 新增 `docs/sar_variable_prf_resampling_quality_matrix_acceptance_report.md`。

### 测量结果

- `A=0.05`：raw NRMS `0.000164`，图像 NRMS `0.000090`。
- `A=0.15`：raw NRMS `0.000463`，图像 NRMS `0.000254`。
- `A=0.35`：raw NRMS `0.000947`，图像 NRMS `0.000526`。

### 验证结果

- 默认与 Eigen 3.3.9 定向矩阵各 `1/1` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。

### 阶段状态

- 阶段 86 完成当前平台审批。
- 阶段 87 时变 PRF 质量矩阵后续决策门待启动。

## 2026-06-10 阶段 87 时变 PRF 质量矩阵后续决策门

### 决策结论

- 当前单点矩阵不足以冻结通用阈值或批准 RDA 接入。
- 缺失脉冲、多目标、高 Doppler 和随机抖动仍无证据。
- RDA、Session 和 public 接入继续后置。
- 下一方向选择缺失脉冲与最大慢时间间隙诊断契约。
- 新增 `docs/sar_variable_prf_quality_followup_decision.md`。

### 阶段状态

- 阶段 87 完成。
- 阶段 88 缺失脉冲与最大慢时间间隙诊断契约待启动。

## 2026-06-11 阶段 88 缺失脉冲与最大慢时间间隙诊断契约

### 已执行

1. 新增 `SAR_MISSING_PULSE_GAP_DIAGNOSTICS_CONTRACT.md`。
2. 冻结显式 expected interval 和 `1.5x` 最大间隙门。
3. 冻结疑似缺失计数、首个拒绝索引和重采样允许/拒绝语义。
4. 冻结均匀、小抖动、边界、单/多缺失验收矩阵。

### 阶段状态

- 阶段 88 契约完成。
- 阶段 89 缺失脉冲与最大慢时间间隙诊断实现待启动。

## 2026-06-11 阶段 89 缺失脉冲与最大慢时间间隙诊断实现

### 已执行

1. 新增独立间隙诊断与结构化缺失计数。
2. 新增门禁二维慢时间重采样包装入口。
3. 保持原有无门禁基础函数行为不变。
4. 新增 `docs/sar_missing_pulse_gap_diagnostics_acceptance_report.md`。

### 验证结果

- 默认与 Eigen 3.3.9 定向测试各 `5/5` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。

### 阶段状态

- 阶段 89 完成当前平台审批。
- 阶段 90 缺失脉冲诊断后续接入决策门待启动。

## 2026-06-11 阶段 90 缺失脉冲诊断后续接入决策门

### 决策结论

- 参考场景 `prf_hz` 可作为明确 expected interval 真值。
- 内部参考质量链可接入门禁包装入口。
- Session 缺少显式时变 PRF 请求语义，生产接入继续后置。
- 下一方向选择缺失脉冲拒绝参考矩阵契约。
- 新增 `docs/sar_missing_pulse_gap_followup_decision.md`。

### 阶段状态

- 阶段 90 完成。
- 阶段 91 缺失脉冲拒绝参考矩阵契约待启动。

## 2026-06-11 阶段 91 缺失脉冲拒绝参考矩阵契约

### 已执行

1. 新增 `SAR_MISSING_PULSE_REJECTION_MATRIX_CONTRACT.md`。
2. 冻结允许、边界、单缺失和多缺失参考矩阵。
3. 冻结允许 case 的重采样/RDA 完成语义。
4. 冻结拒绝 case 输出为空且 RDA 未尝试的停止语义。

### 阶段状态

- 阶段 91 契约完成。
- 阶段 92 缺失脉冲拒绝参考矩阵实现待启动。

## 2026-06-11 阶段 92 缺失脉冲拒绝参考矩阵实现

### 已执行

1. 新增允许/边界/单缺失/多缺失参考矩阵。
2. 验证允许 case 可完成重采样与 RDA。
3. 验证拒绝 case 输出为空且 RDA 未尝试。
4. 新增 `docs/sar_missing_pulse_rejection_matrix_acceptance_report.md`。

### 验证结果

- 默认定向测试：`3/3` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_unit` 与 `sar_cxx11_compat` 通过。

### 阶段状态

- 阶段 92 完成当前平台审批。
- 阶段 93 缺失脉冲拒绝矩阵后续决策门待启动。

## 2026-06-11 阶段 93 缺失脉冲拒绝矩阵后续决策门

### 决策结论

- Session/public 缺少显式时变 PRF 请求和失败传播契约。
- RDA 默认路径接入与缺失修复、稀疏恢复、NUFFT 继续后置。
- 下一方向选择内部慢时间重采样请求与执行边界契约。
- 新增 `docs/sar_missing_pulse_rejection_followup_decision.md`。

### 阶段状态

- 阶段 93 完成。
- 阶段 94 内部慢时间重采样请求与执行边界契约待启动。

## 2026-06-11 阶段 94 内部慢时间重采样请求与执行边界契约

### 已执行

1. 新增 `SAR_INTERNAL_SLOW_TIME_RESAMPLING_REQUEST_CONTRACT.md`。
2. 冻结显式请求输入、结构化状态/拒绝原因和无回退语义。
3. 冻结输出原子性、输入不变和确定性。
4. 明确执行器不调用 RDA、不接入 Session/public 表面。

### 阶段状态

- 阶段 94 契约完成。
- 阶段 95 内部慢时间重采样请求执行器实现待启动。

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
