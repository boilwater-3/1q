# SAR Phase 0 现状审计报告

> **历史快照（historical snapshot, not current API）。** 本审计记录 2026-06-23 当时的实现状态，其中提到的 `SarSessionFactory`（friend 模式）已在后续收口中移除——创建入口现为 `SarSession::Create` / `SarSession::CreateWithValidation` 静态成员。审计结论作为历史记录保留，不代表当前 API。

Date: 2026-06-23

## 1. 审计目的

`module_design.md` 把 Phase 0(会话/配置/trace-replay 契约 + CMake 安装清单)标为
`✅ 完成(公共 API 冻结)`。本审计逐条比对该描述与 `include/1q/sar/`、`src/sar/session/`、
`src/sar/SarSources.cmake`、`src/sar/CMakeLists.txt`、`tests/contract/` 的实际实现,
识别过时/不准确标记。本审计不修改代码。

## 2. 审计结论

Phase 0 **描述基本准确,公共 API 确已冻结且完整**。发现 4 处轻微不一致(均为注释或
措辞层面,不影响契约正确性),无实质性契约漂移。

| 审计项 | 实际状态 | 结论 |
|---|---|---|
| 公共会话契约(Session/Config/Input/Result/Patch) | 全部存在且 ONEQ_API 导出 | ✅ 一致 |
| sar.hpp 聚合 | 聚合 config + 全部 6 个 session 头 | ✅ 一致 |
| SarSession PIMPL + 私有构造 + friend SarSessionFactory | 实现(`SarSession.h:57-61`) | ✅ 一致 |
| SarFocusedImage 实/虚双 vector\<double\> 行主序 | `SarCycleResult.h:78-85` | ✅ 一致 |
| SarRuntimeConfigPatch has_* + 值, 含 retain_focused_image | `SarRuntimeConfigPatch.h:17-37` | ✅ 一致 |
| Trace/Replay/Factory 超设计交付 | 全部落地 | ✅ 一致 |
| SarSources.cmake 双清单 + frozen 护栏 | 存在且生效 | ✅ 一致 |

## 3. 公共头文件实证

### `include/1q/sar/config/`(7 文件)

| 文件 | 声明 | ONEQ_API |
|---|---|---|
| `SarHardwareConfig.h` | `SarHardwareConfig` | ✅ |
| `SarMissionConfig.h` | `SarWaypointConfig` + `SarWaypointConfigList` + `SarMissionConfig` | ✅ |
| `SarPolicyConfig.h` | `SarPolicyConfig`(含 `enable_l3_bp_imaging` 等布尔开关) | ✅ |
| `SarEnvironmentConfig.h` | `SarEnvironmentConfig` | ✅ |
| `SarRuntimeConfigPatch.h` | `SarRuntimeConfigPatch`(has_* + 值,含 `retain_focused_image`) | ✅ |
| `SarSessionConfig.h` | `SarSessionConfig`(四域聚合) | ✅ |
| `sar_config.hpp` | 聚合上述 6 个 .h | — |

### `include/1q/sar/session/`(6 文件)

| 文件 | 声明 | 关键点 |
|---|---|---|
| `SarSession.h` | `class SarSession` | PIMPL(`struct Impl` + `unique_ptr<Impl>`)、私有构造(`:61`)、`friend SarSessionFactory`(`:58`)、`Step`/`StepWithResult`/`ApplyRuntimeConfig`/`TryApplyRuntimeConfig` |
| `SarSessionFactory.h` | `class SarSessionFactory` | `static SarSession Create(...)` |
| `SarTraceSession.h` | `SarTraceSessionOptions` + `class SarTraceSession` | PIMPL,持有 `shared_ptr<TraceSink>` + `shared_ptr<ReplayTraceWriter>` |
| `SarReplaySession.h` | `SarReplaySessionResult` + `class SarReplaySession` + `ReplaySarTrace()` | `ReplaySarTrace(trace_dir)` 在 `:42`,定义在 `SarReplaySession.cpp:207` |
| `SarCycleInput.h` | `SarPlatformState`(10 字段)、`SarPointTarget`、`SarRawIqFrame`、`SarCycleInput` | `SarRawIqFrame` 行主序 split `i_values`/`q_values` |
| `SarCycleResult.h` | 枚举 5 个 + `SarDiagnosticIssue` + `SarFocusedImage` + `SarOutputFrame` + `SarCycleResult` | `SarFocusedImage` = `real_values`/`imaginary_values` double vector + `is_placeholder` + `source` |

## 4. SarSources.cmake 实证

- `SAR_ENGINE_SOURCES`(18 文件)、`SAR_CORE_SOURCES`(9 文件)、`SAR_CXX11_COMPAT_SOURCES`(engine + 7 session 文件)均与文档描述一致。
- `src/sar/CMakeLists.txt` 仅 `include()` manifest(`:7`),本地不持有源列表。✅ 与文档"src/sar/CMakeLists.txt 仅 include() 该 manifest"一致。
- CMakeLists 内含注释(`:4-6`)明确 CSA/OmegaK/FocusingSelector/RadiometricCalibration"不再纳入构建"。
- `ENABLE_INSTALL` 块列出公共头清单。

## 5. 冻结源合同护栏

`tests/contract/check_sar_frozen_sources.cmake`(698 字节)存在:
- 构造 `ACTIVE_SAR_SOURCES = SAR_ENGINE_SOURCES + SAR_CORE_SOURCES`。
- 定义 4 个冻结 pattern(OmegaK/CSA/FocusingSelector/RadiometricCalibration)。
- 逐个 active source 匹配冻结 pattern,命中则 `FATAL_ERROR "Frozen SAR source entered active build manifest"`。

✅ 与文档"孤儿文件章节补充 sar_frozen_sources 合同护栏"一致。

## 6. Trace/Replay 链实证

- FlatBuffers 生成头:`src/sar/session/generated/sar_replay_generated.h`(32 KB)、`sar_session_replay_generated.h`(42 KB)。
- `SarTraceSession.cpp` 持有 `TraceSink` + `ReplayTraceWriter`(`:114-115`),构造期写 config 事件(`:72-75,88`),逐步写 input/output 事件(`:163-170`),失败写 marker(`:110`)。
- `ReplaySarTrace()` 定义在 `SarReplaySession.cpp:207`。
- ✅ 与文档"整条 trace/replay 链已落地"一致。

## 7. 发现的不一致(均为轻微)

### 不一致 1 — SarTraceSession.h stale doxygen(注释层面)

`SarTraceSession.h:3` 自称"定义 SAR trace 会话**占位门面**",但 `SarTraceSession.cpp`(7965 字节)是完整真实实现(写 TraceSink + ReplayTraceWriter)。"占位"措辞过时。

### 不一致 2 — SarReplaySession.h stale doxygen(注释层面)

`SarReplaySession.h:4` 同样自称"占位门面",但 `SarReplaySession.cpp`(9183 字节) + `ReplaySarTrace()` 是完整实现。"占位"措辞过时。

### 不一致 3 — SarSessionFactory 无独立 .cpp(实现组织层面,非遗漏)

`SarSessionFactory` 仅公共头,无 `SarSessionFactory.cpp`。其 `Create()` 声明在头,定义实际在 `SarSession.cpp` 内(friend 模式)。文档目录树列出 `SarSessionFactory` 未说明此组织,但 CMake manifest 正确不引用任何 `SarSessionFactory.cpp`。非文档错误,仅是实现组织未显式说明。

### 不一致 4 — SAR_CXX11_COMPAT_SOURCES 排除项未在目录树标注

`SAR_CXX11_COMPAT_SOURCES` 故意排除 `SarTraceSession`/`SarReplaySession`/`SarReplayFlatbufferCodec`(C++11 不兼容的 trace/replay 依赖)。文档目录结构章节(L143-146)列出全部 session 文件,但未标注这三个文件被排除出 C++11 兼容构建。轻微,非事实错误。

## 8. 处置建议

- 不一致 1、2(注释层面):**建议修正** `SarTraceSession.h` / `SarReplaySession.h` 顶部 doxygen,删除"占位门面"措辞,改为准确描述。低风险纯注释。
- 不一致 3、4(组织/说明层面):**建议在 `module_design.md` 目录树补一句**说明 SarSessionFactory 由 SarSession.cpp friend 实现、SAR_CXX11_COMPAT_SOURCES 排除 trace/replay 三文件。非必须。

## 9. 本审计的非目标

- 不修改任何 C++ 源代码逻辑。
- 不变更冻结清单或公共 API。
- 不构成 Phase 0 之外的实现授权。
- 不重开孤儿文件恢复审批。
