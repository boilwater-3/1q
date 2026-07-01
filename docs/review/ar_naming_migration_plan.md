# AR 命名迁移计划

Status: draft
Last-reviewed: 2026-07-01
Authority: proposed migration plan for airborne_radar naming cleanup

本文冻结 `airborne_radar` 模块从 `Radar*` 命名迁移到 `Ar*` 命名的分批计划。目标不是改变运行时语义，而是让 AR 的 public API 与 `Sar*`、`Esr*`、`Eos*` 的模块前缀规则一致，避免 `Radar*` 被误读为跨 radar-family 的通用基础层。

## 0. 前置约束

当前 `docs/common/contract.md` 和 `tests/contract/check_docs_structure.cmake` 允许 `docs/review` 作为扁平草案目录，但草案不得作为当前权威文档引用。进入生产迁移前必须二选一：

1. 保留本文件作为短期迁移草案，并在每个实现 stage 后同步更新。
2. 将本计划的最终结论并入 `docs/airborne_radar/design.md`，跨模块规则并入 `docs/common/contract.md` 或 `docs/common/open_questions.md`，然后删除本草案。

除文档落点外，迁移必须遵守现有 public API 边界：AR 唯一用户自定义 SPI 仍是 `ITacticalDecisionEngine`，不得借命名迁移重新公开 pipeline、controller、environment service、mutable context 或 generated replay headers。

## 1. 命名决策

采用 `Ar*` 作为 `airborne_radar` 模块所有权前缀。

必须改名的是“模块归属前缀”，不是所有雷达领域词。判断规则：

- 类型代表 AR 模块 public DTO、session、config、builder、validation、adapter、trace/replay、debug view、lifecycle recorder 时，`Radar*` 必须迁移到 `Ar*`。
- 类型位于 public header，即使当前名称在业务上可解释为雷达，也优先迁移到 `Ar*`，保证外部调用面一致。
- internal 类型如果承担模块装配、session 编排或上下文所有权，也应迁移到 `Ar*`，但可以晚于 public API。
- 纯领域算法、物理公式、字段语义或传感器术语可以保留 `Radar` / `radar`，避免把领域概念改成缩写。

## 2. 必须改名

Public config 主类型：

| 现名 | 目标名 |
|---|---|
| `RadarHardwareConfig` | `ArHardwareConfig` |
| `RadarMissionConfig` | `ArMissionConfig` |
| `RadarPolicyConfig` | `ArPolicyConfig` |
| `RadarEnvironmentConfig` | `ArEnvironmentConfig` |
| `RadarOrientationConfig` | `ArOrientationConfig` |
| `RadarSessionConfig` | `ArSessionConfig` |
| `RadarRuntimeConfigPatch` | `ArRuntimeConfigPatch` |
| `RadarRuntimeConfigBuilder` | `ArRuntimeConfigBuilder` |
| `RadarSessionConfigBuilder` | `ArSessionConfigBuilder` |
| `RadarSessionConfigValidation` naming surface | `ArSessionConfigValidation` naming surface |
| `RadarWorkMode` / `RadarWorkSubMode` | `ArWorkMode` / `ArWorkSubMode` |

Public session 主类型：

| 现名 | 目标名 |
|---|---|
| `RadarSession` | `ArSession` |
| `RadarCycleInput` | `ArCycleInput` |
| `RadarCycleResult` | `ArCycleResult` |
| `RadarCycleInputAdapter` | `ArCycleInputAdapter` |
| `RadarCycleOutputAdapter` | `ArCycleOutputAdapter` |
| `RadarEnvironmentInput` | `ArEnvironmentInput` |
| `RadarExternalInputAdapter` | `ArExternalInputAdapter` |
| `RadarExternalOutputAdapter` | `ArExternalOutputAdapter` |
| `RadarInputValidation` naming surface | `ArInputValidation` naming surface |
| `RadarSceneTarget` / `RadarSceneTargetList` | `ArSceneTarget` / `ArSceneTargetList` |
| `RadarOutputTypes` naming surface | `ArOutputTypes` naming surface |
| `RadarReplaySession` / `RadarReplaySessionResult` | `ArReplaySession` / `ArReplaySessionResult` |
| `RadarTraceSession` / `RadarTraceSessionOptions` | `ArTraceSession` / `ArTraceSessionOptions` |
| `RadarTrackLifecycleRecorder` and event types | `ArTrackLifecycleRecorder` and event types |
| `RadarTrackOutputDebugView` and debug state/status types | `ArTrackOutputDebugView` and debug state/status types |
| `RadarCommand` / `RadarCommandType` / `RadarCommandSource` | `ArCommand` / `ArCommandType` / `ArCommandSource` |
| `RadarControlProfile` | `ArControlProfile` |

Internal ownership and orchestration types:

| 现名 | 目标名 |
|---|---|
| `RadarController` | `ArController` |
| `MutableRadarContext` | `MutableArContext` |
| `RadarSessionCompositionRoot` / `RadarSessionComposition` | `ArSessionCompositionRoot` / `ArSessionComposition` |
| `RadarReplayFlatbufferCodec` | `ArReplayFlatbufferCodec` |
| `RadarSceneTargetUtils` | `ArSceneTargetUtils` |
| `RadarTrackOutputDebugViewBuilder` | `ArTrackOutputDebugViewBuilder` |
| `RadarOrientationUtils` | `ArOrientationUtils` if it stays AR-specific |

Files and public include paths should follow type names. Example:

```text
include/1q/airborne_radar/session/RadarSession.h
  -> include/1q/airborne_radar/session/ArSession.h

include/1q/airborne_radar/config/RadarSessionConfig.h
  -> include/1q/airborne_radar/config/ArSessionConfig.h
```

## 3. 保留领域名

以下命名不纳入机械迁移，除非后续证据证明它们已经承担模块所有权前缀：

- `airborne_radar` namespace 和目录名：它是模块标识，不改成 `ar`。
- `radar_cross_section_dbsm`、`rcs`、`radar_mount_angles_deg` 等物理或输入字段：这是领域语义。
- `RadarEquations`：雷达方程是领域算法名，可保留；若未来抽到 common radar physics 层，再单独契约化。
- 文档中的中文“机载雷达”“雷达探测”“雷达方程”等自然语言。
- FlatBuffers schema 文件名 `airborne_radar_replay.fbs` / `airborne_radar_session_replay.fbs`：文件名已经带模块名。schema 内 table/type 名是否改为 `Ar*` 需要单独评估 replay 兼容策略。
- trace/replay payload string 的历史值，如 `"RadarCycleInput"`：不能无迁移策略直接改，否则旧 trace 读取会断。

## 4. 兼容期守护

迁移分两个 public API 层：

1. primary API：新增 `Ar*` 头文件和类型，aggregate header 优先暴露 `Ar*`。
2. deprecated compat API：旧 `Radar*` 头文件保留为薄 wrapper，内部 include 新头并提供 `using RadarSession = ArSession;` 这类别名。

兼容期规则：

- 旧 `Radar*` public 头不得继续承载实现，只能转发到 `Ar*` 主头。
- 旧 `Radar*` 类型别名必须与新 `Ar*` 类型 ABI 等价，不新增适配对象、不复制状态。
- compat 别名集中放在同一 namespace 下，避免产生 `radar` 子 namespace。
- 新代码、文档、consumer tests、示例和 `airborne_radar.hpp` 必须使用 `Ar*`。
- 旧名只允许出现在 compat headers、compat consumer test、历史 trace/replay payload 映射和迁移说明中。
- 如果启用 deprecated attribute，先只作用于 direct include 的旧 wrapper，不作用于 using alias 本身，避免旧 consumer 在兼容期被警告淹没。

需要新增或调整的 guard：

| Guard | 责任 |
|---|---|
| `public_api_boundary_guard` | whitelist 同时区分 `AR_PUBLIC_PRIMARY_HEADERS` 与 `AR_DEPRECATED_COMPAT_HEADERS` |
| `install_manifest_guard` | 安装清单必须包含新 primary 头；旧 wrapper 只在兼容期存在 |
| `cross_domain_naming_guard` | 禁止新 public primary header 引入 `Radar*` 模块前缀 |
| `airborne_include_style_guard` | internal include 优先包含 `Ar*` 主头 |
| `doc_legacy_term_guard` 或新 AR naming guard | 规范性文档不得把 `Radar*` 当推荐 API |
| replay/trace roundtrip tests | 验证旧 payload string 仍可读，新 payload string 若引入也可读 |

## 5. 分批实施

Stage 0：冻结契约和文档落点。

- 状态：已完成。
- `docs/review` 草案目录已纳入 `docs_structure_guard`，仅允许扁平 `Status: draft` Markdown。
- `docs/airborne_radar/design.md` 已记录 `Ar*` 作为推荐 public API 前缀。
- `docs/common/contract.md` 已明确模块 public 类型前缀规则：`Ar*`、`Eos*`、`Esr*`、`Sar*`。

Stage 1：新增 `Ar*` public primary API，不删除旧名。

- 状态：部分完成。
- 已新增 `include/1q/airborne_radar/config/Ar*.h` 和 `session/Ar*.h`，通过 alias/inline forwarding 提供 `Ar*` public 入口。
- 已更新 `airborne_radar.hpp`、`airborne_radar_config.hpp` 优先 include `Ar*` 头。
- 已新增 `ArPrimaryNamingContractTest` 覆盖 `ArSessionConfigBuilder`、`ArSession`、`ArCycleInput`、`ValidateArCycleInput`、`ArRuntimeConfigBuilder`。
- 已更新 `ar_session_consumer.cpp` 和 `ar_extension_consumer.cpp` 使用 `Ar*`，并新增 `ar_compat_consumer.cpp` 证明旧 `Radar*` 仍可编译。
- 未完成项：旧 `Radar*.h` 仍是实现承载头，尚未反转为 wrapper；这留给 Stage 2/4 随内部文件迁移一起处理。

Stage 2：更新 internal session/config/runtime 命名。

- 迁移 `src/airborne_radar/session/RadarSession.cpp`、`RadarSessionCompositionRoot`、`MutableRadarContext`、adapter、debug builder、replay codec。
- 迁移 `src/airborne_radar/runtime/RadarController.*`。
- 更新 internal tests 中直接引用的 ownership 类型。
- 不改 `RadarEquations` 和领域字段。

Stage 3：trace/replay schema 与 payload 兼容。

- 评估 FlatBuffers table/type 是否改名。若改名，decoder 必须同时识别旧 `Radar*` 和新 `Ar*` payload type。
- 旧 trace 文件的 payload string 必须继续通过 `ReplayRadarTrace`。
- generated headers 只通过项目固定 `flatc` 重新生成，不手写。

Stage 4：收口旧名。

- 将 `Radar*` wrapper 移入 deprecated compat whitelist。
- 新增 guard 禁止 primary API、规范性文档、示例和新测试继续使用旧模块前缀。
- 在 open question 或 contract 中写明旧名移除条件：至少一个发布/集成窗口后，且外部 consumer 无旧名依赖。

## 6. 验收测试

每个 stage 的最低验证：

```bash
ctest --preset llvm-ninja-release -R "public_api_boundary_guard|install_manifest_guard|cross_domain_naming_guard|airborne_include_style_guard|airborne_include_direction_guard|public_header_cxx11_guard" --output-on-failure
```

Public API 和 consumer 验证：

```bash
ctest --preset llvm-ninja-release -R "1q_contract_tests|public_api_boundary_guard|install_manifest_guard" --output-on-failure
```

AR focused runtime 验证：

```bash
build/llvm-ninja-release-local/bin/1q_unit_tests --gtest_filter="RadarSessionConfigBuilderTest.*:RadarSessionCreateWithValidationTest.*:RadarInputValidationTest.*:TraceSessionAdapterTest.*:RadarReplayCodecRoundtripTest.*:RadarRuntimePatchMapperTest.*"
```

AR integration 验证：

```bash
ctest --preset llvm-ninja-release -R "ar_session|airborne_radar_scene" --output-on-failure
```

文档结构验证：

```bash
ctest --preset llvm-ninja-release -R "docs_structure_guard|doc_legacy_term_guard" --output-on-failure
```

验收标准：

- 新 `Ar*` public consumer 能编译链接并覆盖 session、runtime patch、decision SPI。
- 旧 `Radar*` compat consumer 能编译链接，且只依赖 wrapper/alias。
- `airborne_radar.hpp` 和 `airborne_radar_config.hpp` 推荐路径不再 include 旧主头。
- 旧 trace/replay payload 仍可解码；如果引入新 payload string，新旧 roundtrip 都通过。
- `Radar*` 在 public primary API、规范性文档和新示例中只出现在兼容说明或领域术语白名单内。
- 运行时行为不变：AR config commit/rollback、environment presence、decision SPI、trace/replay 语义不因命名迁移改变。

## 7. 非目标

- 不更改 `airborne_radar` namespace。
- 不调整 AR 决策 SPI 的能力边界。
- 不公开 internal pipeline/controller/environment service 作为新扩展点。
- 不把 `RadarEquations` 抽到 common 层。
- 不借迁移重写 FlatBuffers schema 语义或 replay 文件格式。
- 不通过删除旧头来制造“干净”命名；兼容期必须可验证。
