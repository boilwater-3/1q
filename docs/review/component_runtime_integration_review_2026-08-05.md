---
Status: draft
Date: 2026-08-05
Review-Baseline: `feature/component-attachment-ecs` @ `42273194`（含两次集成全部改动）
Authority: 非规范性审查记录；不得替代 `docs/common/contract.md`、
`docs/common/session_contract.md` 及各模块 `docs/<module>/design.md`。
若本文与库实现冲突，以库为准。
---

# 组件运行时修改接口集成审查：反开发者集成逻辑清单

## 0. 定位与结论

本文审计两次集成的**集成者体验**：库是否在"提供功能"的同时，把本应由库承担的
构造、接线、适配工作留给了开发者手动补充。

- **集成①**：`examples/component_attachment` 组件运行时修改接口（`feat(examples)` +
  `test(examples)`，c0bc1466 / be3a2bef）——为 AR/ESR/EOS/FD 组件定义库运行时修改
  设计的调用入口，并补组件层测试。
- **集成②**：库层测试缺口修复（`test(sar)` / `test(flight_dynamic)`，
  42846cbe / 42273194）——SAR session 层边缘路径测试 + `FlightManager::ClearManeuvers`
  库层测试。

结论：两次集成共发现 **1 项库能力缺口、4 项工具/接线缺省、3 项设计如此、1 项验证后
排除**。全部为**报告项，本轮不修改任何库逻辑**；其中能力缺口与工具缺省项可另立任务
补库 API（见 §5 优先级）。

> 后续状态（2026-08-06）：C-3（SAR/SBIRS 无组件类）已由后续集成解决——见
> §2.3 表内注记；其余报告项（A-1/B-1~B-4）状态不变。

## 1. 判定方法

审查以实际 include 目录、库头文件与构建脚本为证据，不以设计文档声明代替。分类：

- **A：库能力缺口**——库未提供该能力，开发者无法自行补齐（必须先补库 API）；
- **B：工具/接线缺省**——库有功能，但构造工具、共享适配或构建接线缺失，
  开发者须手动重复；
- **C：设计如此**——有意的边界决策或模块特化，外部集成者须知悉（不构成缺陷）；
- **D：怀疑不成立**——假设的缺口经验证不存在。

## 2. 发现

### 2.1 A 类：库能力缺口（1 项）

| 编号 | 发现 | 证据 |
|---|---|---|
| A-1 | **Fusion 无运行时修改 API**。`include/1q/fusion/` 仅有 DetectionRecord.h / FusedTarget.h / FusionConfig.h / FusionEngine.h / fusion.hpp，无 `FusionRuntimeConfigPatch`/Builder；`FusionEngine` 只在构造时接受 `FusionConfig`，运行期参数不可变。组件集成因此无法定义运行时接口（FusionComponent 类注释与 `examples/component_attachment/README.md` 已注明"须先补库 API"）。 | `include/1q/fusion/`；`examples/component_attachment/components/fusion_component.h` |

### 2.2 B 类：工具/接线缺省（4 项）

| 编号 | 发现 | 证据 |
|---|---|---|
| B-1 | **FD 机动指令无 builder（builder 不对称）**。5 个传感器模块（AR/ESR/EOS/SAR/SBIRS）的运行时修改均有链式 `XxxRuntimeConfigBuilder`，而 `ManeuverCommand` 为裸 struct，且字段语义**高度重载**：`value`/`duration_sec`/`heading_tolerance_rad`/`altitude_tolerance_m` 随机动类型含义完全不同（如 `kSTurn` 的 `heading_tolerance_rad` 是摆幅°，`altitude_tolerance_m` 是周期 s；`kRacetrack`/`kFigure8` 各有重载）。两次集成中所有构造均为手动逐字段赋值，须靠读注释记忆字段约定。 | `include/1q/flight_dynamic/FlightManager.h:60-97`；`include/1q/flight_dynamic/guidance/` 无 Builder；对照 `include/1q/<模块>/config/*RuntimeConfigBuilder.h` |
| B-2 | **传感器输出 → `DetectionRecord` 的融合边界适配不在库内**。`FusionEngine` 只接受 `fusion::DetectionRecord`，各传感器 Session 只输出自有格式；"去真值化 + 质量基准 + 源通道映射"整段业务边界适配由示例层 header-only 文件 `examples/common/sensor_adapt.h`（125 行）承载，供 component_entt 与 component_attachment 两个示例共用（注记 2026-08-10：component_entt 已删除，现仅 component_attachment 消费）。第三方集成多传感器融合时须自行编写或从 examples 复制。注：其文件头注释表明**有意**置于示例层（业务层边界适配），属边界设计决策，但对集成者效果相同。 | `examples/common/sensor_adapt.h:1-13`；`include/1q/fusion/FusionEngine.h` |
| B-3 | **FD 消费方 CMake 接线无封装**。静态库不传递依赖：每个消费 FD 的 target 须手动复制"链接 `JSBSim::JSBSim` + 注入 `FD_JSBSIM_ROOT_DIR` + include `third_party/jsbsim/src`"。`FlightDynamicPartitions.cmake` 的 foreach 接线模式在 `Unit.cmake` examples 分区被逐行复制；库/构建系统未提供带 usage requirements 的封装 target 或 `oneq_link_flight_dynamic(target)` 函数。新增消费方（测试分区、示例、外部服务）均须手工重复。 | `tests/cmake/FlightDynamicPartitions.cmake:26-39`；`tests/cmake/partitions/Unit.cmake` examples 分区 FD 门控块 |
| B-4 | **测试基建重复**。SAR 的 `MakeSmallRdaConfig()`/`MakeInput()` 在 5 个测试文件独立声明（session_pipeline / controller_runtime_state / cycle_input_adapter_bridge / output_boundary_contract / 本次新增 session_runtime_config）；FD 的 `FlightDynamicTest` fixture 在 fd_adapter 与 fd_robustness 重复（`fd_test_helpers.h` 仅共享 RunSteps/RunUntilDone/ExpectNoNaN）。库层无共享测试 helper 基建，新测试文件靠复制。 | `tests/unit/sar/sar_session_pipeline_test.cpp:34-87` 及上述 5 文件；`tests/unit/flight_dynamic/fd_adapter_test.cpp:25-46`、`fd_robustness_test.cpp:15-37` |

### 2.3 C 类：设计如此，外部须知悉（3 项）

| 编号 | 发现 | 证据 |
|---|---|---|
| C-1 | **组件层 bool 返回是自创信号**。库的 `FlightManager::ClearManeuvers`/`Abort` 返回 void；`FlightComponent` 包装自创"FD 可用性"语义（FD 未启用/初始化失败返回 false）。库侧无直接可用性查询（须由 `GetState()==kAborted` 推导）。 | `include/1q/flight_dynamic/FlightManager.h:168`；`examples/component_attachment/components/flight_component.cpp:245-248` |
| C-2 | **ESR 独有结构化拒绝结果**。仅 ESR 提供 `ApplyRuntimeConfigWithResult`（拒绝原因枚举 `kRejectedInvalidScanRate` 等）；AR/EOS/SAR/SBIRS 只返回 bool，拒绝原因仅在库日志。外部做"按原因分支"决策在非 ESR 模块无结构化入口。 | `include/1q/electronic_surveillance_radar/session/EsrSession.h`（`EsrRuntimeConfigApplyResult` 定义处；经 `esr_sensor_component.h` 暴露） |
| C-3 | **SAR/SBIRS 无组件类（已解决）**。审查时组件集成范围仅覆盖 AR/ESR/EOS/FD/Fusion 五组件。2026-08-06 后续集成补齐：`SbirsSensorComponent`（复刻 EOS 形态——`SbirsSession` + `SbirsDetectionLifecycleRecorder` + `TryApplyRuntimeConfig`，探测适配为融合第 4 源通道）与 `SarSensorComponent`（`SarSession` + `SarProductLifecycleRecorder`，产品生命周期事件经信号发布，无探测输出不入融合，符合 Bahavior.md 契约）。 | `examples/component_attachment/components/{sbirs_sensor,sar_sensor}_component.{h,cpp}` |

### 2.4 D 类：怀疑不成立（1 项）

| 编号 | 发现 | 证据 |
|---|---|---|
| D-1 | **Patch builder 对称性怀疑不成立**。曾怀疑 AR/ESR/EOS 缺 builder，验证后确认 5 个传感器模块全部具备 `XxxRuntimeConfigBuilder.h` + `XxxRuntimeConfigPatch.h` + `XxxSessionConfigBuilder.h`，传感器侧补丁构造无缺口。 | `include/1q/{airborne_radar,electronic_surveillance_radar,electro_optical_sensor,sar,sbirs_sensor}/config/` |

## 3. 建议优先级（报告项，未实施）

| 优先级 | 项 | 性质 | 建议动作 |
|---|---|---|---|
| 高 | B-1 `ManeuverCommandBuilder` | 工具缺省，字段重载最需防错 | 补库 API：链式 builder 对标 `XxxRuntimeConfigBuilder`；随后组件层/测试改用 |
| 高 | A-1 Fusion 运行时修改 API | 库能力缺口 | 补库 API（如 `FusionEngine::TryApplyRuntimeConfig`）；FusionComponent 解除"不可变"注明 |
| 中 | B-2 sensor_adapt 升格评估 | 边界设计决策 | 评估提炼为库工具的边界（业务适配是否属库职责），或保持示例层并记录 |
| 中 | B-3 FD CMake 接线封装 | 构建接线重复 | 提供带 usage requirements 的封装（如 `oneq_link_flight_dynamic(target)`） |
| 低 | B-4 测试 helper 共享基建 | 纯集成成本 | 抽共享 helper 头（SAR config/input、FD fixture），不改变既有测试语义 |

## 4. 结论

两次集成本身未发现库行为缺陷；`ClearManeuvers`/`TryApplyRuntimeConfig` 等既有 API 语义
与测试断言一致。全部发现集中于**集成面**：库在"能力"上基本齐备，但在"构造工具
（B-1）、业务适配（B-2）、构建接线（B-3）、测试基建（B-4）"四个层面存在需开发者
手动补充的缺省，另有 Fusion 一项真实能力缺口（A-1）。按 §3 优先级另立任务，本轮
**不修改库逻辑**。
