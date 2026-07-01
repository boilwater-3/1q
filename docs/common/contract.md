# 跨模块契约

Status: active
Last-reviewed: 2026-07-01
Authority: common contract for all modules

本文合并原顶层 public API customization、session config builder、三层输出可观测性和文档治理契约。模块级文档不得与本文冲突。

## 证据优先开发模式

对算法、架构、模块内部优化、输出语义、配置语义和 public API 相关改动，默认采用
`skills/evidence-first-freeze-contract` 定义的证据优先模式。

强制规则：

1. 先判定，再契约，再实现。
2. Stage A 未得到 `pass` 或 `narrow` 判定时，不进入生产代码实现。
3. Stage B 前必须冻结实现契约，明确允许范围、禁止范围、行为边界、验收条件和非目标。
4. 实现只能覆盖已被证据证明的最小边界；不得借机扩大 public API、跨模块抽象、schema、trace/replay 或兼容层。
5. 验收失败时回到证据矩阵重新拆分原因；不得通过放宽阈值、扩大 skip 或弱化测试制造通过。

具体 evidence matrix、契约模板、输出格式和回写要求由 repo skill 维护；公共契约只规定该流程是高风险开发的默认门禁。

## Public API 边界

默认 public API 只允许稳定门面和稳定 DTO：

- 模块聚合入口头。
- `*Session`，包括 `Create` / `CreateWithValidation` 等静态创建入口。
- `*SessionConfig` 四域配置和运行期 patch。
- `*CycleInput`、scene target/emitter/point target 等单周期输入 DTO。
- `*OutputFrame`、`*CycleResult` 等输出和结构化执行结果 DTO。
- trace/replay、debug view、lifecycle recorder 等已经形成外部消费合同的工具。

默认禁止公开：

- pipeline/controller/context/environment service 等内部装配 seam。
- algorithm executor、focusing selector、calibration/focusing/truth oracle 等内部阶段。
- generated replay headers、内部 execution config、测试专用 mock 接口。
- 仅有单一生产实现且没有外部替换需求的虚接口。

唯一允许的用户自定义 SPI 是 `airborne_radar` 的 decision engine。其它模块默认只提供稳定 session 门面。

## SessionConfigBuilder

所有 `*SessionConfigBuilder` 都是 semantic builder：

1. 只表达高层 profile、intent、preset 或语义开关。
2. `Build()` 产生完整 `*SessionConfig`，不写日志、不隐式校验、不产生副作用。
3. 细粒度工程参数由调用方直接编辑 `*SessionConfig` 四域字段。
4. 运行期变更走 `*RuntimeConfigPatch` / `*RuntimeConfigBuilder`。
5. 配置合法性由独立 validator 检查最终 config。

不得重新引入 leaf setter，例如 frame rate、scene center、minimum SNR、atmospheric loss 这类直接字段编辑器。

## 运行期配置提交策略

`*RuntimeConfigPatch` 的提交（commit）与周期内失败回滚（rollback）按 pipeline 的状态空间复杂度分两类。每个 `*Session` 必须显式归属其中一类，且实际行为不得与所属类的承诺冲突。四模块当前归属固定如下——这是对已实现行为的契约化，不是行为变更要求。

| 类别 | 承诺 | 归属模块 |
|---|---|---|
| **事务性提交** | patch 经 resolver 校验；配置延迟到下个周期边界原子落定；commit 或周期执行失败时，对持有跨周期累积状态的子系统做 capture/restore 完整恢复。 | `airborne_radar` |
| **立即提交** | patch 经 resolver 校验；`TryApplyRuntimeConfig` 调用即生效，配置单向落定、不在 session 层回滚。若 pipeline 持有累积状态且执行可能失败，回滚边界由该模块在内部层（如 controller）声明，不上升为 session 层契约。 | `electronic_surveillance_radar`、`electro_optical_sensor`、`sar` |

规则：

1. **归属由状态空间决定，不由风格偏好决定。** 仅当 pipeline 同时满足"有跨周期累积状态"且"commit/执行存在真实失败路径"时，才采用事务性提交。两者缺一即为立即提交。
   - `airborne_radar`：4 个子系统各有独立 runtime state，`UpdateConfig`/`UpdateExecutionConfig` 可返回 false，故需事务对齐（`RadarSession.cpp:117` CommitPendingRuntimeConfig、`:185-197` capture/restore、`:167-176` 成功后才 FinalizePendingRuntimeConfig）。
   - `electronic_surveillance_radar`：config 无累积（每 RunCycle 重新派生），`UpdateConfig` 走换 config 留 tracks（`InterceptPipeline.cpp:52-57`）；`InterceptPipelineResult` 是三通道纯数据载体，不含 pipeline 自报执行状态，因此当前无 pipeline 执行失败 abort 路径。
   - `electro_optical_sensor`：执行回滚封装在 `EosController::RunOnce`（`EosController.cpp:68-111`），不上升为 session 层事务。
   - `sar`：每 Step 从 `runtime_config` 全量重建，无累积状态可回滚；以执行前 gate（`ValidateRuntimeConfigForStep`）兜底。
2. **所有四模块的 patch 必须经 resolver 校验**（`is_valid`/`has_requested_update`），不得盲写。`sar` 已通过 `SarRuntimeConfigResolver` 对齐该规则。
3. **立即提交类不得声称 session 层回滚。** 若其内部存在 capture/restore 能力（如 ESR 的累积状态快照），必须在代码 doc 注明该机制的实际边界，避免阅读者误以为 session 层提供配置回滚或已激活的执行失败回滚。
4. **事务性提交类不得在执行成功前落定配置语义状态。** 配置的"逻辑当前值"（如 AR 的 `runtime_state`）与"已推送到子系统的物理状态"必须在对齐点之后才一致。

## 三层输出模型

所有传感器/产品模块遵守三层输出模型：

| 层级 | 入口 | 责任 |
|---|---|---|
| 原始系统输出层 | `Step()` 返回的 `*OutputFrame` | 真实传感器或产品输出 |
| 结构化执行结果层 | `StepWithResult()` 返回的 `*CycleResult` | 输出帧、执行状态、校验、abort reason 和诊断摘要 |
| 开发调试视图层 | `*OutputDebugViewBuilder` / `*LifecycleRecorder` | 人读状态、生命周期事件、输入实体回填 |

规则：

- `Step()` 只返回主系统输出帧。
- `StepWithResult()` 是状态判断入口。
- 日志只用于人读运行信息，状态判断不得依赖解析日志文本。
- 数值 ID 是稳定关联键，名称只用于人读、trace/replay、报告和调试视图。
- 仿真真值不得混入面向外部系统的真实输出通道。

## 文档结构

`docs/` 只允许以下一级目录：

- `common`
- `airborne_radar`
- `electro_optical_sensor`
- `electronic_surveillance_radar`
- `flight_dynamic`
- `sar`

每个业务模块只保留 `design.md` 作为设计权威文档。历史决策记录（旧版 `decisions.md`、`history.md`、`contract.md`）和模块入口（`README.md`）的内容已内聚到 `design.md` 中。

`common/` 只允许保留两份文档：

- `contract.md` —— 公共契约（规定性：所有模块必须遵守的规则）。
- `open_questions.md` —— 跨模块架构观察与待决项（非规定性：记录调查中发现但尚未定论的议题，不构成契约约束）。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从 open_questions.md 移除。

模块目录内不保留 `archive/`、`audits/`、`contracts/`、`design/`、`decisions/`、`workflow/`、`migration/` 等展开式历史目录。历史细节需要追溯时从 git 历史读取。

各模块只保留 `design.md` 作为设计权威文档。限制条件与否决方向的证据引用直接嵌入 design.md 中的 `[evidence: ...]` 标注，指向对应测试文件和 git 历史。

## 模块间关系

各业务模块之间的数据流向如下。`flight_dynamic` 是平台状态的生产者；所有传感器模块消费平台状态、外部目标和环境输入后独立输出。

```mermaid
flowchart LR
  subgraph Common["common/ · 公共基础层"]
    CORE[坐标 · 大气 · 传播\n定时 · 数值 · 校验]
  end

  subgraph FD["flight_dynamic · 飞行动力学"]
    direction TB
    JSB[JSBSim 动力学引擎]
    STATE[PlatformState\nposition_lla · velocity_ned · attitude · altitude_m]
    JSB --> STATE
  end

  subgraph EXT["外部输入"]
    TGT[Targets / Scene\n目标 / 场景]
    ENV[Environment\n大气 / 环境]
    IQ[External Raw IQ\n外部原始 IQ]
    EMIT[Emitter Scene\n辐射源场景]
  end

  subgraph SENSORS["传感器模块"]
    AR[airborne_radar\n机载雷达]
    EO[electro_optical_sensor\n光电传感器]
    ESR[electronic_surveillance_radar\n电子侦察]
    SAR[sar\n合成孔径雷达]
  end

  subgraph OUT["模块独立输出"]
    O1[Track / Detection\n航迹 / 探测]
    O2[Detection / Classification\n检测 / 分类]
    O3[Intercept / ELINT\n截获 / 情报]
    O4[SAR Image / SLC\n图像 / 复数据]
  end

  CORE -.->|共享类型| FD
  CORE -.->|共享类型| SENSORS

  STATE -->|平台状态| AR
  STATE -->|平台状态| EO
  STATE -->|平台状态| ESR
  STATE -->|平台状态| SAR

  ENV --> AR
  ENV --> EO
  TGT --> AR
  TGT --> EO
  TGT --> SAR
  IQ -.->|可选的| SAR
  EMIT --> ESR

  AR --> O1
  EO --> O2
  ESR --> O3
  SAR --> O4
```

读图规则：
- 箭头表示数据流向，虚线表示可选路径或跨模块共享类型。
- 没有传感器模块之间的直接数据流——各传感器独立处理平台状态和外部输入。
- `common/` 层提供坐标转换、大气物理、数值方法等跨模块共享类型，不作为独立运行时层。
- `flight_dynamic` 是唯一的平台状态生产者；传感器模块不反向影响飞行动力学。
