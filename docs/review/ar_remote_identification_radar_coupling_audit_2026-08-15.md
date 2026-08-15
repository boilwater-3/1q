---
Status: draft
Date: 2026-08-15
Review-Baseline: `main` @ `96de367c`（merge: SBIRS 2026-08 正式设计变更与审计修复落地）
Authority: 审计报告；为"远程识别雷达从机载雷达模块解耦"的拆分任务提供
  归属判定标准与文件级迁移清单。不得替代各模块 `design.md`/`boundaries.md`；
  若本文与库实现冲突，以库为准。
---

# 机载雷达（AR）模块归属审计：远程识别雷达耦合识别与解耦边界

## 0. 定位与结论

审计范围：`include/1q/airborne_radar/`（45 个 public 头）、`src/airborne_radar/`
（162 个源文件）、`schemas/`、`tools/`、`tests/`、`examples/`、`docs/airborne_radar/`
中与识别相关的全部内容。

结论：AR 模块内混入的"非机载雷达"功能**只有一块**——**远程识别雷达子系统（kLrr）**，
涵盖四维特征提取、多周期积累、模板匹配、只读特征数据库、建库工具、识别配置与结果类型、
以及场景侧识别真值输入。未发现 ESR/SAR/SBIRS/EOS 或其他雷达功能侵入。

**装备关系前提（已确认）**：远程识别雷达是与机载雷达相互独立的另一部雷达装备，
不是 AR 的工作模式或子能力。当前 AR 文档把 kLrr 定位为"AR 的并行输出子能力"
（`docs/airborne_radar/design.md` §远程识别子系统），该定位本身即耦合的文档化表现，
本审计予以推翻。

本审计**只做归属判定与迁移清单，不修改任何代码**。拆分实施另立任务，
本文 §9 为拆分验收标准。

## 1. 边界判定标准

### 1.1 三条标准

1. **装备前提**：远程识别雷达是独立装备——自带硬件配置域、自管波束、独立输入输出、
   独立 replay/trace，与 AR 只存在"航迹供给"这一模块间接口。
2. **库内证据信号**（用于识别"寄生耦合"，不依赖功能价值判断）：
   - **模块解剖完整度**：本库每个独立雷达模块都有同一套标准解剖（四域配置 + 会话 +
     周期输入输出 + replay/trace + 校验码 + 自有资产/工具 + 测试 + 文档）。kLrr 几乎
     具备全套（独立配置子域、独立结果类型、独立 DB schema + 建库工具、独立 replay 表、
     独立测试、独立文档章节），唯独没有独立会话，且每件都寄生在 AR 解剖上——
     "全套器官、长在别人身上"即耦合的形态学证据。
   - **数据流独立性**：识别链只读 AR 已确认航迹与硬件数字，只写回航迹快照，不进
     `DecisionInputFrame`/威胁评估/控制归约——纯并行输出、无循环依赖。
     旁证：`ArExternalOutputAdapter::TryMakeExternalTrackFromSnapshot` 逐字段构造快照时
     **不拷贝** recognition，结论由 `ArController` 事后直接写回 output_frame——
     识别是后挂平行链，与 AR 主链路无数据依赖。
   - **物理口径异质性**：识别有两条"识别专用更高保真观测路径"（F1 双通道极化、
     F2 距离像相干叠加），与探测链物理口径明确不对账（见 boundaries.md §识别子模型的
     物理保真度边界）。同一部雷达的同一物理链出现口径分裂是异常；独立装备则自洽。
3. **功能归属清单**：
   - 仿真机载雷达应具备：发射/接收/前端账本、物理探测、数据关联、航迹滤波与生命周期、
     扫描调度、战术决策（威胁/LPI/ECCM）、控制归约、干扰观察（自身接收链）、
     环境/传播、会话与 replay 基础设施。
   - 仿真远程识别雷达应具备：对航迹目标的驻留观测、四维特征提取（RCS/运动/双通道
     极化/宽带距离像）、多周期积累、模板匹配与型号/大类判定、特征数据库管理与加载、
     识别效能摘要。
   - 否决项沿用 `docs/airborne_radar/boundaries.md` §远程识别子系统边界中"非目标"
     清单（迁出时随文迁移，不改语义）。

### 1.2 与文档现状的冲突声明

AR 文档四篇（design/boundaries/data-flow/algorithms）均含 kLrr 章节并把识别写作
AR 子能力；`docs/common/contract.md` 未单独定义远程识别雷达。因此本审计的"应该具备
什么"没有现成成文依据，功能归属清单以 §1.1-3 为准，拆分任务第一步应把该清单
（含否决项）迁入新模块文档集。

## 2. 分类一：纯属远程识别雷达（整体迁出，AR 不留痕迹）

### 2.1 实现目录

`src/airborne_radar/recognition/` 全部 17 个文件（8 cpp + 9 h）：

| 文件 | 职责 |
|---|---|
| `RcsFeatureExtractor.{h,cpp}` | RCS 特征提取（视角覆盖 + SNR 门控） |
| `MotionFeatureExtractor.{h,cpp}` | 运动特征提取（速度/高度/加速度/转弯半径） |
| `PolarizationFeatureExtractor.{h,cpp}` | 双通道极化特征提取（F1） |
| `RangeProfileFeatureExtractor.{h,cpp}` | 一维距离像特征提取（F2，散射中心投影） |
| `RecognitionObservationBuilder.{h,cpp}` | 效能化观测构造（SNR/带宽/驻留约束 → 特征集） |
| `RecognitionFeatureDatabase.{h,cpp}` | SQLite 只读加载器（schema v1.1 校验、全量驻留内存） |
| `RecognitionMatcher.{h,cpp}` | 加权相似度匹配、候选排序、大类分数 |
| `RecognitionTracker.{h,cpp}` | 多周期积累、判定状态机、结论保持/过期、快照回滚 |
| `RecognitionTypes.h` | 识别内部类型（观测上下文/特征集/观测结构） |

[evidence: src/airborne_radar/recognition/*]

### 2.2 public 类型（迁出并改前缀，`ArRecognition*` 前缀随迁废弃）

| 文件 | 内容 |
|---|---|
| `include/1q/airborne_radar/session/ArRecognitionResult.h` | `ArRecognitionState` / `ArRecognitionCategory` / `ArRecognitionFeatureDimension` / `ArRecognitionFeatureScores` / `ArRecognitionResult` / `ArRecognitionCycleSummary` |
| `include/1q/airborne_radar/config/ArRecognitionConfig.h` | `ArRecognitionFeatureWeights` / `ArRecognitionConfig`（enabled、积累确认、判定门限、保持时间、最大距离、驻留、权重、数据库路径） |

[evidence: include/1q/airborne_radar/session/ArRecognitionResult.h]
[evidence: include/1q/airborne_radar/config/ArRecognitionConfig.h]

### 2.3 资产与工具

| 路径 | 角色 |
|---|---|
| `schemas/recognition/recognition_feature_database.sql` | 识别特征库权威 DDL（schema v1.1 单源） |
| `tools/recognition_db_builder.py` | 建库工具（DDL 唯一消费方之一） |
| `examples/configs/recognition/recognition_database_input.json` | 建库输入源 |
| `examples/configs/recognition/target_feature_database_v1.1.db` | 交付库（集成测试经 `ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH` 加载） |

### 2.4 识别专属测试（整体迁出）

| 文件 | 层级 |
|---|---|
| `tests/unit/airborne_radar/RecognitionSqliteTestUtil.h` | 测试共用（临时库写入 + DDL 常量） |
| `tests/unit/airborne_radar/ar_recognition_database_test.cpp` | 库加载/拒绝矩阵 + 匹配器 |
| `tests/unit/airborne_radar/ar_recognition_feature_test.cpp` | 观测构造 + 四提取器 + 门控 |
| `tests/unit/airborne_radar/ar_recognition_integration_test.cpp` | kLrr 调度/积累/保持/回滚/replay 溯源 |
| `tests/integration/airborne_radar/ar_recognition_example_database_test.cpp` | 交付库加载 |
| `tests/integration/airborne_radar/ar_recognition_scenario_test.cpp` | 七类场景 + 混合 + 模式切换 + replay 往返 |
| `tests/integration/airborne_radar/ar_recognition_us_military_scenario_test.cpp` | 美方公开型号 15 型参数化场景 |

## 3. 分类二：AR 本体中被侵入的耦合点（删除/归还清单）

### 3.1 配置/输入/输出 public 面

| 位置 | 侵入内容 | 处置 |
|---|---|---|
| `ArOrientationConfig.h:70` | `ArWorkMode::kLrr = 4` | 删除枚举值；新模块自带工作模式 |
| `ArPolicyConfig.h:156` | `ArRecognitionConfig recognition{}`（policy 第七子域） | 删除子域 |
| `ArSceneTypes.h:20-53` | `AspectRcsSample` / `PolarizationRcsSample` / `RangeRcsScatterer` | 迁入新模块场景输入 |
| `ArSceneTypes.h:84-87` | `ArSceneTarget` 三个识别真值字段（`aspect_rcs_samples` / `polarization_rcs_samples` / `range_rcs_scatterers`） | 删除字段 |
| `TrackStateSnapshot.h:70-71` | `recognition` 字段（结论寄生在 AR 航迹快照） | 删除字段；结论归新模块独立输出 |
| `ArCycleResult.h:68-70` | `has_recognition_summary` / `recognition_summary` | 删除字段；摘要归新模块结果 |
| `ArIssueCodes.h:103-117` | `kRecognitionWeightsInvalid` 等 5 个 issue code | 迁入新模块 issue code 集 |
| `airborne_radar.hpp:16` | include `ArRecognitionResult.h` | 删除聚合 |
| `config/airborne_radar_config.hpp:16` | include `ArRecognitionConfig.h` | 删除聚合 |
| `tests/contract/check_public_api_boundary.cmake:26,54` | 白名单两行识别头 | 移除白名单条目 |

### 3.2 执行链（src 胶合）

| 位置 | 侵入内容 |
|---|---|
| `runtime/ArController.h:64-69` | 快照字段：`recognition_tracker_state` / `recognition_config` / `recognition_database_path` / `latest_recognition_summary` / `has_latest_recognition_summary` |
| `runtime/ArController.h:78-84` | `ArRecognitionStaticContext`（借用 AR hardware 域 transmitter/receiver/antenna） |
| `runtime/ArController.h:101-106` | 构造参数 `recognition_static_context` |
| `runtime/ArController.h:112-126` | `UpdateRecognitionRuntime` / `HasLatestRecognitionSummary` / `GetLatestRecognitionSummary` / `GetActiveRecognitionDatabaseVersion` |
| `runtime/ArController.cpp:106-116` | Impl 成员（tracker/database/模式/摘要） |
| `runtime/ArController.cpp:229-265` | `UpdateRecognitionRuntime`（选项映射 + 库加载） |
| `runtime/ArController.cpp:267-326` | `PrepareLrrPointing` / `IsBetterLrrCandidate`（kLrr 波束指向 Path A，见 §5.1） |
| `runtime/ArController.cpp:328-336` | `ComputeSnrDb`（识别专用效能级 SNR，借用 AR `RadarEquations`） |
| `runtime/ArController.cpp:339-358` | `MakeRecognitionContext` |
| `runtime/ArController.cpp:360-413` | `RunRecognitionCycle`（积累/判定/回填） |
| `runtime/ArController.cpp:423-449` | 四个转发方法 |
| `runtime/ArController.cpp:502-551` | RunOnce 内 kLrr 指向调用 + 识别执行/禁用回填 |
| `runtime/ArController.cpp:683-688,723-732` | Capture/Restore 识别状态快照 |
| `signal/pipeline/ISignalPipeline.h:61-75` | `SetCycleScanCenterOverride` / `ClearCycleScanCenterOverride`（kLrr 专用接口） |
| `signal/pipeline/SignalPipeline.h:120`、`.cpp:105,167` | `cycle_scan_center_override_` 成员与消费 |
| `signal/pipeline/ScanScheduleResolver.cpp:76,196,244` | kLrr 纯驻留 passthrough 分支 |
| `session/ArSession.cpp:113-121,150-152,303-305,892-907` | 识别运行期提交/摘要回填/replay `active_database_version` 采集 |
| `session/ArSessionCompositionRoot.cpp:45-52` | 从 AR hardware 组装识别静态上下文 |
| `session/ArSessionConfigBuilder.cpp:172-220` | 识别配置校验段（5 个 issue code 触发点） |
| `config/mapping/RuntimePatchMapper.h:25`、`.cpp:104` | recognition 整域 patch 搬运 |
| `session/ArExternalInputAdapter.cpp:159-161` | 识别真值字段拷贝进场景目标表 |
| `session/ArReplayCycleRecord.h:46` | `ArSessionReplayState::active_database_version` |
| `session/ArReplayFlatbufferCodec.cpp:31-56,93,129` | 识别结果编解码 |
| `session/ArReplayFlatbufferCodec.cpp:341` | 工作模式合法范围上界含 `kLrr`（删除后收紧为 `kStt`） |
| `session/ArReplayFlatbufferCodec.cpp:706-720,957-976` | 识别配置编解码 |
| `session/ArReplayFlatbufferCodec.cpp:1369-1380,1445-1446,1495-1496` | 识别摘要编解码 |
| `session/ArReplayFlatbufferCodec.cpp:1539-1566` | `active_database_version` 编解码 |

### 3.3 replay schema

| 位置 | 侵入内容 |
|---|---|
| `schemas/replay/airborne_radar_replay.fbs:14,59` | `ArRecognitionResultV1` 表 + 航迹快照 `recognition` 字段 |
| `schemas/replay/airborne_radar_replay.fbs:301,336-337` | `ArRecognitionCycleSummaryV1` 表 + 结果帧两字段 |
| `schemas/replay/airborne_radar_session_replay.fbs:166,173-183,195` | `ArRecognitionFeatureWeightsV1` / `ArRecognitionConfigV1` 表 + policy 表 `recognition` 字段 |

### 3.4 构建

| 位置 | 侵入内容 |
|---|---|
| `src/airborne_radar/CMakeLists.txt:32-39` | `recognition/` 8 个 cpp 列入 `AIRBORNE_ENGINE_SOURCES` |
| `src/airborne_radar/CMakeLists.txt:90` | `airborne_engine` 私有链接 `SQLite::SQLite3`（识别专用依赖） |

### 3.5 嵌入识别断言的测试（部分删除，测试本体保留）

| 文件 | 嵌入段 |
|---|---|
| `tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp:68-82,139-153` | 识别配置 roundtrip 段 |
| 同上 `:364-442` | 识别结果/快照/会话状态 roundtrip 三段（`RecognitionFieldsRoundtripPreserved` 等） |
| `tests/unit/airborne_radar/ar_runtime_patch_mapper_test.cpp:182-221` | `HasPolicyCarriesRecognitionConfig` / `RejectedPatchKeepsRecognitionConfigUnchanged` |
| `tests/unit/airborne_radar/ar_session_config_builder_test.cpp:420-495` | 识别校验 6 用例 + 默认值 2 用例 |
| `tests/unit/airborne_radar/ar_signal_scan_schedule_test.cpp:556,588` | kLrr 调度用例 |

### 3.6 文档与示例

| 位置 | 侵入内容 |
|---|---|
| `docs/airborne_radar/design.md:45-60` | §远程识别子系统（kLrr）+ 导航行 |
| `docs/airborne_radar/boundaries.md:189-253` | §远程识别子系统边界 + §F1/F2 物理保真度边界 + §识别特征数据库契约 |
| `docs/airborne_radar/data-flow.md:234-251` | §远程识别链路（kLrr） |
| `docs/airborne_radar/algorithms.md:260-281` | §远程识别链路（kLrr） |
| `examples/configs/README.md:15-22` | recognition/ 子目录说明 |
| `examples/README.md:42,90` | recognition 目录引用 |
| `examples/component_attachment/logger/logger_i18n.h:190-194` | 5 条识别 issue code 中文翻译 |

## 4. 分类三：确认属于机载雷达本体（保持不动）

| 子域 | 判定 |
|---|---|
| `signal/detection`（雷达方程/前端账本/检测/量测误差） | AR 探测链本体 |
| `signal/association`、`signal/tracking` | AR 关联与航迹本体 |
| `signal/pipeline`（除 §3.2 波束覆盖三处） | AR 物理流水线本体 |
| `decision`（威胁/LPI/ECCM/归约/战术协调） | AR 决策本体。`LpiSourceInfo::has_recon_platform` 是 LPI 决策对场景侦察平台的观察输入，本体 |
| `runtime`（除识别段）、`session` 基础设施、`config`（除 `ArRecognitionConfig`） | AR 会话/运行时/配置本体 |
| `environment`（环境服务/传播模型/场景管理） | AR 探测链输入本体 |
| `output`、`utils` | AR 输出与工具本体 |
| 干扰观察（`ArInterferenceObservationResolver` 等） | AR 自身接收链对干扰的观察，非 ESR 被动侦察功能；ESR 已独立成模块，无侵入 |
| trace/replay 基础设施 | 模块级观测能力本体（识别数据迁出后保留空壳结构） |

## 5. 三条接缝按"独立装备"的处置

### 5.1 波束（Path A）——AR 全删，识别雷达自管

现状：`ArController::PrepareLrrPointing` 从 AR 上一周期输出选优先确认航迹（威胁等级
优先、斜距次近），经 `ISignalPipeline::SetCycleScanCenterOverride` 注入 AR 波束；
`ScanScheduleResolver` 对 kLrr passthrough。

处置：三件套接口 + 成员 + 分支 + `ArWorkMode::kLrr` 全部删除；`PrepareLrrPointing` /
`IsBetterLrrCandidate` 的优先航迹选择逻辑迁入新模块的波束调度；AR replay codec 的
工作模式合法范围上界从 `kLrr` 收紧为 `kStt`。若场景需要两雷达对同目标协同驻留，
由调用方编排（模块间不建波束服务接口，符合"独立装备"前提）。

### 5.2 硬件——新模块自带 hardware 域

现状：`ArSessionCompositionRoot` 把 AR 的 engineering `TransmitterConfig` /
`ReceiverConfig` / `AntennaConfig` 塞进 `ArRecognitionStaticContext`；识别 SNR 由
`ComputeSnrDb` 调用 AR internal 的 `RadarEquations`。

处置：删除借用胶合。新模块四域配置的 hardware 域自带 transmitter/receiver/antenna
（含识别任务参数：`max_range_m`、`recognition_dwell_sec` 等语义归位）；识别链路预算
由新模块自带实现（可经 `src/common/` 共享雷达方程，但不得跨模块引用 AR internal
`RadarEquations`）。

### 5.3 航迹供给——消费 AR 公开输出，不新增 AR 机制

现状：`RunRecognitionCycle` 读 internal `decision_frame.tracks`（`TrackStateSnapshotList`）。

处置：新模块消费 AR 现有公开 `TrackOutputFrame` 的 confirmed 航迹（位置/速度/威胁
类别/association_key）。AR 无需新增任何供给接口；调用方编排顺序为"AR `Step()` →
识别雷达消费航迹 `Step()`"。注意：AR 的航迹生命周期（`kLost`/`kRecycled`/键重分配）
语义对新模块积累状态的影响，需在拆分任务中按 boundaries.md 既有规则
（键重分配视为新目标、丢失保持结论至 hold 过期）迁移为模块间契约条款。

### 5.4 其余接缝一句话处置

- 场景真值：`ArSceneTarget` 三字段迁为新模块 `CycleInput` 的识别真值输入。
- 输出：识别结论/摘要迁为新模块独立输出类型（不寄生 AR 快照与结果帧）。
- 配置：识别配置升级为独立 `*SessionConfig` 的 policy 域，独立校验与 issue code。
- replay：识别表迁为新模块独立 fbs + codec；`active_database_version` 移出
  `ArSessionReplayState`。
- 依赖：`airborne_engine` 的 SQLite3 链接删除；若全库无其他 SQLite 消费，conan
  依赖可同步瘦身（拆分任务时确认）。

## 6. 新模块标准解剖（迁移后形态）

按本库五模块同构约定，远程识别雷达模块应具备（对应迁移来源）：

| 解剖件 | 内容 | 来源 |
|---|---|---|
| config 四域 | hardware（自带雷达参数）/ mission（识别任务与工作模式）/ policy（权重、门限、窗口、驻留）/ environment | `ArRecognitionConfig` 升级 + 新 hardware/mission |
| session | `*CycleInput`（含场景识别真值）、`*CycleResult`（识别结论 + 效能摘要）、validation/issue codes | `ArSceneTypes` 真值字段 + `ArRecognitionResult` + 5 issue code |
| 内部实现 | 观测构造/四提取器/积累/匹配/数据库加载/波束调度 | `src/airborne_radar/recognition/` + `PrepareLrrPointing` 逻辑 |
| replay/trace | 独立 fbs + codec + `database_version` 溯源 | AR codec 识别段 + fbs 表 |
| assets | schema + 建库工具 + 示例库 | §2.3 四项 |
| tests/docs | 识别测试 7 文件 + 文档 4 章节 + 示例说明 | §2.4 + §3.6 |

命名与治理（2026-08-15 定稿）：模块目录/命名空间 `remote_identification_radar`，
public 前缀 `Rir*`（如 `RirSession`/`RirRecognitionConfig`），issue code 前缀
`rir.*`；`ArRecognition*` 前缀废弃，不保留 deprecated compat 层
（对齐 `Radar*`→`Ar*` 一次性迁移先例）；`check_public_api_boundary.cmake` 白名单
同步收敛。第一阶段计划见
[ar_remote_identification_decoupling_phase1_plan_2026-08-15.md](ar_remote_identification_decoupling_phase1_plan_2026-08-15.md)。

## 7. 关键风险与兼容性说明

1. **replay 字节兼容断裂（接受）**：AR replay fbs 删除识别表/字段、`ArWorkMode` 值域
   收紧均为破坏性 schema 变更。本库无存量 replay 数据跨版本承诺的明确先例约束，
   拆分任务应评估是否保留 schema 版本号兼容层；按 AR 现状惯例（无旧版本兼容层，
   `ArWorkMode` 值不重排原则仅保护枚举值加性扩展），建议一次性断裂并在 release
   note 明示。
2. **航迹生命周期耦合**：识别积累状态依赖 AR 航迹状态机（confirmed/lost/recycled/
   键重分配）。迁移后该依赖变为跨模块时序契约，需在拆分任务中补充集成测试锁定
   （现由 `ar_recognition_integration_test` 的保持/过期/回滚用例覆盖，迁出后改双模块
   集成场景）。
3. **`IsBetterLrrCandidate` 的威胁类型依赖**：优先航迹选择消费 AR 决策层填写的
   `target_type` 字符串（`HIGH_THREAT_FIGHTER`/`LOW_THREAT_TARGET`）。迁出后该逻辑
   只能消费 AR 公开航迹输出中的威胁类别字段，威胁分级语义需在模块间契约中固定
   （字符串枚举当前无公共定义，见 `TacticalDecisionTypes.h`）。
4. **SQLite 依赖面**：识别库加载器是 SQLite 唯一已知消费方；拆出后 AR 构建面
   移除 SQLite，需全库确认无其他引用（拆分任务验证步骤之一）。

## 8. 审计方法说明

- 以代码/构建/测试/资产为证据，不以文档声明代替（文档本身是耦合的一部分，
  §1.2 已声明）。
- 检索面：`Recognition`/`recognition`/`kLrr`/`aspect_rcs`/`polarization_rcs`/
  `range_rcs_scatterers`/`database_version`/`远程` 等关键词在 include/src/schemas/
  tools/tests/examples/docs 全域展开；`surveillance`/`esm`/`无源`/`电子侦察`/
  `passive` 反查确认无其他雷达功能侵入。
- 行号引用以审计基线 `96de367c` 为准。

## 9. 拆分验收标准（供后续任务引用）

1. AR 全域 grep：`kLrr|Recognition|recognition|aspect_rcs|polarization_rcs|
   range_rcs_scatterers|active_database_version` 除迁移说明注释外零命中
   （include/src/schemas/tests/examples 四层）。
2. `airborne_engine` 不再链接 SQLite3；`ArWorkMode` 值域为 kStby/kTas/kTws/kStt；
   replay codec 工作模式上界为 `kStt`。
3. AR public 白名单不含识别头；`ArPolicyConfig`/`ArSceneTarget`/
   `TrackStateSnapshot`/`ArCycleResult` 无识别字段；`ArIssueCodes.h` 无识别码。
4. AR 测试套件在删除识别段后全绿（`unit::airborne_radar`、`contract::airborne_radar`、
   `replay::airborne_radar`、`batch_validation::airborne_radar`、
   `integration::cross_domain`）；原 7 个识别测试文件迁入新模块并全绿。
5. 新模块具备 §6 全套解剖（四域配置、会话、validation、replay/trace、assets、
   tests、docs 设计文档集）；功能归属清单与否决项落入新模块 `boundaries.md`。
6. `check_public_api_boundary.cmake` 与 `check_cross_domain_naming.cmake` 通过。
7. AR 文档四篇移除 kLrr 章节并同步 §1.2 冲突声明涉及的定位变更；
   `examples/configs/README.md` 与 `logger_i18n.h` 收敛。
