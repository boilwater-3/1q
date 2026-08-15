---
Status: draft
Date: 2026-08-15
Plan-Baseline: `feature/remote-identification-radar-phase1` @ `725279de`
Revision: v2（2026-08-15 需求方定案：AR 与 RIR 完全独立、无模块间协作接口，
  目标改为把 AR 中满足 RIR 需求的能力模块迁移改造进 RIR、RIR 自持检测+
  轻量关联。v1 的"2-M AR 删除先行"段序与 M7/M8 双模块契约条款废止）
Authority: 第二阶段实施计划；AR 侧删除归属与验收以
  `docs/review/ar_remote_identification_radar_coupling_audit_2026-08-15.md`（下称
  《审计》）§3/§9 为准，九项信号链能力归属与口径以
  `docs/review/rir_signal_chain_capability_boundary_2026-08-15.md`（下称《边界》，
  同日修订版）为准。若本计划与库实现冲突，以库为准。
---

# 远程识别雷达 第二阶段计划（v2）：AR 能力模块迁移改造与 RIR 自持化

## 0. 策略与目标

**架构定案（推翻阶段 1 两项前提）**：AR 与 RIR 是完全独立的两部雷达装备，
**无任何模块间协作接口**——阶段 1 的"航迹供给"接缝（AR 输出 → RIR 输入）与
等价性测试配对关系整体退役。阶段 2 的主线是**把 AR 中满足 RIR 需求的能力模块
迁移改造（副本改写为 `Rir*`）进 RIR**，使 RIR 自持：自有检测链（九项能力）→
轻量关联/滤波/生命周期 → 内部目标图像 → 识别积累。

四段递进，顺序执行：

| 段 | 主题 | 性质 |
|---|---|---|
| **2-M 迁移改造** | AR 检测链与环境子集副本改写进 RIR（旁路新增，不接线） | 纯增量 |
| **2-T 轻量跟踪** | 识别所需跟踪子集（KF/关联/生命周期）副本改写 | 纯增量 |
| **2-S 自持化重构** | 输入面重构 + 流水线接线 + `RirTrackFeed` 退役 + 等价性测试删除 | **破坏性切换点** |
| **2-C AR 侧收尾** | AR 识别耦合删除（《审计》§3 清单 + §9 验收） | 删除 |

**等价性测试时序**：2-M/2-T 为旁路增量，现有链路零改动，等价性测试保持可跑
并持续绿；2-S 输入面重构时直接删除（无投影契约测试转生——AR↔RIR 配对关系
不存在，v1 的 M7/M8 条款废止）。

**显式延后项**：`max_range_m`/`recognition_dwell_sec` 四域归位、雷达方程 common
化（阶段 3）；CA-CFAR、战斗级跟踪（IMM/LAPJV/航迹池，见 D-A4 否决）。

## 1. 架构决策记录（v2 新增/修订）

| 编号 | 决策 | 说明 |
|---|---|---|
| D-A1 | **独立性**：RIR 输入 = 场景目标（caller）+ RF 场景（common incident links + own emission 身份）+ 环境数据（caller）+ 平台状态 | 不消费任何 AR 输出；调用方编排与 AR 无关 |
| D-A2 | **自持链路**：检测（九项能力）→ 轻量关联/滤波/生命周期 → 内部航迹 → 识别积累 | 识别积累的 `association_key`/`hit_count`/confirm 语义内化为自产 |
| D-A3 | **`RirTrackFeed` 公开输入退役**（2-S 破坏性变更） | 类型语义内化为 internal 航迹类型；`RirTrackFeedEntry`/`RirTrackFeedStatus` 出 public 面 |
| D-A4 | **轻量边界**：单目标 KF + 门限/最近邻关联 + 计数生命周期 | IMM、LAPJV 全局关联、航迹池、combat-grade 跟踪**不迁**（识别雷达低密度驻留观测假设，写入 boundaries 非目标） |
| D-A5 | **驻留排序语义变更**：威胁等级输入（AR 威胁分类 `target_type`）随独立性消失 → 改"未识别优先（`kUnknown`/无结论优先）+ 斜距次近" | `IsBetterLrrCandidate` 威胁排序逻辑不迁移 |
| D-A6 | **真值统计随迁**：`target_name` 移入 `RirSceneTarget`（人读+准确率统计，语义不变） | 摘要正确率统计不改语义 |
| D-A7 | **场景目标补速度**：`RirSceneTarget` 增 velocity（检测单元多普勒/径向速度消费）+ `target_swerling_type` | 阶段 1"场景目标不含速度"约定随 D-A2 失效 |

## 2. 段 2-M：能力模块迁移改造（纯增量旁路）

副本改写纪律沿用阶段 1：文件头注来源 commit、类型换 `Rir*`、零 AR include；
每步单测随迁改写；**不接线现有链路**（`RirController` 不动）。

| 步 | 内容 | AR 来源 |
|---|---|---|
| M1 | `RirRadarEquations` 扩充为全函数集：积累增益 `G=N`、测距/测角误差、Swerling 0-4 检测概率、Marcum Q 门限、蒙特卡洛判决 | `signal/detection/RadarEquations.*` 全集 |
| M2 | `dwell/RirAntennaPatternRuntime`（4 主瓣模型 + 扫描损失 + 主/旁/后瓣）+ `RirBeamControl` 子集（有效波束宽度推导、安装系指向、离轴角→增益） | `AntennaPatternRuntime.h`、`BeamControlResolver.h` |
| M3 | `dwell/RirDetectionCellResolver`：线性域雷达方程 + 时延/多普勒 + 接收窗脉冲计数 + **干扰时频重叠聚合**（`TryResolveCellInterference` 语义副本；抗 RGPO 减半不带，ECCM 属 AR） | `ArDetectionCellResolver.*` |
| M4 | `dwell/RirSignalDetector`：统计级 CFAR 判决链（Pfa 门限 → Swerling Pd → 种子判决；`min_snr_db`/`min_detection_margin_db` 门）；种子 `SetRandomSeed` 语义 | `SignalDetector.*` |
| M5 | `internal/RirPropagationModel` 子集：传播损耗 + 杂波功率（基线+植被散射物理混合口径）；植被场景事实入 `RirEnvironmentConfig`（**激活空占位域**，满足其"先有真实消费路径"头注；基线系数按 AR 环境域"不含内部调参项"合约保留在实现内部），植被场景数据入周期输入（阶段 2-S） | `environment/PropagationModel.*` 语义 |
| M6 | `dwell/RirMeasurementErrorModel` 子集：检测量测位置误差（SNR/带宽驱动）→ 关联与滤波输入 | `MeasurementErrorModel.h` |

四增益偏置参数（《边界》§3.2 方案 A）随 M3/M4 落地：`RirSignalProcessingConfig`
入 `RirHardwareConfig`（默认 0 dB），账本分子分母施加点同《边界》表。

## 3. 段 2-T：轻量跟踪子集迁移（纯增量旁路）

**识别消费闭包**（迁移范围判定——仅迁识别链实际消费的字段与语义，防蔓延）：
`association_key`（跨周期身份）、`status`（kConfirmed 门）、`hit_count`（键重分配
检测）、位置/速度/加速度/`estimation_uncertainty_trace`（运动特征）。

| 步 | 内容 | AR 来源 |
|---|---|---|
| T1 | `tracking/RirTrackFilter`：单目标 KF 预测/更新（位置-速度-加速度状态 + 协方差→不确定度迹） | `KalmanPredictor`/`KalmanUpdater`（IMM 不迁） |
| T2 | `tracking/RirTrackAssociator`：门限 + 最近邻检测-航迹关联（波门按量测误差定标） | `DataAssociation`/`DistanceMetric` 子集（LAPJV/假设分支不迁） |
| T3 | `tracking/RirTrackLifecycle`：hit 计数、confirm/lost 判定、键回收；**键重分配=新目标语义内化**（阶段 1 供给语义转自产） | `TrackLifecycleManager` 子集 |
| T4 | internal 航迹类型（对齐消费闭包字段；`RirTrackFeedEntry` 语义的内部化身） | `TrackState` 子集 |

## 4. 段 2-S：自持化重构（破坏性切换点）

| 步 | 内容 | 设计要点 |
|---|---|---|
| S1 | 输入面重构：`RirSceneTarget` 增 `velocity_x/y/z`/`target_name`/`target_swerling_type`（D-A6/A7）；`RirCycleInput` 增 RF 场景（incident links + own emission 身份）与环境快照（天气）；**`RirTrackFeed` 公开输入退役**（D-A3） | 契约白名单/`ONEQ_SENSOR_SESSION_CONTRACT` 签名核对；校验与 `rir.validation.*` 新码 |
| S2 | 流水线接线：`RirController` 内 检测（M 段）→ 量测误差（M6）→ 关联/滤波/生命周期（T 段）→ 识别积累（既有 `RirTracker`）；驻留指向消费**内部航迹**（D-A5 排序：未识别优先+斜距次近）；`kIdentify` 门控整链 | 工作模式语义不变（kStby/kIdentify）；关机不触碰链路状态 |
| S3 | 会话/replay/输出适配：replay 破坏性版本评估（航迹语义内化 + 检测随机性种子入 replay 状态）；`RirRecognitionCycleSummary` 增驻留预算摘要（《边界》B7 口径） | 三层输出模型不变 |
| S4 | 场景级集成测试重锚定：标注场景期望值按自持架构重算（观测序列来自自产航迹）；单元/集成/replay 测试集适配 | 阶段 1 测试中依赖 `RirTrackFeed` 的用例改内部注入或重写 |
| S5 | 文档四件套改写：boundaries.md（"航迹供给"章节→"独立输入面"、非目标 #4 精化为"不做战斗级跟踪/关联决策/战术决策，自持轻量跟踪"、供给契约条款删除）；design.md 模块定位；data-flow.md（图与状态所有权：内部航迹归 `RirTrackLifecycle`/`RirTrackFilter`）；algorithms.md 登记表 +检测/关联/滤波/生命周期行 | 《边界》§6 修订版条款为权威 |
| S6 | 等价性测试删除 + `integration::cross_domain` partition 清理 | 配对关系不存在；审计 §9.4 该条款按清理后状态解释 |

## 5. 段 2-C：AR 侧收尾（《审计》§3 执行序）

| 步 | 内容 | 引用 |
|---|---|---|
| C1 | public 面删除（`kLrr` 枚举、识别配置子域、场景真值类型/字段、寄生字段、issue codes、聚合头、契约白名单） | §3.1 |
| C2 | 执行链胶合删除（`ArController` 识别段、扫描中心覆盖接口、kLrr 分支、会话/composition/patch/适配） | §3.2 |
| C3 | replay schema 收敛（识别表/寄生字段/codec 段；工作模式上界收紧 `kStt`） | §3.3 |
| C4 | 构建与依赖（`recognition/` 8 cpp 移出、SQLite3 链接删除、conan 瘦身确认） | §3.4 |
| C5 | 嵌入识别断言测试段删除 | §3.5 |
| C6 | 文档与示例（AR 四篇 kLrr 章节、§1.2 冲突声明、examples README、`logger_i18n.h`） | §3.6 |
| C7 | 段验收：《审计》§9 七条逐条核对 | §9 |

## 5.5 执行进度（2026-08-15）

- **段 2-M（迁移改造）已完成**：M1 `e6e073b9`（雷达方程检测子集全函数集 +
  `RirSwerlingModel`）、M2 `87bd2ae1`（`dwell/RirAntennaPatternRuntime` +
  `RirBeamControl`）、M3 `99b53792`（`dwell/RirDetectionCellResolver` 干扰聚合 +
  四增益偏置入 `RirHardwareConfig`/校验/issue code）、M4 `f28cb874`
  （`dwell/RirSignalDetector` 统计级 CFAR + Pfa 闭环/统计验证）、M5 `950625ed`
  （`internal/RirPropagationModel` 植被散射杂波 + 环境域激活）、M6 `e66d5293`
  （`dwell/RirMeasurementErrorModel`）。
- **段 2-T（轻量跟踪迁移）已完成**：T4 内部航迹类型前置交付 `6fa38268`
  （`tracking/RirTrackTypes.h`，识别消费闭包内部化身；`2020f739` 补
  `speed`/`acceleration_mps2` 派生模长字段）；T1 `0b3e8d70`
  （`tracking/RirTrackFilter.*`：common 6/3 KF 包装，初始化/CV 预测/动态 R
  更新）；T2 `79bef1d6`（`tracking/RirTrackAssociator.*`：马氏门限 +
  全局最近邻唯一分配，LAPJV/假设分支不迁）；T3 `ce995bda`
  （`tracking/RirTrackLifecycle.*`：hit 计数、confirm/lost/回收、KF 接线、
  运行态捕获恢复；对象池/IMM/反欺骗分支不迁）。
- 段验证门通过：unit 97/97（阶段 2-T 随迁新增 23 例；阶段 1 既有 26 例零修改
  通过）、integration 28/28、replay 3/3、cross_domain 7/7（含等价性测试）——
  旁路增量零回归。
- 构建注记：本机 64 位 MSBuild 需 `UCRTContentRoot` 环境变量（预设已内置，
  见 `VisualStudio.15.0-amd64` preset 描述）；直连 `cmake --build` 须显式导出。

## 6. 文件级映射（RIR 侧增量；AR 侧见《审计》§3）

| 动作 | 路径 | 说明 |
|---|---|---|
| 新增 | `src/remote_identification_radar/dwell/`：`RirAntennaPatternRuntime.h`、`RirBeamControl.*`、`RirDetectionCellResolver.*`、`RirSignalDetector.*`、`RirMeasurementErrorModel.h` | 检测链（M2-M4/M6） |
| 新增 | `src/remote_identification_radar/tracking/`：`RirTrackFilter.*`、`RirTrackAssociator.*`、`RirTrackLifecycle.*`、内部航迹类型 | 轻量跟踪（2-T） |
| 修改 | `internal/RirRadarEquations.*`（扩充）、`internal/RirPropagationModel.*`（新增） | M1/M5 |
| 修改 | `include/.../session/RirSceneTypes.h`、`RirCycleInput.h`（增字段/增输入）；`config/RirEnvironmentConfig.h`（激活）、`RirHardwareConfig.h`（+`RirSignalProcessingConfig`）、`RirPolicyConfig.h`（+检测策略/门控模式） | 无新 public 头；`RirTrackFeedTypes.h` 删除（D-A3） |
| 修改 | `runtime/RirController.*`、`session/*`、`RirIssueCodes.h`、`RirCycleResult.h`、replay codec、模块 CMake | S2/S3 |
| 删除 | 《审计》§3.1-§3.6 全清单；`tests/integration/cross_domain/ar_rir_recognition_equivalence_test.cpp`；`include/.../session/RirTrackFeedTypes.h` | AR 树 + 阶段 1 供给面 |

## 7. 测试计划与验证门

1. 2-M/2-T 每步：随迁单测 + **既有测试零改动全绿**（旁路增量验证门）。
2. 2-S：缺省语义对齐检查（检测链在无干扰/无环境输入下的退化口径 = 旧 SNR 公式，
   逐位）；场景级重锚定测试（S4）；replay 破坏性版本 roundtrip。
3. 检测门 Pfa 统计验证（固定种子蒙特卡洛，虚警率偏差在抽样容差内）；账本分项
   单独激励；检测随机性种子确定性（同种子同输入同检测序列）。
4. 2-C 末：AR 全量套件 + 审计 §9；2-S 后全量 release `/completeness-review`。

## 8. 风险与处置

| 编号 | 风险 | 处置 |
|---|---|---|
| R-Q1 | 自持检测蒙特卡洛随机性 → 观测序列抖动影响识别积累 | 积累窗口（`min_confirmed_hits`/`min_observation_count`）天然吸收；种子入 replay；Pd 高于门限时判决稳定的场景入回归 |
| R-Q2 | 2-S 破坏性公开变更波及调用方 | S1 一次到位（不设 compat 层，对齐阶段 1 "无 compat"先例）；examples/tests 同步改造入 S4 |
| R-Q3 | 轻量关联密集场景错关联 → 运动特征污染 | 低密度驻留假设写入 boundaries 非目标与算法登记；波门定标单测；超密度场景不在验证范围 |
| R-Q4 | 跟踪/检测副本与 AR 版漂移 | 副本头注来源 commit 与差异点；阶段 3 common 化收敛 |
| R-Q5 | replay 破坏性版本割裂存量记录 | 版本号显式升位 + 旧版本拒绝策略（拒绝并报 issue，不静默误读） |
| R-Q6 | 驻留排序语义变更（D-A5）未被场景测试感知 | 排序纯函数单测 + 驻留选择入周期摘要（可观测） |
| R-Q7 | 迁移体量蔓延到战斗级跟踪 | §3 消费闭包表为冻结边界；超闭包需求须重开边界评审 |

## 9. 验收标准

1. RIR include 闭包：无 AR 头（不变式）+ 无 `RirTrackFeed*` public 残留 +
   RIR 测试/示例零 AR 输出消费。
2. 自持链路端到端：场景目标 + RF 场景 + 环境输入 → 检测 → 内部航迹 → 识别结论
   的场景级测试全绿；`kStby`/关机/校验拒绝语义不回退。
3. 《审计》§9 七条（2-C）；《边界》§8 修订版（检测门/账本/增益条目按自持口径）。
4. 检测门 Pfa 统计验证与种子确定性通过；四增益偏置默认 0 dB 等价性与超界拒绝
   通过。
5. 全量 release `/completeness-review` 通过。

## 10. 阶段 3 预告（本计划不展开）

雷达方程/方向图/检测单元/干扰聚合 common 化（AR 与 RIR 副本收敛）；`max_range_m`/
`recognition_dwell_sec` 四域归位；会话配置 replay 与 trace 事件流评估。
