---
Status: draft
Date: 2026-08-15
Plan-Baseline: `feature/remote-identification-radar-phase1` @ `19f2447c`
Authority: 第二阶段实施计划；AR 侧删除归属与验收以
  `docs/review/ar_remote_identification_radar_coupling_audit_2026-08-15.md`（下称
  《审计》）§3/§9 为准，九项信号链能力归属与口径以
  `docs/review/rir_signal_chain_capability_boundary_2026-08-15.md`（下称《边界》）
  为准。若本计划与库实现冲突，以库为准。
---

# 远程识别雷达 第二阶段计划：AR 侧删除收尾 + RIR 驻留链路预算（九项能力落地）

## 0. 策略与目标

三段递进，每段独立可交付、独立验证，顺序执行：

| 段 | 主题 | 依据 |
|---|---|---|
| **2-M（主线）** | AR 侧识别耦合删除 + 等价性测试退役 + 双模块集成契约文档化 | 《审计》§3/§5/§9；阶段 1 计划 §9 预告 |
| **2a（输入面与前置）** | 环境快照 / RF 入射链路输入、RIR 自建波形视图、驻留指向迁移 | 《边界》决策 10/11 |
| **2b（驻留链路预算）** | 方向图评估、四增益偏置、分项 SINR 账本、统计级 CFAR 检测门、特征门控升级 | 《边界》决策 1/5/6/7/8/9、§3.2 参数化 |

**默认兼容公理（贯穿 2a/2b 的设计不变式）**：所有新输入与新参数的缺省值必须
逐位复现阶段 1 行为——`propagation_loss_db=0`、`clutter_power_db=0`、
`incident_links` 空、`target_swerling_type=0`、四增益偏置 0 dB、
`enable_directional_pattern=false`（既有默认）、检测策略 `enabled=false`、
门控模式 legacy。任何一步都可用"全缺省输入"回归门验证零行为漂移。据此，
《边界》§7 的时序约束（先删 AR 侧再升级口径）自然满足：本计划 2-M 先行，
且 2a/2b 的口径升级只在显式配置/输入下激活。

**显式延后项（本计划不做）**：`max_range_m`/`recognition_dwell_sec` 四域归位
（mission 域）——与阶段 3 common 化同批，避免一次变更叠加两个语义轴；
CA-CFAR、增益绝对覆盖模式（方案 C）、点迹/量测提取（见《边界》§5 否决清单）。

## 1. 前置与依赖

1. 《审计》§3.1-§3.6 为 AR 删除的权威文件清单，本计划不重复枚举，只定执行序
   与验证门；§9 七条为 2-M 验收。
2. 《边界》§5 决策 1-11 为 RIR 侧能力的权威归属；§3.2 四参数表为增益配置的
   权威定义（字段名、作用位置、符号约定、[0, 40] dB 值域）。
3. AR 冻结解除范围 = 《审计》§3 清单本身；§4 所列 AR 本体（探测链/关联/跟踪/
   决策/环境服务）仍然不动。
4. 现状基线：RIR 59 例测试（等价性 1 例除外）+ replay 字节快照为行为锁定层，
   2-M 删除等价性测试前须确认该基线全绿。

## 2. 段 2-M：AR 侧识别耦合删除（主线）

执行序（每步一提交，步骤内文件清单见《审计》对应节）：

| 步 | 内容 | 引用 | 验证门 |
|---|---|---|---|
| M1 | public 面删除：`ArWorkMode::kLrr`、`ArRecognitionConfig` 子域、场景真值三类型与三字段、航迹快照/结果帧识别字段、5 个 issue code、聚合头 include、契约白名单两行 | §3.1 | 编译 + contract guards |
| M2 | 执行链胶合删除：`ArController` 识别成员/方法（含 `PrepareLrrPointing`/`IsBetterLrrCandidate`/`ComputeSnrDb`/`RunRecognitionCycle` 及转发）、`ISignalPipeline` 扫描中心覆盖接口、`ScanScheduleResolver` kLrr 分支、会话/composition root/配置校验/patch 搬运/外部输入适配 | §3.2 | `unit::airborne_radar` 全绿 |
| M3 | replay schema 收敛：`airborne_radar_replay.fbs` 识别两表与寄生字段、`airborne_radar_session_replay.fbs` 配置表、codec 全部识别段、工作模式合法上界收紧为 `kStt` | §3.3 | `replay::airborne_radar` 全绿 |
| M4 | 构建与依赖：`recognition/` 8 cpp 移出 `AIRBORNE_ENGINE_SOURCES`、SQLite3 私有链接删除、conan 依赖瘦身确认（全库无其他 SQLite 消费时） | §3.4 | 构建 + 依赖图检查 |
| M5 | 嵌入识别断言的测试段删除（测试本体保留） | §3.5 | 对应测试套件全绿 |
| M6 | 文档与示例：AR 文档四篇移除 kLrr 章节、§1.2 冲突声明涉及的定位变更、`examples/configs/README.md`、`logger_i18n.h` 收敛 | §3.6 | docs 一致性走查 |
| M7 | 等价性测试退役 → **投影契约测试转生**：删除 `ar_rir_recognition_equivalence_test.cpp` 的结论对比部分，将其中 AR `TrackOutputFrame` → `RirTrackFeedEntry` 投影参考实现抽出为 `ar_rir_track_feed_projection_test.cpp`（只锁定字段映射/帧约定/`hit_count` 语义，不再对比识别结论） | 阶段 1 计划 §9 | `integration::cross_domain` 全绿 |
| M8 | 双模块集成契约文档化（阶段 1 计划 §9 预告三件）：① 航迹供给接口时序（AR `Step()` → 投影 → RIR `Step()`）；② 生命周期耦合（`hit_count` 回落=新目标、丢失=保持期语义）；③ **威胁类型字符串契约**（`target_type` 值域 `HIGH_THREAT_FIGHTER`/`LOW_THREAT_TARGET`/其他——2a-A3 驻留优先排序消费该值域，字符串匹配脆弱，须文档化并由 M7 契约测试守护）。落点：RIR `boundaries.md`/`data-flow.md` 供给章节 | 审计 §5.3 | 文档评审 |
| M9 | 段验收：《审计》§9 七条逐条核对（grep 零命中、值域/上界收紧、白名单、AR 测试套件、契约检查通过） | §9 | §9 清单全过 |

## 3. 段 2a：输入面与前置（《边界》决策 10/11）

| 步 | 内容 | 设计要点 |
|---|---|---|
| A1 | `RirCycleInput` 扩展：新增 `RirEnvironmentSnapshotInput`（`propagation_loss_db`、`clutter_power_db`，缺省 0/0）与 RF 入射链路（`std::vector<oneq::electromagnetics::RfIncidentLinkResult>`，缺省空，直接用 common 类型不新造）；`RirSceneTarget` 增 `target_swerling_type`（缺省 0 = Swerling 0）。`RirEnvironmentConfig` **保持空占位**——环境是周期数据非配置，遵守其头注"无死输入"纪律 | 新头文件不新增（扩展 `session/RirSceneTypes.h`/`RirCycleInput.h`），契约白名单不变 |
| A2 | RIR 自建发射波形视图（internal `dwell/RirDwellWaveform`）：从 `RirTransmitterConfig`（prf/pulse_width/bandwidth/frequency_plan）构造 `RfWaveformSchedule` 与驻留接收窗；own emission identity 由 `equipment_id` 构造。干扰聚合与接收窗脉冲计数消费它 | 不新增 AR 依赖；`RfScene` 构造器（common）直接复用 |
| A3 | 驻留指向迁移（internal `dwell/RirDwellPointing`）：`kIdentify` 下从 track_feed 选驻留目标——威胁等级（`target_type` 值域同 M8 契约）优先、斜距次近（`IsBetterLrrCandidate` 语义平移，副本头注来源 commit）；指向 = 驻留目标视线角（`PrepareLrrPointing` 公式平移）。内部状态，不驱动任何外部波束；驻留选择入 `RirCycleResult` 摘要（`association_key` 级，可观测） | `kStby` 清除指向状态；排序确定性（同分取先出现）写入单测 |
| A4 | 校验与 issue codes：`rir.validation.environment_*`（有限/非负）、`rir.validation.rf_scene_*`（incident link 有限性、身份字段合法、重复身份拒绝——对齐 `TryResolveArDetectionCell` 的输入约束） | 入 `RirInputValidation`，拒绝路径复用 `kRejectedInvalidInput` |
| A5 | 测试：单测（校验边界、指向排序与清除、波形视图构造）；集成（新输入接线；**全缺省输入与阶段 1 基线逐位一致**回归门）；`RirCycleInput` 新字段的 replay 兼容确认（周期记录不含输入面，无字节影响） | 缺省兼容回归门为本段核心验收 |

## 4. 段 2b：驻留链路预算（《边界》决策 1/5/6/7/8/9）

| 步 | 内容 | 设计要点 |
|---|---|---|
| B1 | `dwell/RirAntennaPatternRuntime.h`（副本改写 `AntennaPatternRuntime.h`，类型换 `RirAntennaPatternConfig`）：4 主瓣模型（高斯/抛物线/余弦幂/sinc²）、扫描损失、主/旁/后瓣判定、`EvaluateAntennaPattern` 等价函数 | header-only 纯函数；`enable_directional_pattern=false` 时调用方直接走主瓣峰值（缺省=旧行为） |
| B2 | `RirSignalProcessingConfig` 入 `RirHardwareConfig`：四偏置字段（`target_processing_gain_db`/`noise_processing_gain_db`/`clutter_suppression_gain_db`/`jamming_suppression_gain_db`，默认 0）+ 校验（有限、[0, 40] dB）+ `rir.validation.signal_processing_*` | 《边界》§3.2 权威定义；静态配置面（builder 提交，不进运行时补丁） |
| B3 | `dwell/RirDwellLinkBudget`（分项 SINR 账本）：分子 = 回波功率（对数域雷达方程 + 离轴方向图增益 + 传播损耗输入）× 脉压增益 `max(1, B·τ)` × `10^(target_gain/10)`；分母 = 热噪声（接收带宽）×`10^(noise_gain/10)` + 干扰聚合功率 ÷`10^(jam_supp/10)` + 杂波输入功率 ÷`10^(clutter_supp/10)`。干扰聚合为 `TryResolveCellInterference` 语义副本（消费 A1 链路 + A2 波形；**不带**抗 RGPO 减半——ECCM 属 AR，副本头注差异点）。分项结果结构体（echo/noise/interference/clutter W、各增益、单脉冲处理 SINR dB）全量回报 | 带宽口径统一为接收/匹配滤波带宽（决策 4 尾项）；无新输入时账本退化为旧 `ComputeSnrDb` 公式（逐位） |
| B4 | `dwell/RirDwellDetectionGate`（统计级 CFAR）：副本改写 `RadarEquations` 检测子集（`ComputeThreshold`/`MarcumQ`/`ComputeDetectionProbability` Swerling 0-4/`ThresholdDecision`）+ 种子管理（`SetRandomSeed` 语义，种子入 replay 状态）。policy 域新增 `RirDwellDetectionPolicy`：`enabled`（默认 false）、`cfar_pfa`、`min_snr_db`、`min_detection_margin_db`、`dwell_pulse_count` | 判决输入 = B3 单脉冲 SINR + `target_swerling_type` + 驻留脉冲数；输出"驻留回波有效性"，不产点迹（非目标 #4 增补条款） |
| B5 | SNR 口径切换：`RirController::ComputeSnrDb` → 账本驱动（链路预算激活条件：方向图开/增益非零/环境输入非零/链路非空/检测策略开，任一满足即走账本，否则 legacy 直通）；`RirObservationContext` 语义扩展（记录账本分项与检测门结果，供门控与摘要） | **全缺省 = legacy 直通**回归门；激活条件写成纯函数便于单测 |
| B6 | 特征门控升级：`RirRecognitionPolicy` 增 `feature_gating_mode { kLegacySnr6Db=0, kDwellDetectionGate=1 }`（默认 legacy）。检测门模式：RCS 维度有效性由驻留检测门判决驱动（替代硬编码 SNR<6 dB），其余提取器门控逻辑不变 | algorithms.md 登记表同步（RCS 行边界说明改写） |
| B7 | 输出与 replay：`RirRecognitionCycleSummary` 增驻留预算摘要（均值 SINR、检测通过数、驻留目标 key——加性字段）；`rir_replay.fbs` 增对应字段的字节兼容评估（FlatBuffers 加性字段，旧记录可读；roundtrip 测试扩字段） | 三层输出模型不变（摘要属结构化执行结果层） |
| B8 | 文档四件套同步：algorithms.md 登记 +4 行（方向图/账本/检测门/门控模式）与反直觉点（"CFAR 是统计级非 CA-CFAR""增益偏置禁止手填派生量"）；data-flow.md 图加驻留链路节点；boundaries.md F1/F2 节按《边界》§6 已改口径核对 | 《审计》"文档与实现同步"纪律 |

提交切分建议：B1-B2（基础件）→ B3-B5（账本与切换）→ B6-B8（门控/输出/文档）。

## 5. 文件级映射（RIR 侧增量；AR 侧见《审计》§3）

| 动作 | 路径 | 说明 |
|---|---|---|
| 新增 | `src/remote_identification_radar/dwell/`：`RirAntennaPatternRuntime.h`、`RirDwellPointing.h/.cpp`、`RirDwellWaveform.h/.cpp`、`RirDwellLinkBudget.h/.cpp`、`RirDwellDetectionGate.h/.cpp` | 全部 internal，不进 public 头；CMake 源列表同步 |
| 修改 | `include/.../session/RirCycleInput.h`、`RirSceneTypes.h`、`config/RirHardwareConfig.h`、`RirPolicyConfig.h`、`session/RirIssueCodes.h`、`RirCycleResult.h`（摘要字段）、`config/RirSessionConfigValidation.*`、`session/RirInputValidation.*` | 无新 public 头，`check_public_api_boundary` 白名单不变 |
| 修改 | `src/.../runtime/RirController.*`、`session/RirSession.cpp`、`RirReplayFlatbufferCodec.*`、模块 CMakeLists | 口径切换/摘要/replay |
| 删除 | 《审计》§3.1-§3.6 全清单 + `tests/integration/cross_domain/ar_rir_recognition_equivalence_test.cpp`（转 M7 契约测试） | AR 树唯一解锁范围 |
| 新增测试 | `tests/unit/remote_identification_radar/`（方向图/账本/检测门/指向/校验）、`tests/integration/remote_identification_radar/`（场景接线 + 缺省兼容回归）、`tests/integration/cross_domain/ar_rir_track_feed_projection_test.cpp` | 分区注册同步 |

## 6. 测试计划与验证门

1. 每步聚焦测试；段末跑段验证门（2-M：《审计》§9；2a：缺省兼容回归；
   2b：《边界》§8）。
2. **缺省兼容回归门**（2a/2b 核心）：阶段 1 全部 59 例在"全缺省输入/配置"下
   不改一行断言全绿；replay 字节快照（旧记录）解码一致。
3. 检测门 Pfa 统计验证：固定种子蒙特卡洛，虚警率与配置 Pfa 偏差在抽样容差内
   （容差按样本量计算写入测试注释）。
4. 账本分项单独激励：热噪声/干扰/杂波/各增益逐一置非缺省，验证只有对应分项
   变化（防串扰）。
5. 2b 末尾全量 `/completeness-review`（release）；2-M 末尾跑 AR 全量 +
   `integration::cross_domain`（阶段 1"聚焦优先、末尾全量"策略沿用）。

## 7. 风险与处置

| 编号 | 风险 | 处置 |
|---|---|---|
| R-P1 | AR 删除后失去等价参照 | 删除前确认 RIR 59 例 + replay 快照基线全绿；缺省兼容回归门持续守护 |
| R-P2 | replay schema 加性变更破坏旧记录 | M3/B7 的 fbs 变更走加性字段 + 旧记录 roundtrip 测试；工作模式上界收紧单独评估存量记录兼容（`kLrr=4` 值域收缩仅在写入侧） |
| R-P3 | 蒙特卡洛随机性破坏 replay 确定性 | 种子显式管理并入 replay 状态（B4）；同种子同输入同判决写入单测 |
| R-P4 | 干扰聚合/方向图副本与 AR 版漂移 | 副本头注来源 commit 与差异点（如无抗 RGPO）；阶段 3 common 化收敛（计划 R2 扩展） |
| R-P5 | 威胁类型字符串契约脆弱（驻留排序消费） | M8 文档化值域 + M7 契约测试断言投影字段；值域变更须走 boundaries.md 设计变更规则 |
| R-P6 | 检测门/账本激活条件组合爆炸 | 激活条件纯函数化（B5）+ 分项激励测试（§6.4）；条件求值单测覆盖全组合枚举 |
| R-P7 | M1-M3 跨 public/schema/codec 三层的大删除互相纠缠 | 步序 M1→M2→M3 按依赖排序（public → 执行链 → schema）；每步编译 + 对应套件门 |

## 8. 验收标准

1. 《审计》§9 七条全部满足（2-M）。
2. 《边界》§8 七条全部满足（2b；其中等价性测试条目按 M7 转生处置）。
3. 缺省兼容回归门全绿：阶段 1 测试集零修改通过 + 旧 replay 记录解码一致。
4. 投影契约测试守护的供给契约（字段映射/帧约定/`hit_count`/`target_type` 值域）
   有文档锚点（M8）。
5. 全量 release `/completeness-review` 通过。

## 9. 阶段 3 预告（本计划不展开）

雷达方程/方向图评估/检测单元求解/干扰聚合 common 化（副本退役，AR 与 RIR
共享）；AR `PropagationModel`/杂波源 common 化评估；`max_range_m`/
`recognition_dwell_sec` 四域归位（mission 域）；会话配置 replay 与 trace 事件流
评估（《边界》决策引用的阶段 2 评估项中未消费部分顺延）。
