---
Status: active
Last-reviewed: 2026-08-20
Authority: AR 算法登记与实现边界
Answers: AR 用了哪些算法/部件、各自实现到什么地步、边界在哪、哪些刻意不实现
---

# Airborne Radar 算法登记

本文是 AR 算法与部件清单及边界的权威。算法本身的逐步逻辑读代码（`src/airborne_radar/`）；本文只回答
"用没用/到哪步/为什么不做"。模块级边界（dt_sec、环境/RF 事实、输出/失败语义、滤波后端选型原则、
public API 边界）见 [boundaries.md](boundaries.md)。

## 算法登记表

| 算法/部件 | 意图（一句话） | 实现状态 | 证据 |
|---|---|---|---|
| 配置映射与 runtime patch | 四域配置转内部工程配置；运行期变更可回滚提交 | session-wired | [evidence: tests/unit/airborne_radar/ar_session_config_builder_test.cpp] |
| 环境冻结与传播 | pending/active scene 管理，冻结周期环境，传播损失/杂波/大气物理 | session-wired | [evidence: tests/unit/airborne_radar/ar_environment_service_test.cpp] |
| 外部 RF 接入 | 以实际时频发射事实构建前端与 detection-cell 干扰账本，J/N 门控后去真值化观测 | session-wired | [evidence: tests/unit/airborne_radar/ar_rf_front_end_resolver_test.cpp] |
| 扫描和波束控制 | 解析扫描中心、坐标组合、波束增益和波束宽度；TWS/TAS 生效模式下 session 级指向逐周期按扫描表推进 | session-wired | [evidence: tests/unit/airborne_radar/ar_signal_scan_schedule_test.cpp] |
| STT 指定航迹跟随指向 | 外部只指定目标，STT 波束指向由指定航迹位置换算（优先级：显式 dwell > 航迹 > scan_center） | session-wired | [evidence: tests/unit/airborne_radar/ar_stt_track_follow_test.cpp] |
| 指定目标生命周期回退 | 指定航迹未确认/丢失时 STT 自动回退 TWS；限时指令窗口耗尽未捕获时作废（kAcquisitionTimeout），经 L2 结果/L3 视图/生命周期事件暴露 | session-wired | [evidence: tests/unit/airborne_radar/ar_stt_track_follow_test.cpp] |
| 统一物理探测与工程 RF 干扰链 | 实际发射→echo→incident RF→前端账本→检测单元→判决的单一物理链 | session-wired | [evidence: tests/unit/airborne_radar/ar_signal_pipeline_test.cpp] |
| 数据关联 | 位置量测、协方差和 track seeds 的 LAPJV assignment | session-wired | [evidence: tests/unit/airborne_radar/ar_signal_association_test.cpp] |
| 航迹过滤与生命周期 | KF/IMM(KF) 更新航迹、missed detection、确认/丢失/回收、反欺骗抑制 | session-wired | [evidence: tests/unit/airborne_radar/ar_track_filter_test.cpp] |
| 战术协调 | 威胁评估、LPI、ECCM、关联压力补触发、状态清理 | session-wired | [evidence: tests/unit/airborne_radar/ar_decision_layer_test.cpp] |
| 控制归约 | proposal 冲突、保持窗口、冷却和下一周期控制配置 | session-wired | [evidence: tests/unit/airborne_radar/ar_tactical_coordinator_test.cpp] |
| 专项序列验证 | 公开 Session 边界六类跨周期序列 | session-wired | `tests/consumer/batch_validation/ar_batch_validation.cpp` |

## 配置映射、运行期提交和回滚

- **意图**：AR 的 public config 是语义配置，signal pipeline 使用内部工程配置；`MapSessionToExecution` 在
  构造时初始化 runtime state，runtime patch 暂存到 `pending_runtime_state`。
- **实现边界**：
  1. 真正提交发生在下一次 `RunCycle` 前：校验输入 → 捕获 context/pipeline/environment/controller 四类快照 →
     同步 pending runtime state 与环境 scenario → 任一失败恢复全部快照并返回 `kRuntimePreparationFailed`。
  2. 提交成功并完成执行后才调用 `FinalizePendingRuntimeConfig`；唯一非执行例外是 `kSensorPoweredOff`：先
     恢复本周期四类快照撤销消费，再重新对齐已验证配置并 finalize。
  3. runtime mapper 必须校验合并后的最终候选配置；合法工作模式与非法波束字段混合时整个 patch 原子拒绝。
  4. `MutableArContext` 快照是强所有权边界：opaque envelope 只能由捕获实例构造，foreign owner、
     moved-from 空载荷、同地址重用旧 envelope 均在 mutation 前拒绝，不保留 schema compatibility 或明文
     字段回退路径。
- **反直觉点**：环境域当前没有 execution-only 字段，故不维护同型公开 Model 类型或恒等 mapper；只有先证明
  存在 execution-only 字段时才可新增内部执行配置。
- **证据**：[evidence: tests/unit/airborne_radar/ar_runtime_patch_mapper_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_mutable_context_runtime_state_test.cpp]
- **证据**：[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp]

## 自然环境、传播和外部 RF 事实

- **意图**：`EnvironmentService` 维护 pending/active scene，`BeginCycle` 到达时冻结快照，controller 和
  pipeline 在同一周期读取同一份 `EnvironmentSnapshot`。
- **实现边界**：见 boundaries.md 环境/RF 事实边界——`EnvironmentScenarioConfig`/`EnvironmentSnapshot` 只承载
  自然环境事实，AR 不从场景推导干扰，外部 RF 由 `RfEmissionFrame` 独立输入。
- **反直觉点**：大气物理附加损耗由信号层按每目标真实几何计算，环境层不重复；`SpaceWeatherContext` 当前
  是死输入，故不对外开放。
- **证据**：[evidence: tests/unit/airborne_radar/ar_environment_config_contract_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_propagation_model_test.cpp]

## 扫描调度、坐标和波束控制

- **意图**：目标先解析到当前雷达参考框架，再计算 range；`ScanScheduleResolver` 将周期/扫描范围/dwell
  center 转成扫描指向，`BeamControlResolver` 给出 one-way gain 与有效波束宽度。TWS/TAS 生效模式下
  session 级指向逐周期按扫描表推进（波束动画，见 boundaries.md 扫描动画接线），pipeline RF v1 回退
  路径经 `ApplyScanScheduleToRuntimeConfig` 使用同一扫描相位。
- **实现边界**：
  1. `ArMissionConfig::orientation.scan_center_deg` 是基础扫描中心 public source of truth；policy 不再保留
     默认中心或 replay-only 副本。runtime patch 的 `dwell_center_deg` 是当次驻留偏移，最终指向为"基础中心 +
     偏移"；replay 分别保留两者。
  2. 机体稳定直接使用扫描中心和 dwell；惯性/对地稳定先用本周期平台姿态和实际安装角反解挂架指向，再同时
     用于 ECEF 发射 boresight 与目标方向增益——平台转动不得使波束随机体漂移。
  3. 天线波束宽度按轴独立解析（commanded > nominal > 由波长/孔径推导）；三级均无有效值时返回 0（不
     抛异常、不拒绝），调用方须保证 commanded 或 nominal 至少其一有效。孔径字段属于可回放硬件配置，
     replay 必须保留 `antenna_length_m`/`antenna_width_m`。
  4. 扫描范围由 `mechanical/electronic_scan_limits_deg` 交集决定，`scan_center_deg` 仅作限位非法时的
     回退中心；扫描表按 `cycle_index % pattern.size()` 取波位，STBY 返回零位、STT 返回扫描中心、
     TWS/TAS 返回波位序列（TAS 步长减半，`prefer_dense_tas_sampling` 再减半）。
  5. **扫描内核为 common 单源**（`src/common/radar/ScanScheduleRuntime.h`）：波位序列构建与轴步长解析由
     AR/RIR 共用，模块侧只保留模式语义与指向消费接线（AR `ScanScheduleResolver` 委托该内核）；
     RIR 驻留调度器（空闲扫描策略）与 AR 同一口径。
  6. ECCM 措施只改变下一次成功发射/接收的实际硬件状态（频率捷变改 carrier/tuning、rejitter 改脉冲时序、
     旁瓣对消/自适应波束改方向增益/零陷、烧穿改发射功率/脉冲能量），不直接改写关联、滤波或生命周期参数。
- **反直觉点**：公开发布的 `emission_frame` 是 base 发射身份，旁瓣对消/自适应波束只作用于接收态
  `receiver_state.antenna`，不进公开发射方向图（见 data-flow.md 输出归属边界）。
- **证据**：[evidence: tests/unit/airborne_radar/ar_beamwidth_resolution_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_orientation_utils_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_rf_session_test.cpp]

### STT 指定航迹跟随与自动回退（方案 A）

- **意图**：STT 模式下外部只指定目标（`ArRuntimeConfigPatch::designated_external_target_id`），
  波束指向由 AR 自身航迹推导（上一周期 `TrackOutputFrame` 的雷达局部位置 →
  `TryTrackPositionToLookAnglesDeg`），不再要求外部提供角度。
- **实现边界**：
  1. 指向来源优先级（冻结）：显式 dwell 非零 > 指定航迹 confirmed > scan_center 回退，
     由 `ResolveSttTrackFollowingPointing` 单一权威解析（纯函数，单测覆盖优先级矩阵）。
  2. 注入点唯一：`ArSession` prepare（`ResolveMountFrameBeamPointing` 的 scan_center 输入），
     经冻结指向链路（`RfV2DetectionContext::beam_pointing_deg`）同时驱动发射/接收/增益/检测单元；
     TWS/TAS 生效模式（含 STT 回退）下该注入点另按扫描表逐周期推进（见扫描调度节）。
  3. 指定状态是会话级状态（`RuntimeConfigState`），不进 pipeline 执行配置；随 patch
     原子暂存/提交/回滚。
  4. 生效模式为每周期派生（latch-free）：指定航迹非 confirmed → 回退 `kTws`（报告），
     指向按回退分支推进（TWS 扫描表逐周期推进，与 session 级 TWS 一致）。
  5. 回退事件暴露：L2 `ArCycleResult` 新字段（`effective_work_mode`/`designation_active`/
     `designated_target_id`/`designation_reverted_to_tws`/`designation_revert_reason`，
     每周期状态指示）；L3 debug view 转写；`ArTrackLifecycleRecorder::kDesignationDropped`
     在转换沿产生；replay 周期记录与 patch 记录同步。
  6. 限时锁定指令（`designation_duration_cycles`，见 boundaries.md 第 7 条）：指定指令
     可带捕获窗口。窗口自指令生效后首个成功周期起算（失败/关机周期不消耗窗口），
     窗口内捕获 confirmed 航迹 → `kAcquired`（此后不再受窗口约束，丢失按既有回退语义）；
     窗口耗尽仍未捕获 → `kExpired`（指令作废，作废沿 = `kPending` → `kExpired` 转移沿，
     即截止后首个成功周期报告 `kAcquisitionTimeout`，其后指定清零、回到扫描）。
     生命周期阶段是**会话级跨周期状态**（`RuntimeConfigState`，随 patch 原子暂存/提交/
     回滚；阶段推进仅在本周期成功完成后落定），与"生效模式 latch-free 派生"正交：后者仍逐周期
     从"已提交配置 + 最新航迹帧"派生，前者只回答"窗口是否已关闭/作废"。
- **反直觉点**：
  1. 指向用上一周期航迹后验位置（一周期滞后近似），目标机动时指向滞后一拍；
  2. 显式 dwell 覆盖时 `designation_active == false` 但不构成回退（`reverted == false`）；
  3. 目标在扫描限位外时指向被 clamp 到边界（`ComputeMountFrameBeamPointing`），离轴增益
     损失可致连续失配 → lost → 自动回退（期望连锁行为）。
- **证据**：[evidence: tests/unit/airborne_radar/ar_stt_track_follow_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_runtime_patch_mapper_test.cpp]
- **证据**：[evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp]
- **证据**：[evidence: tests/consumer/batch_validation/ar_batch_validation.cpp]

## 统一物理探测与工程 RF 干扰链

AR 的核心算法。探测门限属于 **policy，不属于 hardware**：`ArPolicyConfig::detection` 统一承载
`minimum_snr_db`、`pfa`、`pulse_count` 和 `minimum_detection_margin_db`；hardware 描述发射机、接收机、
天线、波形、量测等物理能力与装备级信号处理增益偏置（四偏置缺省 0 dB）。探测意图语义档位
（如 `profiles::kDetectionPriorityDetection`）只翻译为这组 policy 参数。AR **不再提供** heuristic
detection toggle 或启发式 pass。

- **意图**：单一物理链——emission → echo → incident RF → front-end ledger → detection cell → decision——
  而非"把所有外部发射功率加到一个周期总噪声"。
- **实现边界（6 步物理流水线）**：
  1. **实际发射与目标回波**：每周期解析实际频率/功率/脉冲能量/带宽/PRF/抖动/波束/驻留；每个目标由 AR
     自有双程雷达方程计算 echo（含本周期 transmit/receive gain、双程传播、大气、RCS、处理前损耗），目标
     不得转换成公共单程 emission。
  2. **外部入射 RF**：冻结 `RfSceneFrame` 中的发射经公共单程链路到达 AR 接收设备；platform/equipment 路径
     决定设备级 co-site isolation，不能用零距离自由空间公式或单一实体 ID 猜测自扰。当前单周期模型
     **不实现** T/R blanking。同平台 `range_m=0` 时的去真值化扰动钳制（`kMinObservableRangeM`）发生在
     interference observation 解析阶段（见下方 interference observation 通道），不在前端账本。
  3. **前端账本**：在实际接收方向图和预选器下聚合整个前端带宽的输入功率；超过
     `maximum_linear_input_power_w` 时本周期仍是物理执行成功，但输出 `receiver_saturated` impairment，
     不生成目标量测或虚假 interference observation。
  4. **检测单元账本**：在 range/Doppler/beam 以及实际 time-frequency window 内为每个候选目标分别记录
     echo、热噪声、杂波和未分辨外部 RF。噪声型干扰按其 PSD/接收滤波响应/活动占空进入单元；带外、错时
     或被零陷抑制的贡献为零。**首期不把压制噪声解释成虚假目标**。欺骗发射（kPulseTrain）经同一时频重叠
     机制进入，PRI 间隙内 activity 为零；`enable_anti_rgpo_leading_edge=true` 时其有效干扰功率乘 0.5。
  5. **处理后判决**：匹配滤波、脉冲压缩、相参/非相参积累等 processing gain 只属于 AR；统一计算
     `SINR = echo × pulse_compression_gain × 10^(target_processing_gain_db/10) /
     (thermal × 10^(noise_processing_gain_db/10) + clutter / 10^(clutter_suppression_gain_db/10) +
     interference / 10^(jamming_suppression_gain_db/10))`，由 policy 的 Pfa/Swerling/积累模型/最小 margin
     得到 Pd；Monte Carlo 只采样检测事件。四增益偏置缺省全 0 dB，逐位等于保守账本；
     脉压与积累增益永远自动派生，不得手填进偏置。
  6. **可靠性裕量门限**：Monte Carlo 判决后若 `snr_db < min_detection_margin_db` 则强制 `detected = false`；
     该门限作为后验安全网独立于 Pd/Monte Carlo 路径。`detection_margin_db` 输出语义为相对裕量而非原始 SNR。
- **反直觉点（两个噪声基准，不得混用）**：
  - **前端 J/N 门控基准**：`k·T·transmitter.bandwidth_hz·noise_figure`（发射带宽），判断某条外部
    emission 是否过 `interference_observation_jn_gate_db` 被记录为可观测干扰。注意此处用的是
    `transmitter.bandwidth_hz`（发射带宽，默认 4.5 MHz），**不是** `preselector_bandwidth_hz`（宽带前端
    预选器带宽，默认 20 MHz，仅用于 `receiver_state.bandwidth_hz`，不进入 J/N 门热噪声）。
  - **检测单元 SINR 基准**：`k·T·matched_filter_bandwidth_hz·noise_figure`（单 range-Doppler-beam-time-frequency
    cell），分项进入分母 `thermal × noise_bias + clutter / clutter_suppression +
    interference / jamming_suppression`（偏置缺省 0 dB 时即 `thermal + clutter + interference`）。
  - 二者带宽口径不同是有意的：前者回答"前端能否察觉这个干扰源"，后者回答"这个 cell 的信干噪比是多少"。
    不得合并或互相替换。
- **反直觉点（饱和与 ECCM 是降级而非触发）**：前端饱和时本周期输出 `receiver_saturated` impairment 并
  **跳过** interference observation 解析，`EccmEvaluator.Evaluate()` 因观测为空而返回未激活。这是有意语义——
  前端被烧穿后接收机无法可靠测向，强行生成 J/N 门控观测会输出不可信 AoA/RF，故饱和走单独的结构化降级
  路径，不经 ECCM 控制闭环。ECCM 的旁瓣对消/自适应波束触发只依赖未饱和但过 J/N 门的观测，发生在干扰使
  SINR 恶化但前端仍线性的区间。
- **反直觉点（航迹影响边界）**：压制干扰只能通过量测存在性和量测协方差间接影响 association/Kalman/IMM/
  lifecycle；按干扰类别、ECCM profile 或预计算受扰布尔值直接缩放门限、过程噪声、失配容忍或生命周期计数
  的路径都不属于目标架构。
- **单一频率来源**：探测、传播、天线波长和物理 RCS 全部消费当前有效的 `transmitter.frequency_hz`；
  `RcsPhysicsConfig` 不再提供独立频率或隐式继承规则。
- **interference observation 通道**：与目标 track 分离，只在独立能量/J/N 门通过后生成，不能由场景标签或
  ECM source ID 直接生成；输出估计 bearing/frequency/bandwidth/waveform class 和不确定度；不包含 truth
  equipment/emission ID 或"敌方干扰意图"；interference-limited/masked/saturated 是接收机事实，不是外部
  输入字段。
- **证据**：[evidence: tests/unit/airborne_radar/ar_signal_detection_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_detection_cell_resolver_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_rf_front_end_resolver_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_interference_observation_resolver_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_signal_pipeline_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_eccm_evaluator_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_profile_constants_test.cpp]

### 欺骗干扰三层反制

AR 在接收链的三个层次主动反制欺骗干扰（kPulseTrain），均不读 ECM `EcmDeceptionMode` 真值：

1. **观测层**：`ArInterferenceObservationResolver` 从过 J/N 门的 kPulseTrain 观测中，在接收波束宽度和接收
   频率分辨单元内建立连通分量（≥2 同束同频率成员），对成员设置 `deception_class=kLikelyFalseTarget` 并逐
   成员生成内部 `ArDeceptionMeasurementCandidate`。候选携带 source observation/emission provenance 与可观测
   残差；斜距/径向速度写入前按 `RadarEquations::ComputeRangeErrorStdDev` 派生标准差叠加确定性零均值噪声
   （种子由 cycle index + receiver equipment id 派生，replay 可复现）。残差超门限时 RGPO 叠加
   `ΔR = 0.5·c·delay`（≥100 ns），VGPO 叠加 `Δv = -0.5·λ_ref·Δf`（|Δf|≥1 kHz），使候选量测落在欺骗后
   apparent 位置；门限内保持几何值。
2. **ECCM 决策层**：`EccmEvaluator` 按与 ECM 物理匹配的接收端残差路由 RGPO/VGPO/假目标，达阈值分别生成
   `anti_rgpo_score`/`anti_vgpo_score`/`anti_false_target_score` 三项反欺骗提案。
3. **信号层**：`DeceptionMeasurementGenerator` **独立于反制开关**，只消费 resolver 候选（每成员一条），合成
   带 `classified_as_false_target` 的假目标量测注入 `track_measurements`；反制开关只在
   `TrackLifecycleManager::PromoteState` 控制 tentative→confirmed 的抑制。

- **证据**：[evidence: tests/unit/airborne_radar/ar_deception_eccm_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_deception_measurement_generator_test.cpp]
- **证据**：[evidence: tests/integration/cross_domain/multi_model_scenario_test.cpp]

## 数据关联、质量指标和航迹生命周期

- **意图**：探测成功后 `DataAssociationEngine` 把量测和已有 track seeds 关联；public `distance_gate_sigma`
  以标准差倍数表达，内部 assignment 门限统一派生为 `distance_gate_sigma²`。
- **实现边界**：
  1. 生产链路只有一条默认路径：`FullMahalanobisDistanceMetric` + `DenseCostHypothesiser` + `LapjvSolver`（common 单源 `src/common/optimization/LapjvSolver` 适配），
     没有 factory、runtime config 选择或用户可替换接口。`MahalanobisDistanceMetric` 等保留类仅用于局部测试
     和算法对比，不代表第二条生产实现。
  2. association public 配置不再暴露 `unassigned_cost`、启用 hint 或第二套 sigma hint；策略档位如需保持
     既有内部代价行为，必须在 Builder 中以平方根映射到唯一 sigma。
  3. 质量指标（prior/detection/matched/new/missed/match rate/mean-p95 cost）继续用于结果输出和一般质量退化
     诊断，但不得被解释为工程干扰观测，也不得生成 J/N、jammer attribution 或物理 ECCM 触发；最多驱动显式
     标记的保守 `quality_fallback`，且不能声称已探测到干扰源。
- **反 VGPC 加速度限幅**：在 `TrackLifecycleManager` 内实现（而非无状态的 `TrackFilter`），因为限幅需要跨
  周期上一周期速度，只有持久化 `tracks_by_key_` 持有该状态。`enable_anti_vgpo_acceleration_bound=true` 时对
  已存在航迹在 Kalman/IMM 更新之后按 `max_acceleration_mps2 * dt` 裁剪速度各分量相对上一周期变化，新建
  航迹豁免。
- **反假目标鉴别**：在 `TrackLifecycleManager::PromoteState` 内实现，`enable_anti_false_target_discrimination=true`
  且量测被标为疑似假目标时不把 tentative 航迹晋升为 confirmed。
- **反直觉点**：`SyncRuntimeTuning` 用整体赋值 `config_ = lifecycle_config` 同步运行期生命周期配置，而非
  手工逐字段列表。安全性来自 `track_pool_thread_safety_mode` 属于 `LifecycleConfigSignature`（变化触发重建
  而非同步），管理器从不读取该字段即便被覆盖也无副作用。
- **证据**：[evidence: tests/unit/airborne_radar/ar_signal_association_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_track_lifecycle_test.cpp]

## 决策帧、威胁评估、LPI 和 ECCM

- **意图**：`ArController` 从 signal pipeline 得到 `DecisionInputFrame` 后调用 decision engine（默认
  `TacticalCoordinator`），输出目标分类、proposal、selected mode 和 decision summary。
- **实现边界**：
  1. 工程 ECCM 的物理来源只能是统一物理探测链定义的接收机 interference observation；一般 association
     quality 异常只能进入不带干扰归属的 `quality_fallback`。
  2. LPI 激活时输出两个参数化 proposal：功率比例范围 `0.3–0.8`，驻留比例 `clamp(0.5 + 0.5 * power_scale,
     0.65, 0.90)`；首批不自动启用 LPI beamforming。
  3. ECCM 只消费接收机 interference observation；烧穿评分达阈值后输出 `clamp(1.0 + 0.25 *
     burnthrough_gain_score, 1.0, 2.0)`，实际 proposal 必须落在 `(1, 2]`。
  4. 工程压制路径不按 truth technique 或 source ID 改写 proposal 优先级。
  5. selected mode：ECCM 激活→`kProtectedEmission`；否则 LPI 请求降暴露→`kThreatResponse`；其余→`kBaseline`。
- **反直觉点**：默认决策结果始终把目标分类回填到 track output frame，并在每个成功周期推进
  `TacticalStateStore`、计算下一周期 internal baseline。外部响应只覆盖 LPI/ECCM proposals，不替换威胁分类
  路径；外部长期生效后，内部 baseline 仍能立即接管。
- **证据**：[evidence: tests/unit/airborne_radar/ar_tactical_coordinator_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_eccm_evaluator_test.cpp]

## 控制归约和跨周期反馈

- **意图**：控制决策分两阶段——原生归约（`ControlReducer`/`ControlCommandMapper` 把 proposals 归并为唯一
  `ArControlProfile`）和外部覆盖（`SubmitExternalDecision` 整包替换）。
- **实现边界**：
  1. hold/cooldown 仅约束原生路径；LPI/ECCM 的保持窗口和冷却可分别配置，四个周期数默认均为 `0`（关闭窗口，
     下一成功周期可立即切换）。runtime patch 缩短窗口时当前剩余立即收紧为 `min(旧剩余, 新上限)`，改为 `0`
     取消当前窗口。
  2. 生存性规则：同时出现烧穿和 LPI 降功率时，最终功率比例提升到至少 `0.85`；LPI 功率要求 `(0,1]`，驻留
     `[0.25,1]`，ECCM 烧穿 `(1,2]`。
  3. 外部覆盖**完全绕过 hold/cooldown 和冲突解决**，对 profile 字段拥有完全控制权；提交的 profile 经字段
     范围校验（功率/驻留/烧穿比例、hop phase 合法性、无 NaN/Inf），不通过立即返回 `kInvalidProfile`，不进入
     pending 状态、不写入 trace。
  4. 外部覆盖无 cycle/batch 时序耦合——外部模块通过消息总线通信固有延迟使时序匹配不切实际，雷达在下一个
     `StepWithResult` 准备阶段应用。
  5. `ControlDirective`/`ControlDirectiveType`/`ControlDirectiveSource`/`TacticalProposal` 已收口为内部实现细节，
     不再暴露于公共 API。directive→profile 分类单一权威来源是 `ControlReducer::IsLpiDirective`/`IsEccmDirective`/
     `IsValidDirectiveValue` 静态方法；`kCount` 哨兵仅用于编译期/测试期穷尽性检查，不得作为真实意图传递或
     序列化。
- **反直觉点（proposal 消费边界）**：内部/外部 LPI/ECCM proposal 在下一次**成功发布实际 emission**时消费。
  输入拒绝或关机保留 proposal；实际 emission 一经内部发布即提交 transmitter profile、hop/PRI phase 和相关
  计数，后续接收侧失败不得让同一 proposal 再次控制下一次发射。输入验证失败不消费 control profile、reducer
  计数器或 internal baseline；发射已发布后的接收机 impairment 是已完成物理周期，必须返回实际 emission 且
  不回滚已提交发射事实。
- **证据**：[evidence: tests/unit/airborne_radar/ar_control_directive_matrix_test.cpp]
- **证据**：[evidence: tests/unit/airborne_radar/ar_core_controller_test.cpp]
- **证据**：[evidence: tests/replay/airborne_radar/ar_rf_trace_session_test.cpp]

## 滤波后端评估表

AR 生产后端是 Joseph 形式 KF；IMM 是包裹 KF 的多模型融合层（见 boundaries.md 选型原则）。以下候选
**不**与 live 生产链并列：

| 候选 | 当前结论 | 否决依据 |
|---|---|---|
| EKF | 否决接入 AR | AR 使用笛卡尔位置量测，`H=[I3\|03]` 为线性模型；EKF 退化为 KF 并增加 Jacobian 开销。common 模板（`EkfFilter`）保留给非线性量测模块评估。 |
| SRIF | 暂不接入 AR | 信息形式优势与当前已有先验初始化的 AR 路径不匹配；common 模板（`SrifUpdater`）仅作候选资产。 |
| UDKF | 否决接入 AR | 500 周期 CV 与病态初始化表征中，未证明相对 Joseph KF 的正定性、对称性或条件数收益；common 模板（`UdkfUpdater`）仅作候选资产。 |

可接受的"智能"形态（仍保持人工决策）：只读 NIS 诊断（用 `KalmanUpdateResult` 的 `innovation`/
`innovation_covariance` 计算 NIS 报告模型失配，不触发自动切换）和基于任务剖面先验的 IMM 配置。

## 专项序列验证边界

`batch_validation::airborne_radar` 在公开 Session 边界执行六类跨周期序列：同 RCS 双目标交叉、干扰加入/清除、
TWS→STT→TWS、关机恢复、无效输入恢复和混合非法 runtime patch。

1. 场景必须显式启用物理探测、物理 RCS 与 `physics_mix_ratio=1`。
2. 硬契约（影响退出码）：目标身份连续、patch 原子性、非执行周期 lifecycle 静默、failure marker 后完整 replay。
3. warning/error 观测项（不影响退出码）：距离/RCS 等物理趋势。
4. 场景 ID 与运行方式由 `tests/consumer/batch_validation/README.md` 维护。

## 非目标（刻意不实现的算法）

1. **Heuristic detection toggle / 启发式 pass**：探测统一走物理链（emission→echo→...→decision），不再提供
   启发式旁路。
2. **多个 association 生产路径**：当前只有 `FullMahalanobisDistanceMetric` + `DenseCostHypothesiser` +
   `LapjvSolver`（common 单源适配）一条；只有未来出现至少两个已接入、有测试覆盖且语义稳定的实现时，才允许新增用户可见配置
   选择算法。
3. **EKF/UDKF/SRIF 接入 AR 生产链**：见上方评估表，AR 笛卡尔位置量测为线性模型，EKF 退化为 KF，UDKF/SRIF
   未证明收益。
4. **在线残差驱动的自动后端切换**：唯一可选框架是策略配置中的 `enable_imm_lifecycle`；KF 仍是每个模型分支
   的生产更新器（详见 boundaries.md 选型原则）。
5. **T/R blanking**：当前单周期模型不实现，不得把 co-site 衰减描述为发射脉冲消隐。
6. **压制噪声解释成虚假目标**：首期检测单元账本不把压制噪声解释为虚假目标，只走 SINR/量测协方差间接影响。
