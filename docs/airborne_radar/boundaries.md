---
Status: active
Last-reviewed: 2026-08-07
Authority: AR 模块级边界、非目标与设计变更规则
Answers: AR 有哪些模块级禁令与边界、哪些非目标、配置/环境/校验/滤波的特殊语义、文档变更规则
---

# Airborne Radar 模块边界

本文承载 AR 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（统一物理探测链的两
噪声基准、饱和降级、检测门限归属、欺骗 ECCM 路由等）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

AR 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/trace/replay/decision DTO 门面。`EnvironmentService`、
   `SignalPipeline`、`ArController`、`MutableArContext`、tracking lifecycle 和战术决策部件不通过 public
   header 暴露。
2. `ArSessionConfigBuilder` 是薄封装（整域赋值 + `Build()` 返回副本）；语义档位是 `ArProfileConstants.h`
   中的预定义结构体常量，不承担 leaf setter 或隐式 validation。
3. AR 输出遵守三层模型：系统输出（`TrackOutputFrame`）、结构化执行结果（`ArCycleResult`）、调试/生命周期/
   replay 视图分离。
4. decision seam 是同进程步间 observation/response，是唯一的 public 决策扩展点。

## dt_sec 校验边界（反直觉，勿按"四模块一致"补齐）

`ValidateArCycleDeltaTime`（`double` 类型）对 `dt_sec` 仅校验有限性 + 正值，**故意不含** EOS/SBIRS 的
`dt_sec ≤ 10/frame_rate_hz` 上界。

1. AR 配置中没有 `frame_rate_hz` 概念——主动雷达的周期节拍由 PRF、驻留时间、航迹更新率等雷达域量在
   各自子链路里约束，而非成像帧率。
2. dt 的合理性由 PRI/rejitter、emission 调度和 signal pipeline 消费链把关，不适用一个全局 frame_rate 上界。
3. 该差异已由 `ArCycleInput.dt_sec` 为 `double`（其余模块 `float`）、AR 无 frame_rate 字段、本校验链实测
   三方共同固化。

不得为"四模块一致"给 AR 加 frame_rate 上界。

[evidence: src/airborne_radar/session/ArInputValidation.cpp]

## 环境/RF 事实边界

`EnvironmentScenarioConfig`（`ArSessionConfig.environment`）与 `EnvironmentSnapshot` 只承载自然环境事实：

1. **不包含** jammer、J/S、J/N、干扰检测布尔值或预计算接收功率。AR 的外部 RF 输入是独立的
   `oneq::electromagnetics::RfEmissionFrame`，通过 `ArCycleInput::interference` 直接传入。空 frame 表示无
   外部 RF；非空 frame 必须与 AR 周期号、绝对窗口起点和时长完全一致，否则整个周期拒绝。
2. AR **不从环境场景推导干扰**。调用方只提供实际发射事实（发射 platform/equipment/emission 身份、ECEF
   运动学、天线、极化和参数化波形）；接收功率、PSD、J/N、饱和和观测质量全部由 AR 当前接收机链计算。
3. 大气物理附加损耗由信号层 `ComputeTargetSpecificAtmosphericLossDb` 用**每个目标的真实几何**计算，
   不在环境层重复计算；环境层硬编码几何的死计算已移除，`EnvironmentSnapshot` 不再承载
   `atmospheric_physics_loss_db` 字段。
4. AR **不保留** legacy jammer DTO、技术类别、J/S 摘要、欺骗/转发适配层，也不直接获取 ECM 的
   `EcmDeceptionMode` 真值。外部欺骗发射与压制发射经同一 `RfEmissionFrame` 路径进入接收前端。

拒绝暴露 `SpaceWeatherContext`（年积日、F10.7、地磁 Ap、仿真 Unix 时间戳）：当前 GTD7 大气模型退化为
ISA 标准大气，这些字段全部未被消费，属未接入的死输入。仿真时间统一以 `ArCycleInput::cycle_start_time_s`
为唯一来源。

[evidence: tests/unit/airborne_radar/ar_environment_config_contract_test.cpp]
[evidence: tests/unit/airborne_radar/ar_environment_service_test.cpp]

## 输出/输入校验与失败行为

`ArSession` 和 `ArController` 都有明确的失败语义：

1. **校验层归属（COMMON-OQ-9 收敛，2026-08）**：公共路径入口校验在 `ArSession`
   （`ValidateArCycleInput` 含外部运动学坐标系转换，控制器输入面不含 platform/targets 原始
   数据，无法下移）；运行期校验唯一化在 `ArController::RunOnce`（会话层对同一输入的二次
   校验已删除），拒绝时明细经出参直通并装配进最终周期结果；运行期执行失败透传真实
   `abort_reason`（校验拒绝为 `kRejectedInvalidInput` + 细粒度明细），不写死替换。
2. cycle input 校验失败时不执行 pipeline，`ArCycleResult` 携带 validation issues 与显式 abort
   reason `kValidationRejected`（保留 replay/trace 数值语义）。
3. **非执行周期统一不复用（五模块统一规则）**：`Step()` 与 `ArCycleResult.output_frame`
   返回默认空帧（`cycle_index==0`、空 tracks/emission），不论是否存在上一有效输出。调用方仅凭
   `Step()` 返回值即可判定本轮无新航迹。状态判断统一走 `StepWithResult().status`
   （`kRejectedInvalidInput`/`kPoweredOff`/`kRejectedExecution`）。
4. controller 内部 `last_cycle_reused_previous_output` 仅是 RF 接收/检测侧跨周期状态机的簿记标志
   （捕获/恢复 snapshot 用），不进入 public `ArCycleResult`，也不等于公开输出帧被复用；不得据此推断
   `Step()` 会回传历史航迹。
5. signal pipeline abort 时不会发布合成的最新输出。
6. 电源状态单源：`ArSessionConfig::sensor_enabled` 是唯一来源（mission 域无电源字段），运行时电源唯一入口
   为 `ArRuntimeConfigPatch::has_sensor_enabled`。
7. 设备关机是已接受的非执行配置边界：撤销周期副作用后 finalize 关机配置，并保留外部决策等待下一成功周期。

[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::RejectedCycleDoesNotReusePreviousOutput]
[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::StepReturnsEmptyCurrentFrameOnRejectedInput]

## 输出边界要求

1. track output 保持系统侧航迹语义；名称、仿真便利信息或调试归因不能替代 stable association key/status。
2. `ArCycleResult` 由 `ArSession` 汇总 controller、context 和 pipeline 状态，承载执行状态、validation issues、
   abort reason、submitted commands、control profile、association quality metrics、decision observation 和已采用
   来源 provenance；完整 proposal、待消费响应与 reducer 计数只属于内部 `ArReplayCycleRecord`，不进入 public
   业务结果。
3. query/debug/lifecycle/replay 是诊断辅助，不是用户扩展 signal pipeline 的入口；决策 SPI 不拥有输出结构，
   也不能绕过内部 output adapter 写系统输出。

### 三写约束（abort_reason + issues + 日志）

AR 所有中止路径遵守 `session_contract.md` 规则 9 的三写模式与规则 14 的统一问题列表模型：

1. **结构化信号**：`ArCycleResult.abort_reason`（粗粒度枚举）。
2. **结构化诊断**：`ArCycleResult.issues`（`ArIssueList`，细粒度 code 如 `"ar.sensor_powered_off"`、
   `"ar.validation.invalid_cycle_delta_time"`；条目携带 `phase` 来源标签与可选定位）。
   本模块 code 全集单一事实来源：`include/1q/airborne_radar/session/ArIssueCodes.h`（规则 14c）。
3. **人读日志**：`PROJECT_LOG_ERROR`。

`ArCycleResult` 只承载单一问题列表 `issues`：输入校验问题（`phase=kInputValidation`）与执行诊断
（`phase=kExecution`/`kOutputContract`）同列表承载，不设 `validation_issues`/`has_validation_error`
平行字段。校验拒绝时校验问题本身就是 error 级诊断（规则 9 写二），不再附加粗粒度条目。

三写由 `ArDiagnosticUtils::RecordAbort` 统一执行（phase 由中止原因推导），在 `ArSession` 的周期
装配路径中调用。

**正常周期的按目标排除诊断（规则 13b）**：正常执行周期（`status == kCompleted`）中被 SNR 检测门
排除的目标（`min_snr_db` / `min_detection_margin_db` 任一未过；距离/方向图衰减隐式并入 SNR）写
`kInfo` 级 `ArIssue`（code `"ar.target_snr_below_threshold"`，message 携带 `target_id` 与
`snr_db`/`range_m`/门值/偏轴角，phase=`kExecution`），**不属于三写**（三写仅约束中止路径，规则 9）。
**门内归因（规则 13b 归因条款）**：SNR 门为聚合门（距离/波束偏轴/噪声底/RCS 折入单一门限），
`ArIssueCause` 给出机器可读主因——按各因素相对参考状态（1 km 距离、主瓣中心增益、1 m² RCS、
热噪声底、零传播损耗）的损失 dB 判定，损失最大者为 `kDistanceLimited` / `kBeamLimited` /
`kNoiseLimited` / `kRcsLimited`（传播损耗并入距离项）；无法判定为 `kUnknown`。
诊断不改变 `ArCycleStatus`
与 DebugView 状态语义（排除目标仍为 `kNotInOutput`，规则 13c）；生命周期失效（miss 积累 → `kLost`）
不产生排除诊断（规则 13d）。周期摘要日志（`[SignalPipeline] … excluded={{snr=…}}`）仅人读（规则 13a）。

### TrackOutputFrame 不扩展的决策依据

`TrackOutputFrame`（L1）只包含 track 快照（`cycle_index`、`batch_id`、`tracks`）。
`ArCycleResult`（L2）承载 `emission_frame`、`receiver_impairment`、`interference_observations`、
`submitted_commands`、`control_profile`、`decision_observation`、`association_quality_metrics` 等额外字段。

**这些字段不合并入 TrackOutputFrame 的证据**：

1. **消费者画像完全分离**：`ArTrackLifecycleRecorder` 和 `Step()` 只读 track；跨域测试
   `ArRfTestCycleResult` 明确跳过 `emission_frame`；`TacticalCoordinator` 从 `DecisionInputFrame` 读
   `interference_observations`，不从 `CycleResult` 读。
2. **`emission_frame` 是孤立的**：唯一非基础设施消费者是 AR RF 测试自身。跨域消费者只读
   `ecm_result.emission_frame`（ECM 的发射事实作为 AR 输入），不读 `ar_result.emission_frame`。
3. **合并会污染 track-only 消费者**：`ArTrackLifecycleRecorder` 和 `Step()` 会被迫携带它们忽略的数据。

`ArCycleResult` 的膨胀是 AR 多子系统（发射/接收/检测/跟踪/决策）耦合的自然结果，不是三层模型的缺陷。
各字段在 L2 的位置是正确的——它们是"本周期执行结果的完整上下文"，不是"传感器原始输出"。

[evidence: tests/integration/cross_domain/multi_model_scenario_test.cpp — ArRfTestCycleResult 字段选择]

## 滤波后端选型（人工配置为主，不做在线自动切换）

AR 使用标准 Joseph 形式 Kalman 滤波器（KF）作为生产后端。IMM 生命周期（`enable_imm_lifecycle`）是包裹 KF
的多模型融合层，**不是**独立后端。选型决策人工配置在先，理由三点：

1. **可复现性优先于智能性**。在线自动选型会使同一想定因阈值微调走不同后端，结果不可比。
2. **选型决策依赖外部真知**。"目标是否机动"等判据，仿真期真值已知，泄露到选型逻辑等同作弊。
3. **可解释性**。工程评审需能追溯到具体后端与参数。

明确不做：在线残差驱动的自动后端切换。EKF/SRIF/UDKF 的否决依据见 [algorithms.md](algorithms.md) 滤波后端
评估表；可接受的"智能"形态只有只读 NIS 诊断和基于任务剖面先验的 IMM 配置。

## 非目标

1. 不恢复宽 public customization surface。
2. 不把 `EnvironmentService`、`SignalPipeline`、`ArController`、`MutableArContext`、tracking lifecycle 或
   foundation 工程算法暴露为用户可替换 API。
3. 不把单一默认 association 路径包装成 public algorithm family；只有存在多个生产实现时，才通过受控配置
   暴露选择。
4. 不做在线残差驱动的自动滤波后端切换。
5. 不把测试 mock 便利接口升级为 public SPI。
6. 不让外部 decision engine 绕过内部 control reducer 和 command mapper。
7. 不把 debug/lifecycle/replay 字段混入系统输出语义。

上述边界由文档结构守护和 public API contract 测试守护。

[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp]
[evidence: tests/contract/check_public_api_boundary.cmake]

## 远程识别子系统边界（kLrr）

- **纯并行输出**：识别仅回填 `TrackOutputFrame::tracks[i].recognition`，不进
  `DecisionInputFrame`/`DecisionObservation`，不作 ThreatAssessment 输入；若未来需识别影响
  威胁评估，须改 Evaluate 签名并走设计变更规则。
- **非目标（否决项）**：ISAR/二维距离-多普勒像、微动特征、在线学习/自适应权重、实时外部
  数据库联网、信号级 IQ/全波散射求解、非 `kLrr` 模式激活识别链路、威胁分类混入识别输出、
  以场景真值直接产生结论、暴露内部识别类型为 public SPI、process-wide 识别全局状态。
- **单位纪律**：`ArSceneTarget::rcs`/`TrackStateSnapshot::rcs` 为 m²（探测链），识别 RCS 特征
  与数据库一律 dBsm；两者显式区分、不得混用（数据库 units 表 `rcs == 'dBsm'`，声明其他
  单位即拒绝——宁拒绝不静默）。
- **ENU 帧约定**：识别高度观测 = 平台海拔 + `snapshot.position_z`，其中 `position_z` 为平台
  ENU 局部切平面上向分量（含平台姿态旋转，见 `TrackStateSnapshot.h`）。径向高度差在 ECEF z
  上投影 sin(lat)，目标沿 x 运动经 cos(lat)cos(lon) 耦合进上向分量——场景构造须按此帧
  约定补偿（`ar_recognition_us_military_scenario_test.cpp` 的 `AltitudeOffsetFor`）。
- **失败降级**：库未加载/版本不兼容 → `kDisabled`（不影响探测/跟踪/战术决策）；分数/分差
  不足 → `kUnknown` 或仅大类；航迹丢失保持结论至 `result_hold_sec` 后置 `kStale`；
  `association_key` 重分配视为新目标；周期 abort/配置提交失败随四类快照回滚；
  `kSensorPoweredOff` 保持结论至保持期后过期；`kValidationRejected` 不推进积累。
- **接口不变式**：识别内部类型（观测构造/四提取器/积累/匹配器/数据库）不进入 public API；
  识别配置经 `has_policy` 整域提交（无叶子级 recognition patch 字段）；公共枚举加性扩展
  （不重排既有值，replay 字节兼容）；replay 逐周期比较识别结果（浮点容差 `1e-5f`），
  `database_version` 入 `ArSessionReplayState`，不一致即 failure。
- **数据性质**：示例库美方型号参数为公开渠道估算（非敏感占位数据，不作真实情报数据）；
  来源：Wikipedia（含 USAF 事实表转述）、GlobalSecurity RCS 表等，RCS 均为公开估算区间中值。

## 识别子模型的物理保真度边界（F1/F2 定性）

远程识别（`kLrr`）在效能级探测链之外引入两条**识别专用更高保真观测路径**，与探测链物理口径**不逐项对账**：

1. **F1 双通道极化**：探测链严格单极化（`ArSceneTarget::rcs` 单标量 m²，`signal/detection/` 无极化路径；
   `RfScenePolarization` 仅用于干扰链极化失配损耗）。识别双通道极化由场景目标
   `polarization_rcs_samples`（dBsm）经同一雷达方程与 SNR 噪声底派生，通道定义（H/V）由识别特征
   数据库固定（schema v1.1 自描述元数据：meta 键 `polarization_channels` 必填校验，加载器不消费
   通道枚举）。该观测是"识别专用更高保真观测"，不与探测链 SNR/Pd 逐项对账。
2. **F2 距离像相干叠加**：全模块效能级（`SignalDetector` Swerling+MarcumQ；`RfScene` 不生成复数 IQ）。
   识别距离像的距离单元投影与相位相干叠加是**识别专用准信号级子模型**（仅消费场景侧
   `range_rcs_scatterers` 真值列表），不影响探测链信号级语义。散射中心级峰值判定是效能级
   简化（粗距离单元下不合并峰标识，仅投影能量），由 `ar_recognition_feature_test` 锁定。

上述两条仅存在于 `src/airborne_radar/recognition/`，不进入探测/关联/跟踪路径。

[evidence: tests/unit/airborne_radar/ar_recognition_feature_test.cpp]

## 识别特征数据库契约（schema v1.1）

- **自描述**：数据库文件是完整、只读、自描述的识别基线。meta 必填六键
  （`schema_version`/`database_id`/`version`/`created_utc`/`polarization_channels`/
  `polarization_energy_reference`）；units 表必填七量纲且 `rcs` 必须为 `dBsm`
  （匹配数学是 dBsm 域，声明其他单位即拒绝——宁拒绝不静默）。
- **权威 DDL 单源**：`schemas/recognition/recognition_feature_database.sql` 是唯一 schema 事实源，
  C++ 加载器、C++ 测试（configure_file 生成头）、建库工具（`tools/recognition_db_builder.py`）
  共用；禁止在别处维护第二份 DDL。加载器 SELECT 列名与 DDL 的一致性由全字段加载用例守护。
- **加载期只读读取器**：加载时只读打开 → 读表校验 → 关闭连接，成功后全量驻留内存；
  运行期不持有 SQLite 连接，Matcher/Tracker 只读消费内存结构。
- **承载不消费**：`display_name` 与 aspect 适用区间随数据入库并加载校验（往返保真），
  当前不参与匹配/识别结果（扩展需新 freeze item）。
- **版本策略**：`schema_version` 语义为 `major.minor`——major 变更破坏性（加载器拒绝，需 freeze
  流程）；minor 变更增量（新增可空表/列，加载器同步读取，仍精确匹配自身版本）。无存量库，
  不做旧版本兼容层。
- **类别映射**：`category_id` 字符串 → 公共大类枚举由 `RecognitionTracker::CategoryToPublic`
  固定映射（BALLISTIC/NEAR_SPACE/FIGHTER/BOMBER/MISSILE/UAV/OTHER，未映射 → `kUnknown`）。
  枚举值加性扩展（不重排既有值，replay 字节兼容）；枚举定义以
  `include/1q/airborne_radar/session/ArRecognitionResult.h` 为准，新增类别必须同步
  该映射与枚举。

## 设计变更规则

1. 新增、删除或改变 public SPI 时，必须同步本文档集、consumer tests 和 `ar_public_api_convenience_test`。
2. 任何新增 runtime patch 字段，必须明确是否影响 pipeline config、自然环境 scenario 或 RF operating state，
   并接入提交/回滚流程。
3. 探测路径如改变 RCS、大气、干扰、波束、SNR、检测概率或量测协方差语义，必须同步 algorithms.md 和相关
   signal/detection tests。
4. 数据关联和 lifecycle 行为变化，必须同步 association quality metrics、decision frame 说明和对应测试。
5. 战术决策或控制归约策略变化，必须补充 LPI/ECCM/ControlReducer 测试，并在 `[evidence: ...]` 标注中记录
   决策依据。
6. 输出字段变化必须保持 `TrackOutputFrame`、`ArCycleResult`、debug/lifecycle/replay 三层分离。
7. 验证优先使用 `unit::airborne_radar`、`contract::airborne_radar`、`replay::airborne_radar`、
   `batch_validation::airborne_radar`、`integration::cross_domain` 以及 AR guards。
