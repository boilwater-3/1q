# 机载远程识别雷达功能与目标特征数据库方案（精简版）

Status: draft
Last-reviewed: 2026-08-04
Base-architecture: docs/airborne_radar/design.md
Cross-module-contract: docs/common/contract.md

本文件是远程识别功能的设计概要与决策记录。接口的权威定义在公开头文件
（`include/1q/airborne_radar/`），数据库 schema 权威在 `schemas/recognition/recognition_feature_database.sql`，
示例数据权威在 `examples/configs/recognition/recognition_database_input.json`；本文只记录代码与
数据无法传达的定位、边界、否决理由与不变式。逐字段结构不再在此转储（避免与头文件漂移）。

## 1. 目的与范围

在现有 AR 探测、跟踪和战术决策能力上增加远程目标识别：`kLrr` 模式下以稳定航迹为对象，
在效能级约束下提取 RCS、运动、双通道极化和宽带一维距离像特征，与预设目标特征数据库
加权匹配，给出弹道目标、临近空间目标、战斗机、轰炸机、导弹、无人机等常见类别及最可能
型号结论（示例库覆盖常见美方型号，参数为公开估算的非敏感占位数据，见 §7）。

不建设信号级 IQ 处理、全电磁散射求解或真实数据库联网服务；观测由目标真值经距离、SNR、
驻留、带宽、视角覆盖和测量误差等效能约束生成，结果可解释、可配置、可回放。

## 1b. 非目标（否决项）

| 非目标 | 原因 |
|---|---|
| ISAR 成像 / 二维距离-多普勒像 | 需长时相干积累与精确运动补偿，属下一代能力 |
| 微动特征（进动、章动、自旋频率） | 需相位级建模与长时间序列 |
| 在线学习 / 自适应权重更新 | 与 trace/replay 确定性回放冲突；权重调整走数据库版本管理 |
| 实时外部数据库联网 | 首期为本地 SQLite 文件；联网留待 `ConnectDataSource` |
| 信号级 IQ / 全波电磁散射求解 | 效能级仿真定位 |
| 修改现有探测/关联/航迹滤波代码 | 识别是附加链路，不改变探测 SNR、Pd、量测协方差或关联逻辑 |
| 非 `kLrr` 模式激活识别链路 | 仅 `kLrr` 运行识别；其他模式 `recognition.state == kDisabled` |
| `FeatureRepository` 威胁分类混入识别输出 | 两类输出模型独立，不共享字段或权重 |
| 以场景真值直接产生识别结论 | 结论只消费效能化观测 |
| 暴露内部识别类型为 public SPI | 仅 `ArRecognitionConfig`/`ArRecognitionResult`/`ArRecognitionCycleSummary` 可公开 |
| 新增 process-wide 识别单例/全局状态 | 状态绑定 `ArSession` 生命周期 |

## 2. 设计约束

- 识别以 `association_key` 关联航迹；不依赖可缺失的 `external_target_id`。航迹回收时
  `RecognitionTrackState` 随之清理，重分配键视为新目标。
- 仅使用已探测并已形成航迹的量测及其效能化派生特征；不得由 `target_name`/外部 ID/场景真值
  直接产生结论。
- 识别状态跨周期累积，纳入 `ArControllerRuntimeState` 快照矩阵（capture/restore 回滚），
  不新增第五快照域。
- 输出携带特征质量、分项得分和候选差距，不只给单一型号标签；同时携带 `source_cycle_index`/
  `source_batch_id` 支持 replay 溯源（与 `applied_decision_*` provenance 模式一致）。
- 数据库 `database_id` 和 `version` 进入 replay state（`ArSessionReplayState`），保证回放加载
  库版本与录制一致。
- 命名：新增 public 类型统一 `Ar*` 前缀，由 `check_cross_domain_naming.cmake` 守护。

## 3. 工作模式与数据流

`ArWorkMode::kLrr=4`（long-range recognition radar）。扫描策略：默认仅对已确认重点航迹分配
识别驻留（Path A：controller 从 prior-cycle 确认航迹选优先目标，经扫描中心覆盖注入）；无重点
航迹维持任务配置指向。较长驻留、较窄波束、较大有效带宽提升极化与距离像质量；识别调度不得
绕过波束、发射功率、带宽、扫描限位与 LPI/ECCM 控制链。切出 `kLrr` 后停止新增积累，历史结论
按 `result_hold_sec` 保持后置 `kStale`。模式切换复用已有 `has_work_mode` 叶子覆盖，无需新 patch 字段。

```text
ArSceneTarget 特征真值 → 探测/跟踪链路 → 已确认 TrackStateSnapshot
  → RecognitionObservationBuilder（效能化观测）→ 四类 FeatureExtractor
  → 多周期积累 → RecognitionMatcher × RecognitionFeatureDatabase
  → ArRecognitionResult 回填 TrackOutputFrame
```

识别执行点位于 `DecisionInputFrame` 生成之后，是**纯并行输出**：仅回填
`TrackOutputFrame::tracks`，不进 `DecisionInputFrame`/`DecisionObservation`，不作 ThreatAssessment
输入。若未来需识别影响威胁评估，再改 Evaluate 签名。

## 4. 配置与公共输出（要点）

- `ArRecognitionConfig`（`ArPolicyConfig` 第七子域，权威定义 `include/1q/airborne_radar/config/ArRecognitionConfig.h`）：
  总开关 `enabled`、积累（`min_confirmed_hits`/`accumulation_window_sec`/`min_observation_count`）、
  判定（`acceptance_score`/`minimum_margin`）、保持（`result_hold_sec`）、作用范围 `max_range_m`、
  驻留 `recognition_dwell_sec`、`feature_weights` 与 `database_path`。校验规则（拒绝负权重、权重和
  非 1、窗口小于周期步长、`enabled` 且路径空等）由 `ArSessionConfigValidation` 守护。有效带宽取自
  `hardware::transmitter.bandwidth_hz`（只读），距离分辨率 `c/(2B)`。运行期整域提交走已有
  `has_policy` 覆盖，与 `kLrr` 的 `has_work_mode` 可同次提交（叶子优先）。
- `ArRecognitionResult`（嵌入 `TrackStateSnapshot::recognition`，权威定义 `ArRecognitionResult.h`）：
  状态机 `kDisabled/kAccumulating/kCategoryConfirmed/kModelConfirmed/kUnknown/kStale`；输出
  `target_category`（枚举加性扩展，见 §10）、`target_model`、`confidence`、`best_score`/
  `runner_up_score`、`feature_scores`（四维相似度+质量）、`valid_feature_mask`、证据量、
  `database_version` 与 provenance。识别结果与威胁分类（`target_type`/`target_probability`）
  独立填充，互不覆盖。
- **单位纪律**：现有 `ArSceneTarget::rcs` 与 `TrackStateSnapshot::rcs` 为 **m²**；识别 RCS 特征
  与数据库一律 **dBsm**，两者必须显式区分、不得混用（数据库 units 表声明 `rcs == 'dBsm'`）。

## 5. 特征观测与提取（原则）

- **真值 → 效能观测**：`aspect_rcs_samples`/`polarization_rcs_samples`/`range_rcs_scatterers`
  是场景目标特征真值输入（`ArSceneTypes.h`，仅 `kLrr` 消费，默认空）；由
  `RecognitionObservationBuilder` 在 SNR/距离/带宽/驻留/噪声约束下转换为可识别观测。
  空列表 = 维度不可用，不得以单值 `rcs` 或零值替代。
- **RCS**（`RcsFeatureExtractor`）：按视线角对样本网格插值；提取均值（dBsm）、标准差、
  方位/俯仰变化、峰谷比与有效视角覆盖。单样本为退化网格，仅精确命中点有效；多样本网格
  跨距不足视角覆盖下限时维度无效。
- **运动**（`MotionFeatureExtractor`）：消费滤波航迹（非场景真值）——`speed`/`acceleration`
  取模长；`altitude_m = 平台海拔 + snapshot.position_z`，其中 `position_z` 为**平台 ENU 局部
  切平面上向分量**（含平台姿态旋转，`TrackStateSnapshot.h`）；`turn_radius_m = v²/a_lat`，
  横向加速度低于阈值记为直线飞行（不参与半径相似度）。速度/高度/加速度采用窗口均值聚合，
  转弯半径取对数尺度（log10）比对。
- **双通道极化**（`PolarizationFeatureExtractor`）：通道 RCS 经同一雷达方程与距离/传播/天线
  补偿换算到统一参考条件后定义三个 dB 指标（能量差/能量相对差/能量和）；任一通道缺失或
  低于噪声底则维度无效；三项相关，匹配须用子特征权重或相关性折减避免重复加权。
- **宽带一维距离像**（`RangeProfileFeatureExtractor`）：散射中心按 `c/(2B)` 距离单元投影，
  非相干（功率）或相干（相位）叠加 + 噪声底/检测门限后计有效峰；提取长度、峰数、能量集中率
  （前 K=3 峰能量占比）。分辨率不满足 `max_range_resolution_m` 时维度质量降零。

## 6. 多周期融合与识别判定

每活跃 `association_key` 一个 `RecognitionTrackState`（窗口积累：运动取中位数，其余取均值；
质量取均值），确认前仅积累。型号得分 = 最佳适用 profile 的加权相似度 × 型号先验：

```text
score(m) = Σ_d w(d)·q(d)·s(m,d) / Σ_d w(d)·q(d)
s = exp(-0.5·z²), z = |x − mean| / std     （连续特征；转弯半径用 log10 尺度）
```

质量 `q(d)` 由 SNR、样本数、时间新鲜度、视角覆盖、距离分辨率和滤波协方差归一化；质量 0 的
维度不进分子也不进分母。类别得分 = 其下型号未归一化分数之和。

判定（`RecognitionTracker`）：最高分 < `acceptance_score` → `kUnknown`；分数达标但
`best − runner_up < minimum_margin` → 确认大类、型号待定；分数/分差/最小观测数/最小有效维度
数全满足 → `kModelConfirmed`（**运动维度不能单独确认型号**）；特征缺失超时保持结论至
`result_hold_sec` 后置 `kStale`。profile 适用条件（`min_snr_db`/`max_range_resolution_m`/
`minimum_aspect_coverage_deg`/`minimum_bandwidth_hz`）不满足时该 profile 不参与匹配。

## 7. 目标特征数据库

- **文件与版本**：每个库是完整、只读、自描述的识别基线（SQLite，schema v1.1），文件名
  `target_feature_database_v<major>.<minor>.db`；`database_id`/`version` 写入每个识别结果与
  replay 记录。权威 DDL 单源 `schemas/recognition/recognition_feature_database.sql`；建库工具
  `tools/recognition_db_builder.py` 从 `recognition_database_input.json` 生成；加载为全量原子
  替换（模式/单位/数值/交叉引用校验通过才生效，失败保持旧库），运行期不持有连接。
- **结构**：`meta`（六键必填）/`units`（七量纲，`rcs` 必须 `dBsm`）/`categories`/`models`/
  `profiles`（适用条件 + aspect 区间，NULL = 全范围）与四模板组表（行存在 = 组存在）。
  库保存从观测提取的特征统计模板，不保存场景真值列表；同一型号多阶段/多视角差异用多个
  `profiles`，型号内取最高分 profile 参与排序。
- **字段规则**：类别/型号 ID 唯一、类别引用存在、`prior > 0`、`std > 0`、转弯半径用
  `mean_log10/std_log10`（仅有限正半径）、RCS 仅 dBsm、集中率 ∈ [0,1] 并声明最低带宽。
  违反即拒绝加载（错误含路径与表/字段上下文）。
- **示例库（v1.1.0）**：6 类别（BALLISTIC/NEAR_SPACE/FIGHTER/BOMBER/MISSILE/UAV）、17 型号
  （2 占位 + 15 常见美方型号）。美方型号参数为公开渠道估算（**非敏感占位数据，不作真实情报
  数据使用**；来源：Wikipedia 含 USAF 事实表转述、GlobalSecurity RCS 表等；RCS 无官方值，为
  公开估算区间中值，速度/高度置信度中-高、RCS 低）：

  | 类别 | 型号 | RCS (dBsm) | 巡航速度 (m/s) | 巡航高度 (m) |
  |---|---|---|---|---|
  | FIGHTER | F-16C / F-15E / F/A-18E / F-22A / F-35A | 0.8 / 11.8 / -10.0 / -37.0 / -27.0 | 250 / 265 / 250 / 520 / 255 | 10500 / 12000 / 10500 / 16000 / 12000 |
  | BOMBER | B-52H / B-1B / B-2A | 20.0 / 3.8 / -10.0 | 240 / 270 / 250 | 10000 / 100（低空）/ 13000 |
  | MISSILE | BGM-109 / AGM-158A / AGM-86C | -10.0 / -25.0 / -5.0 | 255 / 240 / 246 | 40 / 80 / 40（低空掠海） |
  | UAV | MQ-9A / RQ-4B / MQ-4C / MQ-1C | -12.0 / -5.0 / -6.0 / -15.0 | 78 / 159 / 160 / 60 | 7600 / 18000 / 16500 / 4800 |

  数据边界：model prior 统一 1.0（best_score≈相似度，≥0.6 可确认）；新条目 gate 字段置 NULL
  防 gating 误伤；`min_snr_db = 6.0`。

## 8. 效能模型

识别效能由观测质量承载，**不修改数据库模板**（目标特征知识与传感器性能解耦）。纳入的效能
因素：斜距/传播损耗（SNR）、驻留/脉冲积累（观测数与稳定性）、有效带宽（距离像分辨率）、
视角覆盖（各向 RCS 可用性）、航迹协方差（运动质量，读 `estimation_uncertainty_trace`）、
干扰/ECCM（SNR 与极化可用性）、目标机动（运动可分性）。ECCM 措施在下一成功周期才影响实际
发射参数，识别质量因子应在 ECCM 激活后下一周期反映；接收机饱和时本周期不产生极化观测，
识别维度置不可用，不以最后一次有效值填充。

## 9. 失败、降级与可解释性

- 数据库未加载/版本不兼容/校验失败 → `kDisabled`，不影响探测、跟踪与战术决策。
- 模式未开启或航迹未确认 → `kDisabled`/`kAccumulating`，不输出型号。
- 部分特征可用 → 按动态质量权重融合，输出有效维度和分项得分；全不可用/分差不足/分数不足
  → `kUnknown` 或仅大类。
- 航迹丢失 → 保持结论至 `result_hold_sec`；`association_key` 重分配视为新目标。
- 周期 abort/运行期配置提交失败 → 识别缓存、输出与数据库引用随四类快照回滚；
  `kSensorPoweredOff` 保持结论至保持期后置 `kStale`；`kValidationRejected` 不推进积累。
- 调试视图可查候选总分、四维特征分数、质量、拒绝原因与数据库版本，支持误识/漏识复盘。

## 10. 接口契约（不变式；完整定义以头文件为准）

- **枚举**（`ArRecognitionResult.h`）：`ArRecognitionState` 六态；`ArRecognitionCategory`
  **加性扩展**（kBallistic=0/kNearSpace=1/kOther=2/kUnknown=3/kFighter=4/kBomber=5/kMissile=6/
  kUav=7，不重排既有值，replay 字节兼容）；`ArRecognitionFeatureDimension` 位掩码
  （kRcs=0x01/kMotion=0x02/kPolarization=0x04/kRangeProfile=0x08）。类别映射固定于
  `RecognitionTracker::CategoryToPublic`（BALLISTIC/NEAR_SPACE/FIGHTER/BOMBER/MISSILE/UAV/OTHER，
  未映射 → kUnknown）。
- **配置校验**：`feature_weights` 各 ∈ [0,1] 且和 ≈1；`enabled` 时 `database_path` 非空；
  窗口 ≥ 周期步长；观测数/命中数 ≥1；分数/分差 ∈ [0,1]；`max_range_m`/驻留 >0；违反任一
  规则拒绝构建。
- **内部类型不进入 public API**：`RecognitionObservationBuilder`、四提取器、
  `RecognitionTrackState`、`RecognitionMatcher`、`RecognitionFeatureDatabase` 均在
  `src/airborne_radar/recognition/`。
- **所有权**：`ArRecognitionConfig` 归 `ArSessionConfig::policy`；数据库归 `ArController` 内部
  （构造时加载，析构释放）；每航迹 `RecognitionTrackState` 随航迹创建/回收；回滚快照归
  `ArControllerRuntimeState`。
- **Trace/Replay**：识别结果全字段逐周期比较（浮点容差 `1e-5f`）；`database_version` 入
  `ArSessionReplayState`，不一致则回放 failure；累积状态重建验证
  `observation_count`/`accumulation_sec`。

## 11. 实现与验收状态（2026-08-04）

识别功能已全量实现并验收，权威锚点为测试而非本文：

- 存储：JSON → **SQLite v1.1**（原子加载校验、加载期只读读取器）；schema v1.1 语义分组 +
  自描述元数据；权威 DDL 单源 + 建库工具。
- 示例库：v1.1.0，6 类别 17 型号（含 15 个常见美方型号，公开估算占位数据）。
- 场景验证：`ar_recognition_us_military_scenario_test.cpp`（7 叙事场景 + 15 型号全覆盖 sweep，
  端到端加载交付库）；`ar_recognition_example_database_test.cpp`（工具→DDL→加载器一致性）；
  `ar_recognition_database_test.cpp`（加载校验失败用例）、`ar_recognition_feature_test.cpp`
  （四提取器边界）、`ar_recognition_scenario_test.cpp`（效能场景：达成率/混淆率/模式切换/
  replay 往返）。
- 验收：release 53/53、debug 47/48（仅基线 performance 预算项）；builder 负例拒绝；grep
  无 JSON 兼容层残留。
