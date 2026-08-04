# 机载远程识别雷达功能与目标特征数据库方案

Status: draft
Last-reviewed: 2026-07-24
Reviewer: Architecture audit against current codebase (branch codex/esr-rf-v2-receiver)
Review-report: docs/review/remote_recognition_design_review.md
Base-architecture: docs/airborne_radar/design.md (last-reviewed 2026-07-22)
Cross-module-contract: docs/common/contract.md
Stage-B-freeze: §11 Interface contracts frozen; §12 Per-stage acceptance criteria frozen

## 1. 目的与范围

本方案在现有 `airborne_radar`（AR）探测、跟踪和战术决策能力上增加远程目标识别能力。雷达切换到远程识别模式后，以稳定航迹为对象，在效能级约束下提取目标 RCS、运动、双通道极化和宽带一维距离像特征，与预设目标特征数据库进行加权匹配，给出弹道目标、临近空间目标、战斗机、轰炸机、导弹、无人机等常见目标类别及其最可能型号的识别结论（示例库覆盖常见美方型号，参数为公开渠道估算的非敏感占位数据，见 §7.3）。

本方案不建设信号级 IQ 处理、全电磁散射求解或真实数据库联网服务。各类观测由目标真值参数经过距离、SNR、驻留时间、带宽、视角覆盖和测量误差等效能约束后生成，保证结果可解释、可配置、可回放。

## 1b. 非目标（本方案明确不做的事）

以下能力不在首期范围内，不得在实现中引入：

| 非目标 | 原因 |
|---|---|
| ISAR 成像 / 二维距离-多普勒像 | 需要长时间相干积累和精确运动补偿，属于下一代能力 |
| 微动特征（进动、章动、自旋频率） | 需要相位级建模和长时间序列分析 |
| 在线学习 / 自适应权重更新 | 与 trace/replay 确定性回放冲突；权重调整须通过数据库版本管理 |
| 实时外部数据库联网 | 首期为本地 SQLite 数据库文件，由仿真配置引用；联网留待后续 `ConnectDataSource` 真实实现 |
| 信号级 IQ 处理 / 全波电磁散射求解 | 方案定位为效能级仿真，见 §1 |
| 自动滤波后端切换 / 在线模型选择 | 与 `design.md` §2.10 可复现性原则冲突 |
| 修改现有探测/关联/航迹滤波代码 | 识别是附加链路，不得改变基础探测的 SNR、Pd、量测协方差或关联逻辑 |
| 在 `kTws`/`kTas`/`kStt` 模式下激活识别链路 | 仅 `kLrr` 模式下运行识别观测构造和匹配，其他模式下 `recognition.state == kDisabled` |
| 将 `FeatureRepository` 的威胁分类结果混入识别输出 | 两类输出模型独立，不共享字段或权重 |
| 以场景真值直接产生识别结论 | 见 §2 设计约束；识别结论必须只消费效能化观测 |
| 暴露内部识别类型为 public SPI | 遵循 `contract.md` public API 边界规则；仅 `ArRecognitionConfig`/`ArRecognitionResult`/`ArRecognitionCycleSummary` 可公开 |
| 新增 process-wide 的 recognition 单例或全局状态 | 所有识别状态绑定 `ArSession` 生命周期，随 session 销毁回收 |

## 2. 现有能力与设计约束

AR 已提供物理探测（含 RF 干扰链）、扫描调度、数据关联（LAPJV）、航迹滤波（KF/IMM）、确认/丢失生命周期、运行期配置事务（四域 capture/restore 回滚）、trace/replay 和战术威胁分类。主数据流为：

```text
场景目标 + RF干扰 → 探测(SINR/Pd/Monte Carlo) → 关联(LAPJV) → 航迹(KF/IMM/生命周期)
  → DecisionInputFrame → 威胁评估/LPI/ECCM → ControlReducer → ArControlProfile → 下一周期
```

现有 `TrackStateSnapshot::target_type`（`std::string`）与 `target_probability`（`float [0,1]`）是内部战术决策产生的威胁分类结果，例如 `HIGH_THREAT_FIGHTER`，不可复用为型号识别结论。当前内部 `FeatureRepository`（位于 `src/airborne_radar/environment/`，接口 `IFeatureRepository`）仅覆盖速度、RCS、干扰三项特征（内建键 `"speed"`/`"rcs"`/`"jamming"`，权重 0.45/0.35/0.20），默认记录仅含三类目标（`HIGH_THREAT_FIGHTER`/`LOW_THREAT_TARGET`/`UNKNOWN`），且面向威胁评估；远程识别必须使用独立的特征库与输出模型，避免语义和调参互相污染。

设计约束如下：

- 识别以 `association_key`（`TrackStateSnapshot::association_key`，`std::uint64_t`，单调递增内部键）关联航迹，不依赖可缺失或重复的 `external_target_id`（0 表示未知）。`TrackOutputFrame` 已提供 `BuildTrackMapByAssociationKey()` 查询函数。
- 仅使用已探测并已形成航迹的量测及其效能化派生特征；不得由 `target_name`、外部 ID 或场景型号真值直接产生识别结论。
- 识别状态与航迹状态均跨周期累积，必须纳入 `ArSession` 的 capture/restore 回滚边界。当前回滚机制覆盖四类快照：`ArContextRuntimeState`、`SignalPipelineRuntimeState`、`EnvironmentServiceRuntimeState`、`ArControllerRuntimeState`。识别状态建议归入 `ArControllerRuntimeState` 以复用现有快照矩阵，避免新增第五快照域。
- 输出需要携带特征质量、分项得分和候选差距，不能只给出单一型号标签。同时携带 `source_cycle_index` 和 `source_batch_id` 以支持 replay 溯源（与 `ArCycleResult` 中 `applied_decision_cycle_index`/`applied_decision_batch_id` 的 provenance 模式一致）。
- 首期数据库采用版本化本地 JSON 文件加载，作为仿真输入的一部分参与 trace/replay；不接入外部数据库驱动。数据库 `database_id` 和 `version` 需进入 replay state（`ArSessionReplayState`），保证回放时加载的库版本与录制时一致。

## 3. 工作模式与运行流程

### 3.1 工作模式

`ArWorkMode` 定义在 `include/1q/airborne_radar/config/ArOrientationConfig.h:65-70`，当前包含 `kStby=0`、`kTas=1`、`kTws=2`、`kStt=3`。增加 `kLrr=4`（long-range recognition radar）作为与 `kTas`、`kTws`、`kStt` 并列的任务工作模式。其扫描策略为：

- 默认仅对已确认的重点航迹分配识别驻留（Path A：controller 从 prior-cycle 确认航迹选
  优先目标，经 `ArRuntimeConfigPatch::has_scan_center_deg` 与 ISignalPipeline 周期扫描中心
  覆盖设指向；调度器对 kLrr 与 kStt 一样 passthrough）；无重点航迹时维持任务配置指向。
- 通过较长驻留时间、较窄波束和较大有效带宽提升极化与一维距离像特征质量。
- 识别调度不得绕过现有波束、发射功率、带宽、扫描限位和 LPI/ECCM 控制链路。
- 切出 `kLrr` 后停止新增识别积累；历史结论按 `result_hold_sec` 保持，过期后标记为 `kStale`。

`kLrr` 可通过已有的 `ArRuntimeConfigPatch::work_mode` 叶子覆盖切换（`has_work_mode=true`），无需新增 runtime patch 字段。`kLrr` 的目标选择、驻留资源和优先级由新增识别策略配置决定。首期可按航迹威胁等级、距离、确认状态和上周期识别不确定度排序。

### 3.2 周期数据流

```text
ArSceneTarget 目标特征真值
    -> 探测/跟踪链路
    -> 已确认 TrackStateSnapshot
    -> RecognitionObservationBuilder（效能化观测）
    -> 四类 FeatureExtractor
    -> RecognitionTrackState 多周期积累
    -> RecognitionMatcher 与 RecognitionFeatureDatabase 比对
    -> ArRecognitionResult 回填 TrackStateSnapshot
    -> 威胁评估和 TrackOutputFrame 输出
```

识别执行点位于 signal pipeline 生成 `DecisionInputFrame` 之后（对应 `design.md` §1.4
时序图中 `Controller->>Decision: Evaluate(frame, state_store)` 执行点附近）。当前架构下
识别是**纯并行输出**：照威胁分类先例仅回填 `TrackOutputFrame::tracks`，不进
`DecisionInputFrame`/`DecisionObservation`，也不作为 ThreatAssessment 的输入（ThreatAssessment
只读 speed/rcs/status）。若未来需识别影响威胁评估，再改 Evaluate 签名。

注：`ArController` 和 `SignalPipeline` 均为内部类型（位于 `src/airborne_radar/runtime/` 和 `src/airborne_radar/signal/pipeline/`），不在 public API 中。公开可见的语义边界是 `ArSession::StepWithResult()` → `ArCycleResult`（含 `DecisionObservation`）。

## 4. 配置与公共输出

### 4.1 识别配置

新增 `ArRecognitionConfig` 并聚合至 `ArSessionConfig::policy`（当前 `ArPolicyConfig` 结构见 `include/1q/airborne_radar/config/ArPolicyConfig.h:144-151`，含 `detection`/`beam_control`/`association`/`tracking`/`lifecycle`/`decision_control` 六个子域）。识别配置（权重、门限、窗口、数据库路径）属于判决规则和策略参数，按四域所有权模型归入 `policy`。运行期可通过 `ArRuntimeConfigPatch::has_policy` 整域覆盖修改可调项（与 `ArPolicyConfig` 中的 `ArRecognitionConfig` 一起原子提交），切换 `kLrr` 模式则通过已有的 `has_work_mode` 叶子覆盖。核心字段如下。

| 字段 | 单位 | 说明 |
|---|---:|---|
| `enabled` | - | 识别能力总开关 |
| `min_confirmed_hits` | 次 | 允许正式识别所需最小确认命中数 |
| `accumulation_window_sec` | s | 单航迹特征滑动积累窗口 |
| `min_observation_count` | 次 | 允许输出型号所需最小有效观测数 |
| `result_hold_sec` | s | 退出模式或短时特征缺失后的结论保持时间 |
| `acceptance_score` | [0,1] | 型号确认的最低综合得分 |
| `minimum_margin` | [0,1] | 第一、第二候选的最低得分差 |
| `max_range_m` | m | 识别任务最大作用距离 |
| `recognition_dwell_sec` | s | 单次识别驻留时间 |
| `feature_weights` | - | RCS、运动、极化、距离像的基础权重结构体（`rcs_weight`/`motion_weight`/`polarization_weight`/`range_profile_weight`），各为 `float [0,1]`，和为 1 |
| `database_path` | - | 特征数据库 JSON 路径，建议 `examples/configs/recognition/` |

有效带宽取自 `ArSessionConfig::hardware::transmitter.bandwidth_hz`（`TransmitterConfig` 第171行，默认 4.5 MHz）；一维距离像的距离分辨率为 `c / (2 * bandwidth_hz)`。配置校验应拒绝负权重、非有限值、权重和为零、窗口小于周期步长及带宽非正等输入。

### 4.2 识别结果

在 `TrackStateSnapshot`（当前定义于 `include/1q/airborne_radar/session/TrackStateSnapshot.h:30-61`）中增加独立 `ArRecognitionResult recognition` 字段。注意现有 `TrackStateSnapshot::rcs` 单位为 **m²**（第52行），识别结果中的 RCS 特征建议使用 dBsm 以避免混淆：

| 字段 | 类型 | 说明 |
|---|---|---|
| `state` | enum | `kDisabled`、`kAccumulating`、`kCategoryConfirmed`、`kModelConfirmed`、`kUnknown`、`kStale` |
| `target_category` | enum | `BALLISTIC`、`NEAR_SPACE`、`OTHER`、`UNKNOWN` |
| `target_model` | string | 最可能型号；未确认时为空字符串 |
| `confidence` | float [0,1] | 第一候选的归一化综合置信度 |
| `best_score` / `runner_up_score` | float [0,1] | 前两名候选分数，用于说明可分性 |
| `feature_scores` | struct | RCS、运动、极化、距离像的相似度和质量 |
| `valid_feature_mask` | uint8_t | 本周期参与融合的特征维度位掩码 |
| `observation_count` / `accumulation_sec` | uint32_t / float | 证据积累量 |
| `database_version` | string | 输出所使用的特征库版本 |
| `source_cycle_index` | uint32_t | 产生此结论的 cycle_index |
| `source_batch_id` | uint64_t | 产生此结论的 batch_id |

`source_cycle_index` 和 `source_batch_id` 与 `ArCycleResult` 中 `applied_decision_cycle_index`/`applied_decision_batch_id` 的 provenance 模式一致，支持 replay 逐周期溯源。

`ArCycleResult`（`include/1q/airborne_radar/session/ArCycleResult.h:42-65`）应新增识别效能摘要：

## 5. 特征观测与提取

### 5.0 目标识别特征输入

现有 `ArSceneTarget`（`include/1q/airborne_radar/session/ArSceneTypes.h:23-48`）结构为：

```cpp
struct ArSceneTarget {
  std::uint64_t external_target_id{0};
  std::string target_name{};
  float velocity_x/y/z;       // m/s
  float rcs{0.0f};            // m² — 单一等效 RCS，服务于基础探测链路
  float range_m{0.0f};
  float position_x/y/z;       // m
  int target_swerling_type{0};
};
```

`rcs` 是单一等效 RCS（m²），只能继续服务于基础探测链路，不能满足远程识别的距离像和双通道极化观测要求。启用远程识别的场景目标应扩展以下可选识别特征描述子（仅在 `kLrr` 模式下需要，`kTws`/`kTas`/`kStt` 模式下为空即可）：

| 数据 | 最低输入要求 | 消费特征 |
|---|---|---|
| `aspect_rcs_samples` | 方位/俯仰/RCS 样本列表 | 各向 RCS |
| `polarization_rcs_samples` | 方位/俯仰及两极化通道 RCS 成对样本列表 | 双通道极化 |
| `range_rcs_scatterers` | 距离向位置与散射中心 RCS 成对列表 | 一维距离像 |

上述列表表达的是目标特征真值输入，仅由 `RecognitionObservationBuilder` 在 SNR、距离、带宽、驻留和噪声约束下转换为可用于识别的观测。常规搜索/跟踪目标可不提供这些列表，但相应识别维度必须标记为不可用，不能以单值 `rcs` 或默认零值替代。

### 5.1 RCS 特征

场景目标提供按入射方位角和俯仰角离散的 RCS 真值表，而不是单一标量；每个样本定义为：

```text
AspectRcsSample {
  aspect_az_deg: float
  aspect_el_deg: float
  rcs_dbsm: float
}
aspect_rcs_samples: vector<AspectRcsSample>
```

观测构造器根据当前视线角插值，并叠加由 SNR、量测误差和视角覆盖决定的扰动。

提取特征包括平均 RCS（dBsm）、标准差、方位变化幅度、俯仰变化幅度、峰谷比和有效视角覆盖。仅一个观测角时可输出平均 RCS，但角域变化特征应标记无效。注意：现有 `ArSceneTarget::rcs` 和 `TrackStateSnapshot::rcs` 均为 **m²**，但识别 RCS 特征新增字段使用 **dBsm** 存储和比对，避免在线性面积尺度上过度放大大目标差异。两种单位在数据库和输出中必须显式区分，不得混用。

### 5.2 运动特征

运动特征由滤波航迹多周期估计，不直接采用场景输入速度真值。`TrackStateSnapshot` 已提供的基础字段（`TrackStateSnapshot.h:44-50`）：

- 速度模长 `speed`（float, m/s）。
- 加速度模长 `acceleration`（float, m/s²）。
- 速度分量 `velocity_x/y/z`（float, m/s）。
- 加速度分量 `acceleration_x/y/z`（float, m/s²）。

识别所需运动特征直接消费以上字段：

- `speed_mps` → `TrackStateSnapshot::speed`。
- `altitude_m` — 目标绝对高度。`TrackStateSnapshot` 中只有雷达局部坐标 `position_z`，需由 `RecognitionObservationBuilder` 结合平台位姿（`ArCycleInput::platform` 中的平台绝对高度）换算，不在快照中新增字段。
- `acceleration_mps2` → `TrackStateSnapshot::acceleration`。
- `turn_radius_m` — 使用横向加速度计算：`speed² / lateral_acceleration`。横向加速度从 `acceleration_x/y/z` 分解得到。

横向加速度低于阈值时将转弯半径记为直线飞行，不以极大有限数参与距离计算。速度、加速度与高度采用窗口中位数；转弯半径采用对数尺度比对，以兼顾大半径轨迹差异。

### 5.3 双通道极化特征

场景目标至少提供同一观测几何下两个正交极化通道的目标 RCS 数据，而不直接提供极化回波能量真值。应以按视线角索引的 `polarization_rcs_samples` 表示通道 RCS；列表至少包含一个样本：

```text
PolarizationRcsSample {
  aspect_az_deg: float    // 当前视线相对目标参考系的方位角，单位 deg
  aspect_el_deg: float    // 当前视线相对目标参考系的俯仰角，单位 deg
  channel_1_rcs_dbsm: float // 第一极化通道目标 RCS，单位 dBsm
  channel_2_rcs_dbsm: float // 第二极化通道目标 RCS，单位 dBsm
}
polarization_rcs_samples: vector<PolarizationRcsSample>
```

通道定义和顺序必须由任务配置固定并写入数据库元数据，例如 `H/V`、`HH/VV` 或其他明确的收发极化组合；不同通道定义的数据不得在同一数据库中直接比对。观测构造器使用当前视线角对该列表插值，超出样本覆盖范围时极化维度无效，不得回退至单值 RCS。

观测构造器分别以两个通道 RCS 代入相同的雷达方程，并结合通道增益、接收损耗、噪声底、干扰和 SNR 生成接收端线性能量 `E1_linear`、`E2_linear`。能量和在入库/匹配前必须按当前距离、传播损耗、天线增益和发射参数换算到统一参考条件，得到 `E1_ref_linear`、`E2_ref_linear`；否则原始接收能量会随距离和体制变化，不能作为目标型号特征。三项特征定义为：

```text
polarization_energy_difference_db = 10 * log10(E1_linear / E2_linear)
polarization_relative_difference_db = 10 * log10(abs(E1_linear - E2_linear) /
                                                   (E1_linear + E2_linear))
polarization_energy_sum_db = 10 * log10(E1_ref_linear + E2_ref_linear)
```

其中能量差表示通道功率比，能量相对差表示通道功率差相对总功率的比例，能量和表示在统一参考条件下的总能量，三者均以 dB 表示。相对差为零时按配置的最小正数下限计算，避免对数奇异；三项仍存在相关性，匹配器必须使用极化子特征权重或相关性折减，避免重复加权。任一通道 RCS 缺失或观测低于噪声底时，极化维度整体无效。

### 5.4 宽带一维距离像特征

场景目标必须提供距离向散射 RCS 列表，而非仅提供一个总体 RCS。列表的每个元素是一个散射中心，至少包含相对距离 `range_offset_m` 和该散射中心的 `rcs_dbsm`；可选相位 `phase_deg` 和起伏系数。建议公共场景 DTO 使用如下语义：

```text
RangeRcsScatterer {
  range_offset_m: float   // 相对目标参考点的距离向位置，单位 m
  rcs_dbsm: float         // 该位置散射中心 RCS，单位 dBsm
  channel_1_rcs_dbsm: optional // 第一极化通道散射中心 RCS，单位 dBsm
  channel_2_rcs_dbsm: optional // 第二极化通道散射中心 RCS，单位 dBsm
  phase_deg: optional     // 相干距离像所需相位，单位 deg
  fluctuation_std_db: optional // 周期起伏标准差，单位 dB
}
range_rcs_scatterers: vector<RangeRcsScatterer>
```

`range_offset_m` 是散射中心在当前雷达视线方向上的相对距离；场景适配器应在目标姿态或视线变化后重新投影该值。一维距离像只使用 `range_rcs_scatterers`，不允许以单个总体 RCS 代替。若需生成分极化距离像，应使用散射点的两通道 RCS 列表；这两项为可选字段，不替代 `polarization_rcs_samples` 的最低输入要求。若需从散射列表推导总体 RCS，必须先将各项从 dBsm 转为平方米并在物理上相应的域内合成，不允许直接累加 dBsm 数值。观测构造过程：

1. 根据有效带宽计算距离单元宽度。
2. 将每个“相对距离-RCS”散射中心投影至对应距离单元，并将 `rcs_dbsm` 转为线性散射功率后叠加。
3. 未提供相位时按非相干功率叠加；提供相位时，将散射振幅按 `sqrt(rcs_m2) * exp(j * phase)` 相干叠加后取模平方。
4. 叠加由 SNR 决定的噪声底和检测门限。
5. 对超过峰值门限且满足最小间隔的单元计为有效峰。

提取目标长度、有效峰值数量、能量峰值集中率和可选主峰间距。目标长度为首末有效距离单元中心距；能量集中率定义为前 `K` 个峰能量与总有效能量之比，`K` 默认为 3。若距离分辨率大于数据库要求的最大分辨率，距离像维度质量应降为零，不参与该周期融合。

## 6. 多周期融合与识别判定

每个活跃 `association_key` 对应一个 `RecognitionTrackState`，其中保存时间戳特征样本、视角覆盖、特征质量、最近候选排序和最后有效结论。状态随航迹创建，随航迹回收删除；确认前仅积累不确认型号。

对型号 `m` 的每项特征计算相似度 `s(m,d)`，范围为 `[0,1]`。每项的实际贡献由基础权重 `w(d)` 和当前质量 `q(d)` 共同决定：

```text
score(m) = sum(w(d) * q(d) * s(m,d)) / sum(w(d) * q(d))
```

其中 `q(d)` 由 SNR、有效样本数、时间新鲜度、视角覆盖、距离分辨率和滤波协方差归一化得到。无有效维度、有效权重和过小或候选特征不适用时，不产生型号结论。

判定规则：

- 最高分低于 `acceptance_score`：输出 `kUnknown`。
- 最高分满足门限但与第二名差值小于 `minimum_margin`：确认大类，型号保持待定。
- 满足分数、分差、最小观测数及最小有效维度数：输出 `kModelConfirmed`。
- 连续多个窗口未满足质量要求：保留最后确认结论至 `result_hold_sec`，之后变为 `kStale`。

## 7. 目标特征数据库设计

### 7.1 文件组织与版本控制

每个数据库文件是一个完整、只读、自描述的识别基线（SQLite，schema v1.1），存放于
`examples/configs/recognition/`，生产场景由 `database_path` 明确引用。文件名采用
`target_feature_database_v<major>.<minor>.db`；`database_id` 和 `version` 必须写入每个识别结果和 replay
记录。库文件由建库工具 `tools/recognition_db_builder.py` 从本节格式 JSON 生成（示例输入
`examples/configs/recognition/recognition_database_input.json`）；权威 DDL 单源为
`schemas/recognition/recognition_feature_database.sql`。

数据库加载应为全量原子替换：新文件通过模式、单位、数值和交叉引用校验后才替换当前生效库；加载失败时
保持原库不变。每个 `model_id` 在同一库中必须唯一。

### 7.2 顶层结构

schema v1.1 将以下结构映射为 SQLite 表：`meta`（键值表，六键必填，含 `created_utc`、
`polarization_channels`、`polarization_energy_reference`）、`units`（七量纲必填，`rcs` 必须为 `dBsm`）、
`categories`、`models`、`profiles`（适用条件 + aspect 区间）与四个模板组表
（`rcs_templates`/`motion_templates`/`polarization_templates`/`range_profile_templates`，行存在 = 组存在）。
建库工具输入（设计文档 JSON 格式）顶层结构如下：

```json
{
  "meta": {
    "schema_version": "1.1",
    "database_id": "ar-target-recognition-baseline",
    "version": "1.0.0",
    "created_utc": "2026-07-22T00:00:00Z",
    "polarization_channels": ["H", "V"],
    "polarization_energy_reference": "range_propagation_antenna_compensated"
  },
  "units": {
    "rcs": "dBsm",
    "speed": "m/s",
    "altitude": "m",
    "acceleration": "m/s2",
    "turn_radius": "m",
    "polarization": "dB",
    "range": "m"
  },
  "categories": [],
  "models": []
}
```

`categories` 用于定义标准目标大类及先验，`models` 保存可匹配的型号特征原型。数据库保存的是从观测提取出的特征统计模板，不保存场景侧的 `aspect_rcs_samples`、`polarization_rcs_samples` 或 `range_rcs_scatterers` 真值列表。首期采用单原型加容差的高斯型模板；同一型号在不同飞行阶段、姿态区间或视角区间差异显著时，配置多个 `profiles`，取该型号内得分最高的 profile 参与型号排序。

### 7.3 类别和型号记录

以下示例仅展示结构和非敏感占位参数，不应作为真实目标特征数据使用。

```json
{
  "schema_version": "1.0",
  "database_id": "ar-target-recognition-baseline",
  "version": "1.0.0",
  "created_utc": "2026-07-22T00:00:00Z",
  "polarization_channels": ["H", "V"],
  "polarization_energy_reference": "range_propagation_antenna_compensated",
  "units": {
    "rcs": "dBsm",
    "speed": "m/s",
    "altitude": "m",
    "acceleration": "m/s2",
    "turn_radius": "m",
    "polarization": "dB",
    "range": "m"
  },
  "categories": [
    {"category_id": "BALLISTIC", "display_name": "弹道目标", "prior": 0.5},
    {"category_id": "NEAR_SPACE", "display_name": "临近空间目标", "prior": 0.5}
  ],
  "models": [
    {
      "model_id": "BALLISTIC_EXAMPLE_A",
      "category_id": "BALLISTIC",
      "display_name": "弹道目标示例 A",
      "prior": 1.0,
      "profiles": [
        {
          "profile_id": "nominal",
          "applicability": {
            "aspect_az_deg": [-180.0, 180.0],
            "aspect_el_deg": [-90.0, 90.0],
            "min_snr_db": 6.0,
            "max_range_resolution_m": 50.0
          },
          "rcs": {
            "mean_dbsm": -3.0,
            "std_db": 2.0,
            "azimuth_variation_db": 4.0,
            "elevation_variation_db": 3.0,
            "minimum_aspect_coverage_deg": 15.0
          },
          "motion": {
            "speed_mps": {"mean": 1800.0, "std": 300.0},
            "altitude_m": {"mean": 50000.0, "std": 12000.0},
            "acceleration_mps2": {"mean": 12.0, "std": 6.0},
            "turn_radius_m": {"mean_log10": 6.0, "std_log10": 0.5}
          },
          "polarization": {
            "energy_difference_db": {"mean": 2.0, "std": 1.5},
            "relative_difference_db": {"mean": -6.0, "std": 2.0},
            "energy_sum_db": {"mean": 5.0, "std": 4.0}
          },
          "range_profile": {
            "length_m": {"mean": 8.0, "std": 2.0},
            "peak_count": {"mean": 3.0, "std": 1.0},
            "peak_energy_concentration": {"mean": 0.75, "std": 0.10},
            "minimum_bandwidth_hz": 3000000.0
          }
        }
      ]
    },
    {
      "model_id": "NEAR_SPACE_EXAMPLE_A",
      "category_id": "NEAR_SPACE",
      "display_name": "临近空间目标示例 A",
      "prior": 1.0,
      "profiles": [
        {
          "profile_id": "nominal",
          "applicability": {
            "aspect_az_deg": [-180.0, 180.0],
            "aspect_el_deg": [-90.0, 90.0],
            "min_snr_db": 6.0,
            "max_range_resolution_m": 50.0
          },
          "rcs": {
            "mean_dbsm": 2.0,
            "std_db": 2.5,
            "azimuth_variation_db": 3.0,
            "elevation_variation_db": 2.0,
            "minimum_aspect_coverage_deg": 15.0
          },
          "motion": {
            "speed_mps": {"mean": 300.0, "std": 80.0},
            "altitude_m": {"mean": 25000.0, "std": 5000.0},
            "acceleration_mps2": {"mean": 2.0, "std": 1.0},
            "turn_radius_m": {"mean_log10": 4.5, "std_log10": 0.5}
          },
          "polarization": {
            "energy_difference_db": {"mean": -1.0, "std": 1.5},
            "relative_difference_db": {"mean": -8.0, "std": 3.0},
            "energy_sum_db": {"mean": 8.0, "std": 4.0}
          },
          "range_profile": {
            "length_m": {"mean": 20.0, "std": 5.0},
            "peak_count": {"mean": 4.0, "std": 1.0},
            "peak_energy_concentration": {"mean": 0.60, "std": 0.10},
            "minimum_bandwidth_hz": 3000000.0
          }
        }
      ]
    }
  ]
}
```

#### 美方常见型号扩展（示例库 v1.1.0，2026-08-04）

交付库 `target_feature_database_v1.1.db`（meta `version = 1.1.0`）在占位示例之外新增
FIGHTER/BOMBER/MISSILE/UAV 四类共 15 个常见美方型号（公开渠道估算参数，**非敏感占位
数据，不作真实情报数据使用**；来源与置信度见 `docs/review/recognition_us_military_db_plan_2026-08-04.md`）：

| 类别 | 型号 | RCS (dBsm) | 巡航速度 (m/s) | 巡航高度 (m) | 机长 (m) |
|---|---|---|---|---|---|
| FIGHTER 战斗机 | F-16C 战隼 | 0.8 | 250 | 10500 | 15.1 |
| FIGHTER 战斗机 | F-15E 攻击鹰 | 11.8 | 265 | 12000 | 19.4 |
| FIGHTER 战斗机 | F/A-18E 超级大黄蜂 | -10.0 | 250 | 10500 | 18.3 |
| FIGHTER 战斗机 | F-22A 猛禽 | -37.0 | 520 | 16000 | 18.9 |
| FIGHTER 战斗机 | F-35A 闪电II | -27.0 | 255 | 12000 | 15.7 |
| BOMBER 轰炸机 | B-52H 同温层堡垒 | 20.0 | 240 | 10000 | 48.5 |
| BOMBER 轰炸机 | B-1B 枪骑兵 | 3.8 | 270 | 100（低空） | 44.5 |
| BOMBER 轰炸机 | B-2A 幽灵 | -10.0 | 250 | 13000 | 21.0 |
| MISSILE 导弹 | BGM-109 战斧巡航导弹 | -10.0 | 255 | 40（掠海） | 5.6 |
| MISSILE 导弹 | AGM-158A 联合空对地防区外导弹 | -25.0 | 240 | 80（低空） | 4.3 |
| MISSILE 导弹 | AGM-86C 空射巡航导弹 | -5.0 | 246 | 40（低空） | 6.3 |
| UAV 无人机 | MQ-9A 收割者 | -12.0 | 78 | 7600 | 11.0 |
| UAV 无人机 | RQ-4B 全球鹰 | -5.0 | 159 | 18000 | 14.5 |
| UAV 无人机 | MQ-4C 人鱼海神 | -6.0 | 160 | 16500 | 14.5 |
| UAV 无人机 | MQ-1C 灰鹰 | -15.0 | 60 | 4800 | 8.5 |

数据边界：model prior 统一 1.0（best_score≈相似度，≥0.6 可确认）；新条目
`minimum_aspect_coverage_deg`/`minimum_bandwidth_hz`/`max_range_resolution_m` 置 NULL
（不触发 gating）；`min_snr_db = 6.0`。类别映射：FIGHTER→`kFighter`、BOMBER→`kBomber`、
MISSILE→`kMissile`、UAV→`kUav`（§11.1 枚举加性扩展，replay 字节兼容）。

### 7.4 字段规则

| 区域 | 必填字段 | 规则 |
|---|---|---|
| 类别 | `category_id`、`prior` | 类别 ID 必须唯一；先验必须大于零 |
| 型号 | `model_id`、`category_id`、`profiles` | 型号 ID 唯一，类别必须存在，至少一个 profile |
| 极化通道 | `polarization_channels`、`polarization_energy_reference` | 明确两个通道及顺序，以及能量和的距离/传播/天线补偿基准；必须与场景目标双通道 RCS 的定义一致 |
| 适用条件 | `min_snr_db`、`max_range_resolution_m` | 当前观测不满足时该 profile 不参与匹配 |
| 连续特征 | `mean`、`std` | `std` 必须大于零；采用截断高斯相似度 |
| 转弯半径 | `mean_log10`、`std_log10` | 仅对有限正半径参与计算 |
| RCS | `mean_dbsm`、`std_db` | 使用 dBsm，不允许混用平方米 |
| 极化 | 三个 dB 指标 | 对应数据库声明的双通道 RCS 定义；子特征相关时需配置相关性折减 |
| 距离像 | 长度、峰数、集中率 | 模板由距离向 RCS 列表观测提取的统计特征构成；集中率范围为 `[0,1]`，并声明最低有效带宽 |

### 7.5 相似度与先验

连续特征以归一化差值 `z = abs(x - mean) / std` 计算基础相似度 `exp(-0.5 * z^2)`。峰值数量可采用整数距离的同类形式。型号 profile 得分乘以型号先验，型号分数在全部可用型号间归一化形成 `confidence`。

类别分数由其下各型号的未归一化分数求和得到。类别确认允许只依赖满足质量要求的部分特征；型号确认必须同时满足配置的最小有效维度数，其中运动维度不能单独确认型号。

## 8. 效能模型

远程识别效能不只由模板距离决定，至少纳入以下可观测约束（与现有 AR 机制对照）：

| 因素 | 对识别的影响 | 现有相关机制 |
|---|---|---|
| 斜距和传播损耗 | 降低回波 SNR，增加 RCS/极化/距离像测量误差 | `ArSceneTarget::range_m`, `EnvironmentService`/`PropagationModel` |
| 驻留时间和脉冲积累 | 提升有效观测数和特征稳定性 | `ArDetectionPolicyConfig::pulse_count` + 识别专用 `recognition_dwell_sec` |
| 有效带宽 | 决定一维距离像距离分辨率和可分辨散射点数 | `TransmitterConfig::bandwidth_hz`（只读，不修改） |
| 视角覆盖 | 决定各向 RCS 特征是否可用 | 由 `ArOrientationConfig` + 平台姿态 + target look angle 派生 |
| 航迹协方差 | 降低速度、加速度、转弯半径的质量因子 | 预测协方差 P 的 position 分块迹（`TrackStateSnapshot::estimation_uncertainty_trace`，识别只读） |
| 干扰和 ECCM | 改变接收 SNR、极化通道可用性及观测缺失率 | `ArInterferenceObservation::jammer_to_noise_db`, ECCM 激活后的 `ArControlProfile` 字段 |
| 目标机动 | 提供运动可分性，同时可能降低距离像的跨周期一致性 | `TrackStateSnapshot::acceleration`, IMM 模式概率（内部） |

所有效能因素应转换为每个特征维度的质量 `q(d)`，而不是直接修改数据库模板。这样可保持目标特征知识与传感器性能参数解耦。

注意：ECCM 措施（频率捷变、重频抖动、烧穿等）在下一成功周期才影响实际发射参数，识别质量因子应在 ECCM 激活后的下一周期反映变化，不直接读取当前周期的 ECCM intent。接收机饱和（`ArReceiverImpairment::kSaturated`）时跳过干扰观测生成，本周期不产生极化观测；识别维度应标记不可用，不应以最后一次有效值填充。

## 9. 失败、降级与可解释性

- 数据库未加载、版本不兼容或校验失败：识别状态为 `kDisabled`，不影响探测、跟踪和原有战术决策。
- 识别模式未开启或航迹未确认：状态为 `kAccumulating` 或 `kDisabled`，不输出型号。
- 仅部分特征可用：按动态质量权重融合，输出有效维度和分项得分。
- 所有维度不可用、候选分差不足或分数不足：输出 `kUnknown` 或仅确认大类。
- 航迹丢失：保持最后结果至配置时间；重新分配的关联键视为新目标，不继承旧结论。
- 周期 abort 或运行期配置提交失败：恢复识别缓存、最新输出和数据库引用，保证与现有 session 事务语义一致。当前 abort 类型（`SignalCycleAbortReason`）为 `kNone`/`kLifecycleUnavailable`/`kInvalidEnvironmentCycle`/`kRuntimePreparationFailed`/`kValidationRejected`/`kSensorPoweredOff`。`kRuntimePreparationFailed` 时识别状态必须在四类快照回滚中恢复；`kSensorPoweredOff` 时识别应保持最后结论至 `result_hold_sec`，之后标记 `kStale`；`kValidationRejected` 时不推进识别积累。

调试输出应能查看每个候选的总分、四类特征分数、质量因子、被拒绝原因和数据库版本，支持对误识/漏识进行回放复盘。

## 10. 实施阶段与验收

各阶段需同步更新的现有测试与文档引用：

1. **DTO 与配置**：增加 `kLrr`（`ArOrientationConfig.h:65`）、`ArRecognitionConfig`（`ArPolicyConfig` 第七子域）、场景目标特征描述（`ArSceneTarget` 可选扩展）、识别输出（`TrackStateSnapshot` 新增 `recognition` 字段、`ArCycleResult` 新增识别效能摘要）和配置校验。同步更新 `tests/unit/airborne_radar/ar_session_config_builder_test.cpp`。
2. **观测与特征**：实现 RCS、运动、双极化、距离像的效能化观测和特征提取器。新增 `tests/unit/airborne_radar/ar_recognition_feature_test.cpp`。
3. **数据库与匹配**：实现 JSON 加载、结构校验、模板匹配、动态加权、类别/型号判定。新增 `tests/unit/airborne_radar/ar_recognition_database_test.cpp`。
4. **链路集成**：接入 controller 内部执行点（`DecisionInputFrame` 后、`ThreatAssessmentEvaluator` 前）、运行期 patch（`has_policy` 整域覆盖）、航迹生命周期（`association_key` 绑定，回收时清理 `RecognitionTrackState`）、trace/replay（`database_version` 进入 `ArSessionReplayState`、识别结果携带 `source_cycle_index`/`source_batch_id` provenance）、输出适配和调试视图。同步更新 `tests/unit/airborne_radar/ar_core_controller_test.cpp`、`tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp`、`tests/contract/airborne_radar/ar_public_api_convenience_test.cpp`。
5. **效能验证**：建立带标注的弹道目标、临近空间目标、未知目标、低 SNR、强干扰、低带宽和短驻留场景。

验收至少覆盖：四类特征单位和边界（注意 RCS 新增字段 dBsm vs 现有字段 m² 的单位差异）、散射点距离像生成、低横向加速度下的转弯半径、特征缺失降级、类别/型号混淆、模式切换、航迹回收（`association_key` 重分配视为新目标）、周期回滚（四类快照 + 识别状态）、`kSensorPoweredOff` 边界的结论保持与过期、数据库版本回放一致性，以及识别正确率、误识率、漏识率和首次确认时间等效能指标。

命名约定：所有新增 public 类型使用 `Ar*` 前缀（`ArRecognitionConfig`、`ArRecognitionResult`、`ArRecognitionCycleSummary`），与 `ArWorkMode`/`ArSessionConfig` 等现有类型一致，由 `check_cross_domain_naming.cmake` 守护。

---

## 11. 接口冻结契约

以下接口为 Stage B 实现契约，实现必须严格遵守字段类型、取值范围、默认值和所有权语义。偏离本契约的任何行为必须在 `design.md` §4 设计变更规则下同步更新。

### 11.1 枚举定义

```cpp
// ArOrientationConfig.h — ArWorkMode 扩展
enum class ONEQ_API ArWorkMode {
  kStby = 0,
  kTas = 1,
  kTws = 2,
  kStt = 3,
  kLrr = 4   // 新增：远程识别
};

// 新增：识别状态枚举（ArRecognitionResult 内）
enum class ONEQ_API ArRecognitionState : std::uint8_t {
  kDisabled = 0,         // 识别未启用或数据库未加载
  kAccumulating = 1,     // 正在积累特征，尚未达到输出条件
  kCategoryConfirmed = 2,// 大类已确认，型号待定
  kModelConfirmed = 3,   // 型号已确认
  kUnknown = 4,          // 已积累足够观测但无候选满足门限
  kStale = 5             // 退出模式或特征缺失超时，结论已过期
};

// 新增：识别目标大类（值加性扩展，不重排既有值；replay 字节兼容）
enum class ONEQ_API ArRecognitionCategory : std::uint8_t {
  kBallistic = 0,  // 弹道目标
  kNearSpace = 1,  // 临近空间目标
  kOther = 2,      // 其它
  kUnknown = 3,    // 未知
  kFighter = 4,    // 战斗机
  kBomber = 5,     // 轰炸机
  kMissile = 6,    // 导弹
  kUav = 7         // 无人机
};

// 新增：特征维度位掩码
enum class ONEQ_API ArRecognitionFeatureDimension : std::uint8_t {
  kRcs = 1 << 0,           // 0x01
  kMotion = 1 << 1,        // 0x02
  kPolarization = 1 << 2,  // 0x04
  kRangeProfile = 1 << 3   // 0x08
};
```

### 11.2 ArRecognitionConfig（聚合至 ArPolicyConfig）

```cpp
// 新增文件：include/1q/airborne_radar/config/ArRecognitionConfig.h
struct ONEQ_API ArRecognitionFeatureWeights {
  float rcs_weight{0.25f};           // [0, 1]，四者之和必须为 1.0
  float motion_weight{0.25f};
  float polarization_weight{0.25f};
  float range_profile_weight{0.25f};
};

struct ONEQ_API ArRecognitionConfig {
  bool enabled{false};                       // 默认关闭，需显式启用

  // 积累与确认
  std::uint32_t min_confirmed_hits{5U};      // 最小确认命中数，≥1
  float accumulation_window_sec{10.0f};      // 滑动窗口（s），必须 ≥ dt_sec
  std::uint32_t min_observation_count{3U};   // 最小有效观测数，≥1

  // 判定门限
  float acceptance_score{0.70f};             // [0, 1]，型号确认最低综合得分
  float minimum_margin{0.10f};               // [0, 1]，第一/第二候选最低得分差

  // 时间约束
  float result_hold_sec{30.0f};              // 结论保持时间（s），≥0

  // 作用范围
  float max_range_m{300000.0f};              // 最大识别距离（m），>0

  // 驻留
  float recognition_dwell_sec{0.05f};        // 单次识别驻留（s），>0

  // 权重
  ArRecognitionFeatureWeights feature_weights{};

  // 数据库
  std::string database_path{};               // 空字符串表示未配置；非空必须指向有效 JSON
};
```

**验证规则（ArSessionConfigValidation 新增）：**
- `feature_weights` 四分量各自 ∈ [0, 1]，之和 ∈ [0.999f, 1.001f]
- `enabled == true` 时 `database_path` 必须非空
- `accumulation_window_sec` ≥ session 周期步长 `dt_sec`
- `min_observation_count` ≥ 1
- `min_confirmed_hits` ≥ 1
- `acceptance_score` ∈ [0, 1]
- `minimum_margin` ∈ [0, 1]
- `max_range_m` > 0
- `recognition_dwell_sec` > 0
- `result_hold_sec` ≥ 0
- 违反任一规则：session config validation 报 error，拒绝构建

**ArPolicyConfig 变更：**
```cpp
struct ONEQ_API ArPolicyConfig {
  // ... 现有六个子域不变 ...
  ArRecognitionConfig recognition{};  // 第七子域
};
```

### 11.3 ArSceneTarget 扩展

```cpp
// ArSceneTypes.h — ArSceneTarget 新增可选字段（默认空，仅 kLrr 需要）

struct ONEQ_API AspectRcsSample {
  float aspect_az_deg{0.0f};   // 方位角（deg），任意有限值
  float aspect_el_deg{0.0f};   // 俯仰角（deg），任意有限值
  float rcs_dbsm{0.0f};        // RCS（dBsm），有限值
};

struct ONEQ_API PolarizationRcsSample {
  float aspect_az_deg{0.0f};
  float aspect_el_deg{0.0f};
  float channel_1_rcs_dbsm{0.0f};  // 第一极化通道 RCS（dBsm）
  float channel_2_rcs_dbsm{0.0f};  // 第二极化通道 RCS（dBsm）
};

struct ONEQ_API RangeRcsScatterer {
  float range_offset_m{0.0f};       // 相对距离（m）
  float rcs_dbsm{0.0f};             // 散射中心 RCS（dBsm）
  float channel_1_rcs_dbsm{0.0f};   // 可选：第一极化通道散射 RCS
  float channel_2_rcs_dbsm{0.0f};   // 可选：第二极化通道散射 RCS
  float phase_deg{0.0f};            // 可选：相位（deg），0 表示非相干叠加
  float fluctuation_std_db{0.0f};   // 可选：起伏标准差（dB），0 表示无起伏
};

struct ONEQ_API ArSceneTarget {
  // ... 现有字段不变 ...

  // 新增可选字段（仅 kLrr 模式消费；默认空向量）
  std::vector<AspectRcsSample> aspect_rcs_samples{};
  std::vector<PolarizationRcsSample> polarization_rcs_samples{};
  std::vector<RangeRcsScatterer> range_rcs_scatterers{};
};
```

**所有权与生命周期：** 新增字段由调用方填充，`ArSession` 只读消费。空向量表示该维度不可用，不得以 `rcs` 单值或默认零值替代。`ArInputValidation` 在 `kLrr` 模式下对空向量产生 warning（不 block 周期），在非 `kLrr` 模式下不检查。

### 11.4 ArRecognitionResult（嵌入 TrackStateSnapshot）

```cpp
// 新增文件：include/1q/airborne_radar/session/ArRecognitionResult.h

struct ONEQ_API ArRecognitionFeatureScores {
  float rcs_similarity{0.0f};           // [0, 1]
  float rcs_quality{0.0f};              // [0, 1]
  float motion_similarity{0.0f};        // [0, 1]
  float motion_quality{0.0f};           // [0, 1]
  float polarization_similarity{0.0f};  // [0, 1]
  float polarization_quality{0.0f};     // [0, 1]
  float range_profile_similarity{0.0f}; // [0, 1]
  float range_profile_quality{0.0f};    // [0, 1]
};

struct ONEQ_API ArRecognitionResult {
  ArRecognitionState state{ArRecognitionState::kDisabled};
  ArRecognitionCategory target_category{ArRecognitionCategory::kUnknown};
  std::string target_model{};              // 未确认时为空
  float confidence{0.0f};                 // [0, 1]
  float best_score{0.0f};                 // [0, 1]
  float runner_up_score{0.0f};            // [0, 1]
  ArRecognitionFeatureScores feature_scores{};
  std::uint8_t valid_feature_mask{0U};    // ArRecognitionFeatureDimension 按位或
  std::uint32_t observation_count{0U};
  float accumulation_sec{0.0f};
  std::string database_version{};
  std::uint32_t source_cycle_index{0U};
  std::uint64_t source_batch_id{0U};
};
```

**TrackStateSnapshot 变更：**
```cpp
struct ONEQ_API TrackStateSnapshot {
  // ... 现有字段不变 ...
  ArRecognitionResult recognition{};  // 新增；kDisabled 时为默认值
};
```

**回填时机：** 识别结果照威胁分类先例仅回填到 `TrackOutputFrame::tracks[i].recognition`
（`ArController` 深拷贝之后）；不进 `DecisionInputFrame::tracks`（两帧经同一 replay schema
序列化，回放字节比对容忍该分叉）。`target_type`/`target_probability`（威胁分类）和
`recognition`（型号识别）独立填充，互不覆盖。

### 11.5 ArRecognitionCycleSummary（嵌入 ArCycleResult）

```cpp
// 新增字段，嵌入 ArCycleResult

struct ONEQ_API ArRecognitionCycleSummary {
  std::uint32_t participating_track_count{0U};  // 参与识别的航迹数
  std::uint32_t category_confirmed_count{0U};   // 大类确认数
  std::uint32_t model_confirmed_count{0U};      // 型号确认数
  std::uint32_t unknown_count{0U};             // 未知数
  std::uint32_t disabled_count{0U};            // 未启用/无特征数
  float rcs_availability_rate{0.0f};           // [0, 1] RCS 特征可用率（含质量>0的比例）
  float motion_availability_rate{0.0f};        // [0, 1]
  float polarization_availability_rate{0.0f};  // [0, 1]
  float range_profile_availability_rate{0.0f}; // [0, 1]
  float mean_confidence{0.0f};                 // 已确认型号的平均置信度
  float mean_first_confirmation_sec{0.0f};     // 平均首次确认时间（s）
  bool has_ground_truth{false};                // 本周期是否有真值可用于正确率统计
  float category_accuracy{0.0f};               // [0, 1] 仅 has_ground_truth 时有效
  float model_accuracy{0.0f};                  // [0, 1] 仅 has_ground_truth 时有效
};
```

```cpp
// ArCycleResult 变更
struct ONEQ_API ArCycleResult {
  // ... 现有字段不变 ...
  bool has_recognition_summary{false};
  ArRecognitionCycleSummary recognition_summary{};  // 新增
};
```

### 11.6 ArRuntimeConfigPatch — 识别配置的运行期更新

不新增叶子级 recognition patch 字段。识别配置通过已有的 `has_policy` 整域覆盖提交：

```cpp
// 调用方使用方式：
ArRuntimeConfigPatch patch;
patch.has_policy = true;
patch.policy = new_policy;  // 包含新的 ArRecognitionConfig
session.TryApplyRuntimeConfig(patch);
```

切换 `kLrr` 通过已有的 `has_work_mode` 叶子覆盖：
```cpp
patch.has_work_mode = true;
patch.work_mode = ArWorkMode::kLrr;
```

两者可同一次 patch 提交（先整域后叶子，叶子具有最终优先级）。

### 11.7 内部接口（不进入 public API）

以下类型为内部实现，不得进入 `include/1q/airborne_radar/`：

| 内部类型 | 作用域 | 说明 |
|---|---|---|
| `RecognitionObservationBuilder` | `src/airborne_radar/recognition/` | 效能化观测构造 |
| `RcsFeatureExtractor` / `MotionFeatureExtractor` / `PolarizationFeatureExtractor` / `RangeProfileFeatureExtractor` | `src/airborne_radar/recognition/` | 四类特征提取器 |
| `RecognitionTrackState` | `src/airborne_radar/recognition/` | 单航迹跨周期特征积累 |
| `RecognitionMatcher` | `src/airborne_radar/recognition/` | 加权匹配与判定 |
| `RecognitionFeatureDatabase` | `src/airborne_radar/recognition/` | JSON 数据库加载与校验 |
| `IRecognitionFeatureDatabase` | `src/airborne_radar/recognition/` | 数据库抽象接口（仅内部，无外部替换需求） |

以上类型的测试覆盖应通过内部单元测试完成，不污染 public API 测试。

### 11.8 所有权与生命周期契约

| 资源 | 所有者 | 创建时机 | 销毁时机 |
|---|---|---|---|
| `ArRecognitionConfig` | `ArSessionConfig::policy` | Session 构造 | Session 析构 |
| `RecognitionFeatureDatabase` | `ArController` 内部 | Session 构造（数据库路径非空时） | Session 析构 |
| `RecognitionTrackState`（每航迹） | `ArController` 内部 | 航迹首次进入识别链路 | 航迹 `kRecycled` 或 `association_key` 重分配 |
| `ArRecognitionResult`（每快照） | `TrackStateSnapshot` | 每周期识别执行后回填 | 随快照生命周期 |
| `ArRecognitionCycleSummary` | `ArCycleResult` | 每周期识别执行后组装 | 随结果返回给调用方 |
| 识别回滚快照 | `ArControllerRuntimeState` | `CaptureRuntimeState` | `RestoreRuntimeState` 或 session 析构 |

### 11.9 Trace/Replay 契约

| 事件 | 记录内容 | 回放验证 |
|---|---|---|
| 识别结果 | `ArRecognitionResult` 所有字段 + `database_version` | 逐字段比较；`confidence`/`best_score`/`runner_up_score` 容差 `1e-5f` |
| 数据库版本 | `ArSessionReplayState::active_database_version` | 与录制时一致，不一致则 replay 标记 failure |
| 累积状态 | `RecognitionTrackState` 序列化形式 | 回放时重建，验证 `observation_count` 和 `accumulation_sec` 一致 |
| 周期摘要 | `ArRecognitionCycleSummary` 所有字段 | 计数类字段（`*_count`）精确匹配；浮点率字段容差 `1e-5f` |

---

## 12. 逐阶段验收标准

每个阶段必须通过以下验收条件后才能进入下一阶段。验收失败时回到该阶段修复，不得跨阶段推进。

### 阶段 1：DTO 与配置

**完成定义：**

1. `ArWorkMode::kLrr` 编译通过；`check_cross_domain_naming.cmake` 不报新错误。
2. `ArRecognitionConfig` 可作为 `ArPolicyConfig` 第七子域构造和序列化。
3. `ArRecognitionConfig` 非法值（负权重、零窗口、空数据库路径但 enabled=true 等）被 `ArSessionConfigValidation` 拒绝，error 消息包含被拒绝的具体字段名。
4. `ArSceneTarget` 含三个新 vector 字段，默认空；对不含新字段的旧构造调用保持 ABI 兼容。
5. `TrackStateSnapshot::recognition` 字段默认 state=`kDisabled`。
6. `ArCycleResult::recognition_summary` 默认 `has_recognition_summary=false`。
7. `ArRuntimeConfigPatch` 通过 `has_policy` 提交含 `ArRecognitionConfig` 的变更，被拒绝的非法 patch 保持 session 状态不变。

**测试锚点：** `tests/unit/airborne_radar/ar_session_config_builder_test.cpp`（扩展）、`tests/unit/airborne_radar/ar_runtime_patch_mapper_test.cpp`（扩展）。

### 阶段 2：观测与特征

**完成定义：**

1. `RecognitionObservationBuilder` 对空 `aspect_rcs_samples`/`polarization_rcs_samples`/`range_rcs_scatterers` 返回 `valid_feature_mask=0`。
2. RCS 观测：给定单一样本 `AspectRcsSample{-30°, 5°, -3.0f}` 和 `snr_db=20`，输出 `mean_dbsm` 误差 ≤ 3 dB（受 SNR 扰动）。
3. RCS 观测：视角覆盖 < `minimum_aspect_coverage_deg` 时 `valid_feature_mask` 不含 `kRcs`。
4. 运动特征：从 `TrackStateSnapshot{speed=1800, acceleration=12}` 提取的 `speed_mps=1800`，`turn_radius_m` 对直线飞行（横向加速度 < 0.01 m/s²）标记为直线。
5. 极化观测：任一通道 RCS 缺失时 `valid_feature_mask` 不含 `kPolarization`。
6. 极化观测：`energy_difference_db` 对 `channel_1_rcs_dbsm=-3, channel_2_rcs_dbsm=-6` 输出 `≈3.0`（容差 1.5 dB，受 SNR/距离补偿扰动）。
7. 距离像观测：给定 5 个散射中心（`range_offset_m=[-4,-1,0,2,5]`, `rcs_dbsm=[0,-3,-6,-3,0]`），`bandwidth_hz=3e6`（分辨率=50m），输出 `length_m=9.0`（容差等于距离分辨率），`peak_count=5`。
8. 距离像观测：分辨率 > `max_range_resolution_m` 时 `valid_feature_mask` 不含 `kRangeProfile`。
9. 各提取器对无效输入返回 `quality=0`，不抛异常、不 abort。

**测试锚点：** 新增 `tests/unit/airborne_radar/ar_recognition_feature_test.cpp`。

### 阶段 3：数据库与匹配

**完成定义：**

1. 合法 JSON 文件被 `RecognitionFeatureDatabase::Load(path)` 成功加载，返回 `database_id` 和 `version`。
2. 结构错误 JSON（缺 `models`、`std==0`、`category_id` 引用不存在等）被拒绝，错误消息包含具体路径和字段。
3. JSON `schema_version` 不兼容时拒绝加载，不 fallback 到默认库。
4. `RecognitionMatcher` 对完全匹配模板的输入（`z=0`）返回 `similarity=1.0`。
5. `RecognitionMatcher` 对 `z=2`（距均值 2σ）返回 `similarity≈0.135`（`exp(-2)`, 容差 0.01）。
6. 至少一个 profile 的 `applicability` 条件不满足时，该 profile 不参与匹配（其得分为 0）。
7. 型号 `prior` 影响最终排序：两个得分相同的候选，prior 大的排第一。
8. 大类分数等于其下所有型号未归一化分数之和。
9. 动态加权：某维度 `quality=0` 时其权重不参与分母，其他维度按比例放大。
10. 空数据库（零型号）时 `QueryBestMatch` 返回空结果，不 crash。

**测试锚点：** 新增 `tests/unit/airborne_radar/ar_recognition_database_test.cpp`。

### 阶段 4：链路集成

**完成定义：**

1. `kLrr` 模式下，已确认航迹（`TrackStatus::kConfirmed`）参与识别；未确认航迹仅积累，`state=kAccumulating`。
2. 非 `kLrr` 模式下，识别不执行，所有航迹 `state=kDisabled`。
3. `ArRuntimeConfigPatch` 提交 `has_work_mode=true, work_mode=kLrr` 后下一成功周期开始积累。
4. 切出 `kLrr` 后停止新增积累，已确认结论保持 `result_hold_sec` 秒后变为 `kStale`。
5. 航迹 `kRecycled` 时对应 `RecognitionTrackState` 被清理，重新分配的 `association_key` 从零开始积累。
6. Runtime patch 提交失败（非法 `ArRecognitionConfig`）时，识别缓存、最新输出和数据库引用完整回滚。
7. `kSensorPoweredOff` 边界：识别保持最后结论至 `result_hold_sec`；超时后下一成功周期（即使仍在 `kLrr`）的识别快照从 `kStale` 开始。
8. `StepWithResult` → `ArCycleResult` 中 `recognition_summary` 的计数字段与 `TrackOutputFrame` 中航迹的 `recognition.state` 分布一致。
9. Trace 记录 `database_version`；Replay 检测版本不一致时标记 failure。
10. Replay 逐周期比较 `ArRecognitionResult` 所有字段（浮点容差 `1e-5f`），包括 `feature_scores` 各分量。

**测试锚点：** `tests/unit/airborne_radar/ar_core_controller_test.cpp`（扩展）、`tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp`（扩展）、`tests/contract/airborne_radar/ar_public_api_convenience_test.cpp`（扩展）。

### 阶段 5：效能验证

**完成定义：**

1. 弹道目标场景（速度 1800±300 m/s，高度 50000±12000 m，RCS -3±2 dBsm）：`kCategoryConfirmed` 或 `kModelConfirmed` 达成率 ≥ 80%，型号正确率（有真值时）≥ 70%。
2. 临近空间目标场景（速度 300±80 m/s，高度 25000±5000 m，RCS 2±2.5 dBsm）：`kCategoryConfirmed` 或 `kModelConfirmed` 达成率 ≥ 80%。
3. 未知目标场景（特征不匹配任何数据库模板）：`kUnknown` 率 ≥ 90%（拒绝误识）。
4. 低 SNR（< 6 dB）场景：`valid_feature_mask` 不含 RCS 和极化维度，但运动维度仍可用（`kMotion` 位为 1）。
5. 低带宽（< `minimum_bandwidth_hz`）场景：`valid_feature_mask` 不含 `kRangeProfile`。
6. 强干扰（`jammer_to_noise_db > 20`）场景：极化维度质量降为零或 `valid_feature_mask` 不含 `kPolarization`。
7. 短驻留（`recognition_dwell_sec=0.01`）场景：观测质量因子下降，`observation_count` 增长速率低于标称值。
8. 两类目标混合场景（弹道 + 临近空间同时存在）：类别混淆率 < 10%。
9. 模式切换序列（TWS→LRR→TWS→LRR）：第二次进入 LRR 时从零开始积累（不继承第一次的旧状态），第一次退出后的结论在 `result_hold_sec` 内保持。
10. 以上所有场景的 trace/replay 往返一致。

**测试锚点：** `tests/integration/airborne_radar/ar_recognition_scenario_test.cpp`（新增）、`tests/replay/airborne_radar/ar_rf_trace_session_test.cpp`（扩展）。
