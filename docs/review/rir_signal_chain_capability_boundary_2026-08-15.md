---
Status: draft
Date: 2026-08-15
Review-Baseline: `feature/remote-identification-radar-phase1` @ `95dc9921`
Decisions: 2026-08-15 需求方确认——CFAR 采用统计级口径（AR 同款，§3.1）；
  四项增益采用"派生默认 + 四个 dB 偏置参数"（§3.2 方案 A）；**同日二次定案：
  AR 与 RIR 完全独立（无模块间协作接口），RIR 自持检测 + 轻量关联（§4/§6/§7），
  阶段 1"航迹供给"接缝与等价性测试配对关系退役**。
Authority: 能力边界界定报告。对远程识别雷达需求文档所列九项信号链能力
（天线方向图仿真、回波功率计算、干扰功率计算、接收机噪声功率计算、目标信号增益、
噪声增益、杂波信号处理增益、干扰信号处理增益、恒虚警检测）逐项给出现状落点、
语义澄清与归属边界决策，供阶段 2 计划引用。不得替代各模块
`design.md`/`boundaries.md`；若本文与库实现冲突，以库为准。
---

# 远程识别雷达信号链能力边界界定（九项能力）

## 0. 定位与结论

需求文档要求远程识别雷达（RIR）具备九项信号链能力。经代码审查（证据见 §2），
结论分三层：

1. **两项已具备**：回波功率计算、接收机噪声功率计算（`RirRadarEquations` 副本，
   阶段 1 随迁，`RirController::ComputeSnrDb` 消费）——但为效能级简化口径
   （§3.3 所列四项简化）。
2. **七项代码存在于 AR 而未随迁**（审计 §4 判定为"AR 探测链本体"，刻意不迁）：
   天线方向图仿真、干扰功率计算、四项处理增益、恒虚警检测。其中方向图的
   **配置面已随迁**（`RirAntennaPatternConfig` + `enable_directional_pattern`），
   运行时从未评估。
3. **边界调和（2026-08-15 二次定案修订）**：九项能力在 RIR 的形态是**自持检测
   链**——RIR 迁移改造 AR 检测链模块后自行检测、轻量关联形成内部目标图像，
   识别积累挂内部航迹；AR 与 RIR 无模块间协作接口，阶段 1"航迹供给"接缝与
   等价性测试配对关系退役（§6）。

决策记录（2026-08-15 需求方确认，原两个澄清项已闭合）：
(1) **恒虚警检测采用 AR 同款统计级 CFAR**（Pfa 门限 + Swerling Pd + 蒙特卡洛
判决；§3.1），CA-CFAR 排除出 RIR 范围；(2) **四项增益采用"派生默认 + 四个 dB
偏置参数"**（§3.2 方案 A）——账本形态保持，叠加四个可配偏置，缺省 0 dB 与
保守账本逐位一致。

## 1. 审查范围与方法

- 范围：`src/remote_identification_radar/`（全量）、`include/1q/remote_identification_radar/`
  （全量）、`src/airborne_radar/signal/detection/`、`src/airborne_radar/signal/pipeline/
  DetectionExecution.cpp`、`src/airborne_radar/environment/`、
  `include/1q/electromagnetics/`（RfScene/RfLinkBudget）、RIR 文档四件套、
  《审计》与《阶段 1 计划》。
- 方法：沿用《审计》§1.1 三条边界标准（独立装备前提、消费公开输出、单一职责），
  对九项能力逐项回答三问：现状在哪、RIR 需要什么形态、归谁。
- AR 侧冻结约束继续有效（阶段 1 禁止触碰清单；本文档只读不改为前提）。

## 2. 九项能力现状映射（证据表）

| # | 需求能力 | AR 现状（位置） | RIR 现状 | common 现状 |
|---|---|---|---|---|
| 1 | 天线方向图仿真 | `signal/detection/AntennaPatternRuntime.h`（4 主瓣模型：高斯/抛物线/余弦幂/sinc²；扫描损失、旁瓣/后瓣电平、主瓣判定；header-only 纯函数）+ `BeamControlResolver.h`（有效波束宽度推导、安装系指向、离轴角→增益） | **配置已随迁未消费**：`RirHardwareConfig.h:47-72` 含 `RirAntennaPatternConfig`（模型/旁瓣/后瓣/扫描损失/孔径尺寸）与 `enable_directional_pattern`；`ComputeSnrDb` 固定用 `main_beam_gain_db` | 无 |
| 2 | 回波功率计算 | `RadarEquations.h:34-51`（对数域 + 单脉冲能量语义）+ `ArDetectionCellResolver.cpp:212-214`（线性域 + 时延/多普勒） | ✅ `RirRadarEquations.h:33-48` 副本；调用点 `RirController.cpp:54` 传播损耗硬编码 0 | 无（阶段 3 common 化候选，计划 R2） |
| 3 | 干扰功率计算 | 功率求解单源在 common（下列）；AR 持有单元聚合 `ArDetectionCellResolver.cpp:67-169`（目标单元时频重叠聚合、抗 RGPO 减半）与 `ArInterferenceObservationResolver`（J/N 门观测，AR 接收链语义） | ❌ 无；随迁字段 `RirReceiverConfig.interference_observation_jn_gate_db` 已于 2026-08-30 删除（本表裁定其属 AR 接收链语义、RIR 未立项消费），会话回放标识符随之 RIRC→RIRD | ✅ `1q/electromagnetics/RfScene.h:142-176` incident link 功率（`received_power_before_overlap_w`）、`TryEvaluateRfArrivalActivity`、`TryAggregateRfIncidentPower` |
| 4 | 接收机噪声功率计算 | `RadarEquations.h:53-60`（N₀=k·T₀·B·F）；检测单元口径用匹配滤波带宽（`ArDetectionCellResolver.cpp:221-223`） | ✅ `RirRadarEquations.h:51-58` 副本（带宽取发射 `bandwidth_hz`） | 无 |
| 5 | 目标信号增益 | `ArDetectionCellResolver.cpp:219-220` 脉压增益 `max(1, B·τ)`（仅作用于回波）+ `RadarEquations.h:62-67` 线性积累 `G=N`（v1 路径）+ Swerling Pd 内含 N 脉冲积累（v2 路径 `DetectResolvedCell` 用 `effective_pulse_count`） | ❌ 无（`dwell_quality` 驻留质量因子是特征退化语义，不是信号链增益） | 无 |
| 6 | 噪声增益 | 无独立参数：噪声按接收带宽计算（检测单元口径 = 匹配滤波带宽，与脉压增益相消的保守设计，SINR=Pc·G_pc/(N+J+C) 分母不加脉压增益） | ❌ 无 | 无 |
| 7 | 杂波信号处理增益 | 无独立增益：杂波功率进 SINR 分母。杂波源 = `environment/PropagationModel.cpp:158-173`（基线 3 dB + 降雨混合）→ `EnvironmentService` 快照 → `DetectionExecution.cpp:256` 等效噪声折算 | ❌ 无 | 无 |
| 8 | 干扰信号处理增益 | 无独立增益：`TryResolveCellInterference` 聚合后干扰功率进 SINR 分母 | ❌ 无 | 无 |
| 9 | 恒虚警检测 | `SignalDetector.cpp:23-103`：`DetectionPolicy.cfar_pfa`（默认 1e-6，`SignalEngineeringConfig.h:28-31`）→ `RadarEquations::ComputeThreshold`（Marcum Q）→ Swerling 0-4 Pd → 蒙特卡洛判决；附 `min_snr_db` 硬截断与 `min_detection_margin_db` 裕量门 | ❌ 无检测判决；SNR 仅做特征维度有效性门控（RCS 特征 SNR<6 dB 维度无效，`algorithms.md` 登记表） | 无 |

## 3. 语义澄清：需求术语 ↔ 代码实现口径

### 3.1 "恒虚警检测"的两种口径

本仓库现有实现（AR `SignalDetector`）是**统计级 CFAR**：由给定 Pfa 反解检测门限
（Marcum Q），配合 Swerling 0-4 起伏模型计算 Pd，再做确定性种子蒙特卡洛判决。
**不是 CA-CFAR**（无参考单元滑窗、无杂波图、无 OS/GO/SO 变体）；杂波环境自适应
由"环境快照杂波功率进噪声底"承担。

边界界定：
- **决策（2026-08-15 需求方确认）：RIR 采用与本库一致的统计级 CFAR**（AR 同款
  口径：Pfa 门限 + Swerling 0-4 Pd + 确定性种子判决，§5 决策 9）。
- CA-CFAR（参考单元滑窗）**排除出 RIR 范围**；若未来全库立项 CA-CFAR（AR/RIR
  双侧统一），另行评审，本报告不预设结论。

### 3.2 "四项增益"的实现形态与参数化决策（已确认）

需求所列"目标信号增益、噪声增益、杂波信号处理增益、干扰信号处理增益"在现有
代码中**不是四个独立可配参数**，而是一条**分项 SINR 账本**
（`ArDetectionCellResult`）：

- 目标信号：回波功率 × 脉压增益（`max(1, B·τ)`），多脉冲积累增益进入 Pd 计算
  （N 脉冲）或线性积累（v1 路径）；
- 噪声/干扰/杂波：各自功率直接进 SINR 分母，**不施加处理增益**（保守口径）。

**决策（2026-08-15 需求方确认）：采用"派生默认 + 四个 dB 偏置参数"（方案 A）。**
账本形态保持（物理派生量自动应用、不双算），在其上叠加四个可配偏置，**缺省
0 dB 时与保守账本逐位一致**：

| 需求能力 | 配置字段（hardware 域，建议 `RirSignalProcessingConfig`） | 作用位置 | 符号约定 | 典型用途 |
|---|---|---|---|---|
| 目标信号增益 | `target_processing_gain_db`（默认 0） | 分子：回波 × 脉压 × 10^(g/10)；积累 N 进 Pd 计算，不进账本分子 | 正 = 提升 SINR | 检测后处理额外增益 |
| 噪声增益 | `noise_processing_gain_db`（默认 0） | 分母：热噪声 × 10^(g/10) | 正 = 抬高噪声底（与 `noise_figure_db` 同向惯例） | 量化/带宽管理代价 |
| 杂波信号处理增益 | `clutter_suppression_gain_db`（默认 0） | 分母：杂波功率 ÷ 10^(g/10) | 正 = 抑制杂波（MTI 改善因子惯例） | MTI/杂波对消改善因子 |
| 干扰信号处理增益 | `jamming_suppression_gain_db`（默认 0） | 分母：干扰功率 ÷ 10^(g/10) | 正 = 抑制干扰（旁瓣对消惯例） | 旁瓣对消/ECCM 改善因子 |

集成方友好性依据：

1. **缺省即等价**：四参数默认 0 dB，行为与保守账本逐位一致——不填即正确，
   等价性回归可直接复用。
2. **命名与需求 1:1**：照需求文档条目即可对应填写，无需理解内部账本结构。
3. **符号跟随行业惯例**：每项按其物理量规格书的表述方向定义（改善因子正 dB
   为优、噪声代价正 dB 为劣），不引入统一符号换算的心智负担。
4. **派生量保持自动**：脉压（B·τ）与积累（N）永远自动应用，文档明示**禁止把
   派生量手填进偏置**（防双算）；额外链路损耗继续走既有损耗参数
   （`transmit_loss_db`/`receive_loss_db`），不与增益混用。
5. **静态配置面**：四参数入 hardware 域（装备常数语义），经 `RirSessionConfigBuilder`
   提交；不进运行时补丁（当前补丁面仅 work_mode/policy/sensor_enabled）。
   值域校验：有限且 [0, 40] dB，超界 `rir.validation.*` 拒绝；周期结果回报各
   分项有效功率与施加增益（可观测），replay 记录。

已否决的替代方案：

- **方案 B（纯硬编码，AR 现状）**：零旋钮、等价性最强，但集成方无法表达真实
  装备规格（如 25 dB MTI 改善因子），只能通过抬高 `noise_figure_db` 等参数
  间接逼近，污染物理语义。
- **方案 C（绝对覆盖模式）**：提供"手动增益/自动派生"开关切换。集成方须理解
  两种模式的适用边界，双算风险最高，测试与文档面翻倍。

### 3.3 RIR 当前 SNR 口径的四项简化（随迁自 AR kLrr）

`RirController::ComputeSnrDb`（`RirController.cpp:53-59`）的口径：

1. 传播损耗固定 0（调用点硬编码 `propagation_loss_db=0`）；
2. 天线增益固定主瓣峰值（无离轴方向图评估）；
3. 噪声底 = 纯热噪声（无杂波、无干扰）；
4. 无脉压/积累增益项。

该口径与 AR kLrr 原实现逐字段一致（等价性测试锁定），是**效能级链路预算**。
九项能力落地的实质 = 把该口径升级为**物理化驻留链路预算**。

## 4. 边界判定原则

1. **独立装备前提**（审计 §5.2 既定）：RIR 自带 hardware 域与链路预算实现，
   不得引用 AR internal（`RirRadarEquations` 副本模式延续至阶段 3 common 化）。
2. **单源不复制**：物理求解单源已存在于 common（RF incident link 功率）的，
   RIR 只消费不复制；仅"单元聚合/账本/判决"这类装备私有口径做 RIR 副本。
3. **语义定位（2026-08-15 二次定案，同日三次修订）**：九项能力构成 RIR 的
   **自持检测链**，产出内部目标图像（检测 → 关联/滤波/生命周期 → 内部航迹 →
   识别积累）。跟踪边界已突破 2-T 轻量子集：LAPJV 全局最优关联、航迹池、IMM
   列入下一步迁移；战术决策、ECCM 与对外点迹/航迹输出仍为非目标。
4. **AR 冻结**：本文档及后续阶段 2a/2b 实施不触碰 AR 树（阶段 2 删除 AR 识别
   耦合的计划另行执行，见《阶段 1 计划》§9）。

## 5. 逐项边界决策

| # | 能力 | 决策 | 归属与落点 |
|---|---|---|---|
| 1 | 天线方向图仿真 | **迁**（RIR 自建） | internal 副本改写 `AntennaPatternRuntime` → `RirAntennaPatternRuntime`（纯函数集，类型换 `RirAntennaPatternConfig`），消费已随迁的 `enable_directional_pattern`；波束指向/离轴角解析随驻留指向落地（决策 10）。**前置依赖：驻留指向**（无指向则离轴角无定义，只能维持主瓣峰值口径） |
| 2 | 回波功率计算 | **已有保留 + 补输入** | 保持 `RirRadarEquations` 副本；阶段 2a 将传播损耗从硬编码 0 改为环境输入（§5 决策 11 输入面） |
| 3 | 干扰功率计算 | **common 单源 + RIR 聚合副本** | incident link 功率继续单源在 `1q/electromagnetics`（不复制物理求解）；RIR 输入面扩展 incident links，识别驻留单元的时频重叠聚合按 `TryResolveCellInterference` 语义做 RIR 副本；AR 的 J/N 门干扰观测（`ArInterferenceObservationResolver`）**不迁**（AR 接收链观测语义，RIR 无此输出面） |
| 4 | 接收机噪声功率计算 | **已有保留** | 保持副本；带宽口径（发射带宽 vs 匹配滤波带宽）在阶段 2b 账本落地时统一为接收带宽口径并同步 algorithms.md |
| 5 | 目标信号增益 | **迁**（账本分子 + 偏置参数） | 驻留 SINR 账本：脉压增益 `max(1, B·τ)` + 驻留脉冲数 N（积累入 Pd）+ `target_processing_gain_db` 偏置（§3.2）。副本来源注明 commit |
| 6 | 噪声增益 | **账本分母项 + 偏置参数** | 热噪声按接收带宽进分母 + `noise_processing_gain_db` 偏置（§3.2）；口径统一见决策 4 |
| 7 | 杂波信号处理增益 | **账本分母项 + 偏置参数 + 环境输入** | 杂波功率经环境输入面进账本分母，`clutter_suppression_gain_db` 偏置（§3.2）；AR `PropagationModel`/`EnvironmentService` 属 AR 环境本体**不迁**，其 common 化列为阶段 3 评估（与计划 R2 同批） |
| 8 | 干扰信号处理增益 | **账本分母项 + 偏置参数** | 决策 3 聚合后的干扰功率进分母，`jamming_suppression_gain_db` 偏置（§3.2） |
| 9 | 恒虚警检测 | **迁（自持检测判决）** | RIR 落地 `RirSignalDetector`：Pfa 门限 + Swerling Pd + 确定性种子判决（副本改写 `RadarEquations` 检测子集 + `SignalDetector` 判决链）。**2026-08-15 二次定案修订**：检测判决驱动 RIR 自持目标图像（检测 → 轻量关联 → 内部航迹），特征维度门控升级为其下游消费；**不产点迹输出、不做战斗级跟踪/关联决策**；CA-CFAR 口径按 §3.1 澄清后再议 |
| 10 | 驻留指向（前置） | **阶段 2 先行**（审计 §5.1 既定评估项转正式） | 优先航迹选择逻辑迁入 RIR 自管波束调度，消费**内部航迹**；**排序语义变更**：威胁等级输入（AR 威胁分类 `target_type`）随独立性消失，改为"未识别优先 + 斜距次近"（原 `IsBetterLrrCandidate` 威胁排序不迁移）。RIR 不驱动任何外部雷达波束 |
| 11 | 环境输入面（前置） | **阶段 2 迁移改造** | `RirPropagationModel` 子集（传播损耗 + 杂波，副本改写 AR `PropagationModel` 语义）：植被场景事实入 `RirEnvironmentConfig`（激活空占位域；基线系数按 AR 环境域合约保留在实现内部），植被场景数据经 `RirCycleInput` 周期输入；RF 入射链路（common incident links）入 `RirCycleInput`。校验/issue codes/replay 兼容性随迁扩展 |
| 12 | 跟踪子集（三次修订） | **迁（识别消费闭包 → 多目标多模型跟踪）** | 已迁：单目标 KF、门限/最近邻关联、计数生命周期；**下一步迁入：LAPJV 全局最优关联、航迹池、IMM**。AR 来源 `KalmanPredictor/Updater`、`DataAssociation`/`DistanceMetric`/`LapjvSolver`、`TrackLifecycleManager`/`ITrackPool`/`BoostTrackPool`，IMM 数值源取 common `ImmFilter`。战术决策与 ECCM 不迁 |
| 13 | 外部航迹供给退役（二次定案新增） | **废除** | `RirTrackFeed` 公开输入与 `RirTrackFeedEntry`/`RirTrackFeedStatus` 出 public 面（破坏性变更）；识别积累的 `association_key`/`hit_count`/confirm 语义改由决策 12 内部航迹自产；`RirSceneTarget` 增 velocity/`target_name`/`target_swerling_type` |

**判定为"不做"的相邻能力**（显式否决，防蔓延；2026-08-15 二次定案修订）：
对外**点迹/量测输出**（检测量测仅内部消费，供关联滤波；`echo_delay_s`/
`two_way_doppler_shift_hz` 不出 public 面）、战术决策与 ECCM、
抗 RGPO 前沿跟踪减半（ECCM 语义属 AR）、J/N 门干扰观测输出、战术决策。
若后续需求要求，须重开边界评审。

## 6. 与现有非目标的调和（2026-08-15 二次定案修订）

需求方定案推翻了阶段 1"航迹供给"前提：AR 与 RIR **无模块间协作接口**，
RIR 自持检测 + 轻量关联。boundaries.md 非目标 #4（"不实现探测、关联、跟踪；
航迹由外部雷达供给"）按以下条款改写（2-S 落地时同步）：

1. **"探测/关联/跟踪"的语义精化**：RIR 具备自持检测链与跟踪（已迁单目标 KF +
   门限关联 + 计数生命周期；LAPJV/航迹池/IMM 列入下一步迁移）；**否决的是
   战术决策、ECCM 与对外点迹/航迹输出**。
2. **"航迹由外部雷达供给"条款删除**：`RirTrackFeed` 公开输入退役（决策 13），
   识别积累挂内部航迹；与 AR 的唯一关系是同库共存、互不引用。
3. boundaries.md F1/F2 节"本模块无探测链"表述改为：**RIR 具备自持检测链**
   （方向图/分项 SINR 账本/统计级 CFAR，检测量测仅内部消费，不对外发布点迹）。
4. 检测判决不得解释为对外"目标发现"事件（输出面只有识别结论与效能摘要）。

## 7. 分阶段落地（已展开为
`docs/review/remote_identification_radar_migration_status_2026-08-15.md`）

> 时序约束（v2 修订）：等价性测试在迁移期（2-M/2-T 纯增量旁路）保持可跑并
> 持续绿；2-S 输入面重构时**直接删除**（AR↔RIR 配对关系不存在，无转生测试）。
> AR 侧删除（2-C）后置为收尾段。

- **阶段 2-M（迁移改造）**：决策 1-8、11——检测链与环境子集副本改写进 RIR
  （旁路新增不接线）：雷达方程全集、方向图 + 波束控制、检测单元 + 干扰聚合、
  统计级 CFAR 判决器、传播模型子集、量测误差子集 + 四增益偏置参数。
- **阶段 2-T（跟踪迁移）**：决策 12——KF/最近邻关联/生命周期子集已迁；
  LAPJV/航迹池/IMM 按唯一迁移文档下一步迁入。
- **阶段 2-S（自持化重构）**：决策 9/10/13——输入面重构（`RirTrackFeed` 退役、
  场景目标补速度/真值字段）、流水线接线（检测→关联→内部航迹→识别）、
  驻留排序语义变更、replay 破坏性版本、场景测试重锚定、文档四件套改写、
  等价性测试删除。
- **阶段 3（common 化，计划 R2 既定扩展）**：已完成。LAPJV / `RirRadarEquations`
  与 AR `RadarEquations` / 天线方向图已收敛至 `src/common/`；检测单元/干扰聚合/跟踪子集
  维持不动或缓；`max_range_m`/`recognition_dwell_sec` 四域归位已完成
  （识别 policy → mission 域）。执行记录见
  `common_consolidation_execution_plan_2026-08-15.md`。

## 8. 验收标准（阶段 2 各段完成时）

1. 九项能力各有唯一归属且与本报告 §5 一致；RIR 无任何 AR include（不变式）。
2. 方向图：4 模型 × 主瓣/旁瓣/后瓣/扫描损失边界单测全绿；
   `enable_directional_pattern=false` 时与旧口径（主瓣峰值）逐位一致。
3. 账本：热噪声/干扰/杂波分项单独激励的回归测试；无环境输入时回退旧行为。
4. 检测门：Pfa 统计校验（虚警率与配置 Pfa 偏差在抽样容差内）；种子确定性
   （同种子同输入同判决，replay 兼容）。
5. 门控与自持链路：检测门驱动与 6 dB 回退两种模式行为差异有用例覆盖；
   检测量测/内部航迹不出 public 面（`RirTrackFeed*` 零残留）；
   algorithms.md/boundaries.md/data-flow.md 同步更新。
6. 增益偏置：四参数默认 0 dB 与保守账本逐位一致（等价回归）；超界值校验
   拒绝用例；分项有效功率与施加增益的回报/replay 记录有用例。
7. 全量 `/completeness-review`（release）通过；等价性测试按 §7 时序约束在 2-S
   删除（无转生测试）。

## 9. 风险

| 编号 | 风险 | 处置 |
|---|---|---|
| R-SN1 | SNR 口径升级与等价性测试锁定的时序冲突 | §7 时序约束：先删 AR 侧退役测试，或配置开关保持旧口径 |
| R-SN2 | "恒虚警检测"需求语义不确定（统计级 vs CA-CFAR） | **已决策（2026-08-15）**：统计级 CFAR（AR 同款），CA-CFAR 出范围 |
| R-SN3 | "四项增益"若要求独立可配参数，与全库保守账本口径不一致 | **已决策（2026-08-15）**：方案 A 四偏置参数（§3.2），缺省 0 dB 等价保守账本，阶段 2b 落地 |
| R-SN4 | 杂波/传播模型双源（AR environment 本体 vs RIR 环境输入） | 阶段 2b 以输入面解耦（RIR 不迁模型）；阶段 3 common 化收敛 |
| R-SN5 | 检测门蒙特卡洛随机性 vs replay 确定性 | 种子管理沿用 AR 先例（`SetRandomSeed`，replay 状态含种子） |
| R-SN6 | 驻留指向迁移后两雷达协同驻留编排 | 沿用审计 §5.1：调用方编排，不建模块间波束服务接口 |
| R-SN7 | 自持检测随机性 → 观测序列抖动影响识别积累 | 积累窗口天然吸收；种子确定性；Pd 高于门限的稳定场景入回归（v2 计划 R-Q1） |
| R-SN8 | `RirTrackFeed` 公开退役的破坏性变更波及调用方 | 2-S 一次到位不设 compat 层；examples/tests 同批改造（v2 计划 R-Q2） |
