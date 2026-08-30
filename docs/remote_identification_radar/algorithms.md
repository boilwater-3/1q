---
Status: active
Last-reviewed: 2026-08-30
Authority: RIR 算法登记与实现边界
Answers: RIR 每个算法做什么、实现边界在哪、哪些反直觉、哪些刻意不做
---

# Remote Identification Radar 算法

阶段 1 算法为 AR 远程识别子系统（kLrr）的平移改写；阶段 2-S 起识别积累改挂
RIR 自持链路生产的内部航迹，不再消费外部航迹供给。

## 算法登记表

| 算法 | 位置 | 输入 → 输出 | 边界与反直觉点 |
|---|---|---|---|
| 地球遮挡门控 | `runtime/RirController.cpp` → `common/geometry/EarthOccultation` | 平台 ECEF + 目标 ENU 还原 ECEF → 穿地则排除（不入检测候选） | 有限弦-圆球，R=6371 km，相切算遮挡；k 因子不进本门；排在可扫描体积与 SNR 之前；ENU→ECEF 失败则跳过本门 |
| 波束状态解析 | `dwell/RirBeamControl.h` → `common/radar/FrozenBeamResolve.h` | 驻留调度给定波束中心 + 目标视线角 + 天线配置 → 有效宽度/指向/单程增益 | 冻结变体 common 单源（`normalize_azimuth_delta=true`）；调度器给指向、RIR 信指向；视线角有效性 = 位置范数 > 0.1 m；无效回退主瓣峰值增益 |
| 自发射构建 | `dwell/RirEmissionFactory.cpp` | hardware + 周期上下文 → `RfSceneEmission` | 无 ECCM；功率包络钳制；ECEF 波束指向；载频固定取 `transmitter.frequency_hz`（`RirEmissionFactory.cpp:46-49` 的 `ResolveCarrierHz` 忽略周期索引）——频率计划/跳频未实现、周期索引暂不参与解析；驻留窗脉冲数按 ceil(窗/PRI) 计（AR PrepareRfCycle 同口径），下限 1 — **可提取核心，阶段 3b 未迁** |
| 接收机状态 | `dwell/RirReceiverStateBuilder.cpp` | 自发射 + hardware → `RirReceiverOperatingState` | 与 AR 同口径 RF 接收机参数；供前端聚合与 detection cell — **可提取核心，阶段 3b 未迁** |
| RF 前端求解 | `dwell/RirRfFrontEndResolver.cpp` | 合并场景（外部 + 自发射）+ 接收机 → incident links | 按 emission_id 排序；饱和标志独立暴露；集成方只供外部 emission |
| 有效 RCS | `dwell/RirEffectiveRcs.cpp` → `common/rcs/RcsPhysics`（`ComputeMixedPhysicalRcsM2`） | 场景目标 + 视线角 + `rcs_physics` → m² | 混合编排 common 单源；默认开启且 mix=1（随视线角/载频变，非扫描）；`enable=false` 或 mix=0 或 `carrier_hz<=0` 回退 input RCS；写入 detection cell 目标 `rcs_m2` |
| 逐目标大气物理损耗 | `runtime/RirController.cpp` → `common/atmosphere`（`ComputeTargetAtmosphericPhysicsLossDb`） | 周期载频 + 平台/目标几何 + 气象观测 → 大气附加损耗 dB | common 标量胶水；enable/k_factor 留模块侧；叠加进全局植被/天气损耗；`enable_physical_model=false`（默认）时为 0 |
| 检测单元求解 | `dwell/RirDetectionCellResolver.cpp` → `common/radar/DetectionCellResolver` | 目标回波事实 + 库内 incident links + 增益偏置 → 分项 SINR 账本 | common 单源；RIR 恒 `anti_rgpo=false`；四增益缺省 0 dB、均经 [0, 40] dB 钳位（`DetectionCellResolver.cpp:20,24-27`）；`noise_processing_gain_db` 语义为乘性抬高噪声底（进分母乘热噪，`DetectionCellResolver.cpp:257`），与其余三项（目标增益乘分子 `:262`、干扰/杂波抑制除分母项 `:258-259`）方向不对称——配置按「正值=劣化」理解；杂波瓦经 `ComputeEquivalentClutterNoiseW` |
| 验收旁路 MTI/MTD | `common/radar/MtiMtdAcceptanceBank` ← `RirAcceptanceRecords` | cell 功率 + PRF/载频 + 可选干扰单音 → 8 路派生与 MTI/MTD 增益 | **不进 SINR/Pd/航迹**；N=8、2 脉冲、σ_v=0.25 m/s 核内常量；无链路多普勒则干扰通道写 `无`；关验收开关时不求值 |
| 统计级 CFAR | `dwell/RirSignalDetector.cpp` → `common/radar/StatisticalCfarDetector` | SNR + Swerling + Pfa → Pd → 蒙特卡洛判决 | 判决编排 common；不是 CA-CFAR；生效积累脉冲数 = min(`policy.detection.pulse_count`, 驻留窗内可容脉冲)——检测单元路径按接收窗内回波可达数截断（`DetectionCellResolver.cpp:236-239`），默认驻留 0.05 s（`RirMissionConfig.h:32`）× PRF 300 Hz（`RirHardwareConfig.h:107`）窗内可容 15、policy 缺省 10（`RirPolicyConfig.h:32`）→ 生效 10，相对驻留积累能力损失约 1.8 dB（10·log10(15/10)）；**6 dB 真值回退门不在本类**（`RirController` 仿真脚手架） |
| 量测误差 | `dwell/RirMeasurementErrorModel.h` | SNR + 波束宽度 + 带宽 → 距离/角度标准差 + 偏置 | 输入 SNR = 单脉冲判决 SINR + 10·log10(N) 积累折算（N=生效积累脉冲数，cell 取窗口截断值/回退取 policy；2026-08-30 修复旧口径直接喂单脉冲致误差偏大 √N）；2026-08-30 bias/std 拆分：std 只含随机项，距离偏置 20 m 沿视线、角偏 bw÷30 逐轴正偏（施加在量测均值侧——检测器门控模式采样前平移量测位置，SnrFallback 真值口径不施加，与不采样噪声同层级简化）；角度两轴 RMS 合成（std 与 bias 同口径）；只供内部关联/滤波 |
| LAPJV 全局最优关联 | `tracking/RirTrackAssociator.cpp` + `common/tracking/GatedSquareAssignment.h` + `RirLapjvSolver` | 检测量测 + 航迹种子 → 关联键/命中/新键 | 方阵增广核 common；马氏平方波门（缺省 9）；键单调不回收复用 |
| 单目标 KF | `tracking/RirTrackFilter.cpp` | 量测 + 先验状态 → 预测/更新后验 | 6 维 CV 状态；动态 R 更新；LLT 失败跳过更新；q/std 下限 0.001 钳制（AR SignalComponentFactory 工厂同口径） |
| IMM 双路径 | `tracking/RirImmFilter.cpp` | 量测/失配 + 先验状态 → 组合后验 | 数值核 common `ImmFilter<6,3>`；缺省双模型 CV 对数等距、以配置 `kalman_noise_diff_coeff` 为低端锚点（q=1 退化为 {1.0, 10.0}）；对角 0.95 转移；confirmed 命中激活、失配仅预测；命中更新走逐量测动态 R（与 AR 同口径，缺省量测噪声不参与数值）；UpdateConfig 在线热同步既有运行态（每模型 q/转移矩阵，AR SyncRuntimeTuning 同口径；模型数变化丢弃运行态、下次 confirmed 命中惰性重建）；缺省开启 |
| 航迹池与生命周期 | `tracking/RirTrackPool.cpp` → `common/tracking/ObjectPool`；`RirTrackLifecycle` → `TrackLifecyclePromote` | 关联量测 + 周期上下文 → 内部航迹 | 池/PromoteState FSM common；RIR 无 `kRecycled` 中间态（回收即 erase）；双重释放拒绝 |
| 驻留排序 | `runtime/RirController.cpp` | 上一周期内部航迹结论 + 场景目标 → 驻留候选顺序 | 未识别优先 + 斜距次近；威胁等级输入不参与；只决定候选顺序，不生成波束指向 |
| 驻留调度（库内） | `session/RirSession.cpp` + `common/radar/ScanScheduleRuntime.h` | 实际搜索扇区（子窗∩体积）+ scan_center + 指定任务状态 + 场景目标 → 本周期驻留波束中心 | 实际搜索扇区 = `mission.scan_window_deg` ∩ `orientation.steerable_volume_deg`（`internal/IntersectScanSector`），在其相对限位建波位 → center 平移 → 方位归一化（common `BuildScanPattern` 每轴采样上限 4096、超出截断，`ScanScheduleRuntime.h:84,98`）；扫描步长波束宽与检测门同口径（λ=c/f 两级回退 nominal→λ/L，2026-08-30 修复扫描链缺波长致 nominal=0 配置静默空表）；指定任务限位执行以体积为界（越界 kOutsideSteerableVolume），指定目标豁免子窗；非法体积/步长/空交集回退 scan_center；验收旁路另写 `rir_scan_pattern.csv`（未进指向） |
| 实际有效目标最大斜距 | `runtime/RirController.cpp` | 本周期入候选并成航迹的目标斜距 → `RirCycleResult.max_detected_slant_range_m` | 持航迹目标最大输入几何斜距；区别于 `mission.max_range_m`（径向粗筛门）——反映 SNR 链路预算下实际探测距离；无航迹为 0，出扇区目标不计入 |
| 指定识别任务（限时锁定） | `session/RirSession.cpp` | 指定（目标 ID + 窗口周期数）→ 任务生命周期（kPending/kAcquired/kExpired） | 镜像 AR designation 骨架；识别达成即任务完成回扫描（识别是离散结论，不持续跟随）；窗口耗尽作废（kAcquisitionTimeout） |
| 观测构造 | `recognition/RecognitionObservationBuilder.cpp` | 场景目标真值 + 内部航迹 + `RirObservationContext` → `RirFeatureSet` | 驻留质量因子作用于 RCS/极化/距离像（运动除外）；场景真值不得直接产生结论 |
| RCS 特征 | `recognition/RcsFeatureExtractor.cpp` | 视角样本 + 视线角 + SNR → `RirRcsObservation` | **最近邻插值不强制覆盖**；SNR < 6 dB 维度无效；覆盖下限由匹配阶段判定 |
| 运动特征 | `recognition/MotionFeatureExtractor.cpp` | 内部航迹 + 平台海拔 + 不确定度 → `RirMotionObservation` | 仅已确认航迹；横向加速度分解判直线/转弯半径；质量因子 = 10000/(10000+迹/3)（迹按三轴取均值归一化到每轴方差，参考 (100 m)²） |
| 极化特征（F1） | `recognition/PolarizationFeatureExtractor.cpp` | 双通道样本 + 视线角 + SNR/距离 → `RirPolarizationObservation` | 通道定义由数据库固定；强干扰（SNR 压低）维度不可用 |
| 极化验收旁路（L2） | `runtime/PolarizationAcceptanceS.cpp` | 最近邻样本（须 `has_cross_pol` 与 `has_phase_vv`）→ Span / `|det(S)|` / 去极化 / Graves ψτ | 只写 `rir_acceptance.log`，**未进识别**；缺 `has_*` 写暂无，不回退 L1 |
| 距离像特征（F2） | `recognition/RangeProfileFeatureExtractor.cpp` | 散射中心列表 + 带宽 + SNR → `RirRangeProfileObservation` | 分辨率 c/(2B) 超上限维度无效；粗单元不合并峰标识 |
| 数据库加载 | `recognition/RecognitionFeatureDatabase.cpp` | SQLite 文件 → 全量内存模板 | schema v1.1 自描述校验；运行期无连接；units `rcs != 'dBsm'` 拒绝 |
| 匹配 | `recognition/RecognitionMatcher.cpp` | 特征集 + profile 适用条件 → 候选排序/大类分数 | `s = exp(-0.5·z²)`；质量 0 维度不进分子分母；类别得分 = 大类内最佳型号得分（2026-08-30 核查 8.3：由成员求和改判，成员数量不加分） |
| 积累判定 | `recognition/RecognitionTracker.cpp` | 逐周期观测 → 结论状态机 | 分数 ≥ `acceptance_score` 且 margin 足且有效维度 ≥ 2 → `kModelConfirmed`；运动不能单独确认型号 |
| 特征量测帧组装（出口①透出，Stage B） | `runtime/RirController.cpp` + `recognition/RecognitionTracker.cpp`（UpdateCycle 采集出参） | 本周期有效特征观测 × 观测上下文 + 平台位置 → `RirFeatureMeasurementRecord` | 透出原则：只透出识别链实际构建且 mask ≠ 0 的观测（透出点在积累质量门之前——质量门只挡积累）；全维无效不产生；无库/超距/非识别模式周期帧为空；字段同值透出无换算 |
| 归属视图组装（Stage B） | `runtime/RirController.cpp` | 本周期航迹快照 → `RirTrackAttributionRecord`（键↔真值 + hit/位置/速度诊断） | 与出口②同循环覆盖全部快照（tentative/confirmed/lost）；结果层产品，不进输出帧；非执行周期空列表 |
| 发射帧组装（emission_frame，RF 链） | `runtime/RirController.cpp`（`ResolveRfCycle` 解析成功时） | 逐驻留自发射 → `RfEmissionFrame`（经 `RirCycleResult::emission_frame`） | `kIdentify` 且 RF 链解析成功时携带本周期**全部驻留**的实际发射（2026-08-30 核查 9.2：逐驻留一条，指向=该驻留波位，emission_id 帧内唯一化——首驻留保持周期号、其余高位标志编码；接收链仍按首驻留周期级一次求解；与 AR 同契约，供编排层汇集 RF scene）；解析失败为空帧，不虚构 |

## 反直觉点

1. **6 dB 回退模式**：`RirDetectionGateMode::kSnrFallback` 以 SNR ≥ 6 dB 替代
   CFAR 随机判决，且量测位置取真值（只携带误差协方差）；`kDetectorGate` 才
   执行蒙特卡洛判决与量测位置采样。
2. **KF 加速度口径**：加速度 = 滤波后验速度的周期间差分（物理加速度）。hit 时
   以上周期后验速度为基准、按距上次命中的实际经过时间折算（滑行周期后重命中
   不按单周期 dt 膨胀）；(重)置周期（建轨/失跟重捕）滤波速度以场景速度种子
   重初始化、无差分基准、加速度置零；miss 时 CV 外推速度不变、加速度归零。
   速度种子只用于（重）初始化，不进入差分基准，也不再写入航迹状态（2026-08-29
   审计修正：旧口径以本周期种子为基准，输出实为滤波速度滞后误差，方向与真值
   加速度相反、量级虚高，并曾透传进识别运动特征）。
3. **RF 链回退与回波能量基准（2026-08-30 统一）**：`ResolveRfCycle` 失败（hardware
   不完整等）或 detection cell 求解失败时，传播损耗/杂波仍按环境配置注入，但检测
   SNR 回退阶段 1 旧公式口径，且回退路径 `env.jam_noise_w` 恒 0
   （`RirController.cpp` 回退分支硬编码，不注入干扰）、不施四项处理增益/抑制——
   抗干扰能力仅检测单元路径具备，用回退口径评估抗干扰场景会高估门限；RF 链成功时
   走分项 SINR 账本（含外部 `rf_scene` 干扰）。**回波能量基准统一为 B·τ 脉压口径**
   （2026-08-30 裁定物理正确优先）：杂波等效噪声与回退路径目标回波均在 common
   参考脉宽方程（τ/13µs 能量缩放）之上叠加 `ComputePulseCompressionGainDb` =
   10·log10(max(1, B·τ))，与检测单元分子的脉压增益同基准——净修正量
   10·log10(B·13µs) 与 τ 无关（默认 B=4.5 MHz ≈ +17.67 dB）；修正前目标/杂波比
   曾恒差 B·13µs 倍（杂波被相对低估）。接收前端饱和（周期级，入射总功率越
   `maximum_linear_input_power_w`）为致盲语义：该周期全部目标不产生检测，记
   `kTargetReceiverFrontEndSaturated` 排除诊断。环境杂波
   （`vegetation_cover_profile≠kDisabled`）为逐目标主瓣地杂波最小物理模型
   （`RirSurfaceClutterModel`）：擦地角取主瓣俯仰半波束宽减目标仰角（主瓣离地
   归零），杂波面积取脉冲/波束限制较小者（距离单元 c/2B），σ₀ 按植被档位查表
   并随 sinψ 一阶折算，杂波回波走与目标同一雷达方程（2026-08-30 替换旧的
   会话级恒定 CNR 口径；σ₀ 表为 S 波段量级声明值，非实测标定）；该杂波回波按
   主瓣峰值增益近似（走 `RirRadarEquations::ComputeEchoPower_dBW` 用
   `main_beam_gain_db`），而同一 SINR 账本中目标回波用逐目标离轴增益——斜视
   驻留时杂波恒按主瓣中心增益估计，该口径差为已知近似。
4. **分项相似度按实际命中 profile 报告**：`feature_scores` 分项报告用判定
   实际得分的 profile（`RirMatchResult::best_profile_index` 传递、
   `RecognitionTracker` 消费；库热替换致下标越界时回退首 profile；
   2026-08-30 修复旧的 `profiles.front()` 恒取首个口径）。
5. **单候选 margin 恒过**：单候选时 `runner_up_score == 0`，margin 检查恒过。
6. **RCS 最近邻**：视角覆盖判定完全由数据库 profile 的
   `minimum_aspect_coverage_deg` 承担——样本网格仅需非空即产生 RCS 观测。
7. **关联键不回收**：航迹回收只删除内部航迹，`next_key` 继续单调递增；因此
   识别积累不需要检测 `hit_count` 回落，新键天然等于新目标。
8. **波束指向不追目标**：RIR 不会把给定波束中心重算或吸附到目标位置。方向图恒开
   （2026-08-29 还债：`enable_directional_pattern` 开关已删除），调度器指向与目标
   视线角的方位差经归一化后决定天线增益；指向偏离目标即按实际离轴角衰减，这是
   “调度器给指向、RIR 信指向”的可见后果。目标位置退化（范数 ≤ 0.1 m）时视线角无效，
   增益回退主瓣峰值（az=0/el=0 兜底角不做离轴衰减，AR 同口径）。检测候选另受
   主瓣覆盖门约束（候选须落在本周期某驻留指向的半功率宽内；TAS 边搜边跟：
   确认航迹每周期保留专用跟踪驻留）。
9. **实际搜索扇区 + 转台朝向**：空闲驻留波位在**实际搜索扇区**
   （`mission.scan_window_deg` ∩ `orientation.steerable_volume_deg`）相对限位上由 common
   内核构建，再经 `mission.scan_center_deg` 平移并方位归一化；子窗缺省无界时交集退化为
   体积，默认体积 ±60/±30 + center (0,0) 与重构前绝对限位逐位等价；跨界扇区通过 center
   表达。检测候选按实际搜索扇区裁剪（指定目标在体积内豁免子窗）；指定任务越出**体积**
   时回扫描（`kOutsideSteerableVolume`），转台重新瞄准后恢复。
10. **识别达成即任务完成**：指定识别任务在识别状态达 `kCategoryConfirmed`/
    `kModelConfirmed` 后即完成（下一周期指定清零、回到扫描），不做持续跟随——
    与 AR（捕获后持续跟随航迹）的差异源于识别是离散结论而非连续跟踪。
11. **出口①透出点在质量门之前**：观测质量低于 `kMinimumObservationQuality` 的
    周期不计为有效积累，但其特征量测**照常出口**（出口①是量测产品，质量门只
    约束识别积累）；相反，超识别最大距离的键在观测构建之前即被跳过——特征帧
    里根本没有该键的记录（透出原则：只透出实际构建的观测）。
12. **出口①方位角自东起量**：`look_az_deg` 沿用内部 ENU 约定（az 自 +x 东起量），
    与 fusion 方位通道自北约定不同——换算归 `AdaptRirFeatureMeasurementsToDetectionRecords`
    （wrap(90° − az)），库内不做跨系转换。适配器同时用斜距+视线角+平台原点还原融合
    位置量测（失败维持仅方位+原点；2026-08-27 rir-adapter-position）。

## 非目标（刻意不实现的算法）

1. 在线残差驱动的自动后端切换/在线学习/自适应权重。
2. 信号级 IQ / 全波散射求解、ISAR 二维成像、微动特征。
3. CA-CFAR（参考单元滑窗/杂波图/OS-GO-SO）。
4. 战术决策、ECCM 决策与反欺骗（跟踪升级 N1-N7 已落地 LAPJV/池化/IMM，见
   `docs/review/remote_identification_radar_migration_status_2026-08-15.md`）。
5. 对外点迹/量测输出、外部雷达波束控制接口；驻留指向由库内驻留调度器派生且
   仅库内消费，不向任何外部雷达（含 AR）输出或反馈波束控制。
6. 地球椭球/地形/电波视距（4/3 k 因子）遮挡——本门为圆球几何通视；k 只进大气损耗。

## 证据

- 检测/误差/关联/滤波/生命周期：`tests/unit/remote_identification_radar/rir_signal_detector_test.cpp`、
  `rir_measurement_error_test.cpp`、`rir_track_associator_test.cpp`、
  `rir_track_filter_test.cpp`、`rir_track_lifecycle_test.cpp`
- 大气物理链与环境杂波口径：`tests/unit/remote_identification_radar/rir_atmospheric_physics_test.cpp`
  （配置合同/校验负例/开关闭差分）、`rir_surface_clutter_model_test.cpp`
  （逐目标杂波特征化：σ₀ 档位序/擦地角几何/距离衰减/频率响应/退化输入免疫）
- 自持链路与输入面：`tests/unit/remote_identification_radar/rir_self_contained_pipeline_test.cpp`、
  `rir_self_contained_validation_test.cpp`、`rir_emission_factory_test.cpp`、
  `rir_receiver_state_builder_test.cpp`、`rir_rf_front_end_resolver_test.cpp`、
  `rir_effective_rcs_test.cpp`、`rir_rf_physical_parity_test.cpp`
  （AR↔RIR RF 物理链对账：基线/干扰/杂波/饱和）、`rir_emission_frame_test.cpp`
  （emission_frame 出口）
- 双产品出口（Stage B）：`tests/unit/remote_identification_radar/rir_feature_measurement_test.cpp`
  （出口①字段透出/平台位置双路径/透出原则/会话级拒绝周期）、
  `rir_track_attribution_test.cpp`（键↔真值映射/全快照覆盖/非执行周期空列表）、
  `rir_platform_position_validation_test.cpp`（fail-closed 与存在性一致性）
- 提取器与门控：`tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp`、
  `rir_search_sector_test.cpp`、`rir_earth_occultation_test.cpp`（地球遮挡硬门）；
  判定核 `tests/unit/common/common_earth_occultation_test.cpp`
- 匹配与库契约：`tests/unit/remote_identification_radar/rir_recognition_database_test.cpp`
- 场景/型号效能：`tests/integration/remote_identification_radar/`
- replay V2：`tests/replay/remote_identification_radar/rir_replay_codec_roundtrip_test.cpp`
