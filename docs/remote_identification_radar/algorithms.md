---
Status: active
Last-reviewed: 2026-08-20
Authority: RIR 算法登记与实现边界
Answers: RIR 每个算法做什么、实现边界在哪、哪些反直觉、哪些刻意不做
---

# Remote Identification Radar 算法

阶段 1 算法为 AR 远程识别子系统（kLrr）的平移改写；阶段 2-S 起识别积累改挂
RIR 自持链路生产的内部航迹，不再消费外部航迹供给。

## 算法登记表

| 算法 | 位置 | 输入 → 输出 | 边界与反直觉点 |
|---|---|---|---|
| 波束状态解析 | `dwell/RirBeamControl.h` | 驻留调度给定波束中心 + 目标视线角 + 天线配置 → 有效宽度/指向/单程增益 | 调度器给指向、RIR 信指向：不重算、不吸附；指向与视线角同为雷达局部 ENU 系（`az∈[-180,180]`、`el∈[-90,90]`），差值即离轴角 |
| 自发射构建 | `dwell/RirEmissionFactory.cpp` | hardware + 周期上下文 → `RfSceneEmission` | 无 ECCM；功率包络钳制；ECEF 波束指向；载频由频率计划/周期索引解析 |
| 接收机状态 | `dwell/RirReceiverStateBuilder.cpp` | 自发射 + hardware → `RirReceiverOperatingState` | 与 AR 同口径 RF 接收机参数；供前端聚合与 detection cell |
| RF 前端求解 | `dwell/RirRfFrontEndResolver.cpp` | 合并场景（外部 + 自发射）+ 接收机 → incident links | 按 emission_id 排序；饱和标志独立暴露；集成方只供外部 emission |
| 有效 RCS | `dwell/RirEffectiveRcs.cpp` | 场景目标 + 视线角 + `rcs_physics` → m² | AR 同口径 Swerling/视角/物理配置；写入 detection cell 目标 `rcs_m2` |
| 检测单元求解 | `dwell/RirDetectionCellResolver.cpp` | 目标回波事实 + 库内 incident links + 增益偏置 → 分项 SINR 账本 | 干扰按目标单元时频重叠聚合；四增益偏置缺省 0 dB 等于保守账本；自身发射身份不计干扰 |
| 统计级 CFAR | `dwell/RirSignalDetector.cpp` | SNR + Swerling + Pfa → Pd → 蒙特卡洛判决 | 不是 CA-CFAR；`min_snr_db` 硬截断、`min_detection_margin_db` 可靠性门；同种子同判决 |
| 量测误差 | `dwell/RirMeasurementErrorModel.h` | SNR + 波束宽度 + 带宽 → 距离/角度标准差 | 距离偏置 20 m；角度两轴 RMS 合成；只供内部关联/滤波 |
| LAPJV 全局最优关联 | `tracking/RirTrackAssociator.cpp` + `tracking/RirLapjvSolver.cpp`（common 单源 `src/common/optimization/LapjvSolver` 适配） | 检测量测 + 航迹种子 → 关联键/命中/新键 | 马氏平方波门（缺省 9）兼作未分配代价；方阵增广 + 哑行/列承担未分配；门外对填拒绝代价；键单调不回收复用 |
| 单目标 KF | `tracking/RirTrackFilter.cpp` | 量测 + 先验状态 → 预测/更新后验 | 6 维 CV 状态；动态 R 更新；LLT 失败跳过更新 |
| IMM 双路径 | `tracking/RirImmFilter.cpp` | 量测/失配 + 先验状态 → 组合后验 | 数值核 common `ImmFilter<6,3>`；缺省双模型 CV {1.0, 10.0} 对数等距；对角 0.95 转移；confirmed 命中激活、失配仅预测；缺省关闭 |
| 航迹池与生命周期 | `tracking/RirTrackPool.cpp` + `tracking/RirTrackLifecycle.cpp` | 关联量测 + 周期上下文 → 内部航迹 | hit/miss 计数、confirm/lost/回收；lost 重捕获重置 KF；回收不回收关联键；槽位复用经 `generation` 单调递增标识；双重释放拒绝 |
| 驻留排序 | `runtime/RirController.cpp` | 上一周期内部航迹结论 + 场景目标 → 驻留候选顺序 | 未识别优先 + 斜距次近；威胁等级输入不参与；只决定候选顺序，不生成波束指向 |
| 驻留调度（库内） | `session/RirSession.cpp` + `common/radar/ScanScheduleRuntime.h` | 指定任务状态 + 扫描策略配置 + 场景目标 → 本周期驻留波束中心 | 无任务按扫描策略逐周期推进（common 扫描内核，与 AR 同口径）；指定识别任务窗口内对准指定目标；非法限位回退零位 |
| 指定识别任务（限时锁定） | `session/RirSession.cpp` | 指定（目标 ID + 窗口周期数）→ 任务生命周期（kPending/kAcquired/kExpired） | 镜像 AR designation 骨架；识别达成即任务完成回扫描（识别是离散结论，不持续跟随）；窗口耗尽作废（kAcquisitionTimeout） |
| 观测构造 | `recognition/RecognitionObservationBuilder.cpp` | 场景目标真值 + 内部航迹 + `RirObservationContext` → `RirFeatureSet` | 驻留质量因子作用于 RCS/极化/距离像（运动除外）；场景真值不得直接产生结论 |
| RCS 特征 | `recognition/RcsFeatureExtractor.cpp` | 视角样本 + 视线角 + SNR → `RirRcsObservation` | **最近邻插值不强制覆盖**；SNR < 6 dB 维度无效；覆盖下限由匹配阶段判定 |
| 运动特征 | `recognition/MotionFeatureExtractor.cpp` | 内部航迹 + 平台海拔 + 不确定度 → `RirMotionObservation` | 仅已确认航迹；横向加速度分解判直线/转弯半径；质量因子 = 10000/(10000+不确定度) |
| 极化特征（F1） | `recognition/PolarizationFeatureExtractor.cpp` | 双通道样本 + 视线角 + SNR/距离 → `RirPolarizationObservation` | 通道定义由数据库固定；强干扰（SNR 压低）维度不可用 |
| 距离像特征（F2） | `recognition/RangeProfileFeatureExtractor.cpp` | 散射中心列表 + 带宽 + SNR → `RirRangeProfileObservation` | 分辨率 c/(2B) 超上限维度无效；粗单元不合并峰标识 |
| 数据库加载 | `recognition/RecognitionFeatureDatabase.cpp` | SQLite 文件 → 全量内存模板 | schema v1.1 自描述校验；运行期无连接；units `rcs != 'dBsm'` 拒绝 |
| 匹配 | `recognition/RecognitionMatcher.cpp` | 特征集 + profile 适用条件 → 候选排序/大类分数 | `s = exp(-0.5·z²)`；质量 0 维度不进分子分母；类别得分 = 成员型号未归一化之和 |
| 积累判定 | `recognition/RecognitionTracker.cpp` | 逐周期观测 → 结论状态机 | 分数 ≥ `acceptance_score` 且 margin 足且有效维度 ≥ 2 → `kModelConfirmed`；运动不能单独确认型号 |
| 特征量测帧组装（出口①透出，Stage B） | `runtime/RirController.cpp` + `recognition/RecognitionTracker.cpp`（UpdateCycle 采集出参） | 本周期有效特征观测 × 观测上下文 + 平台位置 → `RirFeatureMeasurementRecord` | 透出原则：只透出识别链实际构建且 mask ≠ 0 的观测（透出点在积累质量门之前——质量门只挡积累）；全维无效不产生；无库/超距/非识别模式周期帧为空；字段同值透出无换算 |
| 归属视图组装（Stage B） | `runtime/RirController.cpp` | 本周期航迹快照 → `RirTrackAttributionRecord`（键↔真值 + hit/位置/速度诊断） | 与出口②同循环覆盖全部快照（tentative/confirmed/lost）；结果层产品，不进输出帧；非执行周期空列表 |
| 发射帧组装（emission_frame，RF 链） | `runtime/RirController.cpp`（`ResolveRfCycle` 解析成功时） | 本周期自发射 → `RfEmissionFrame`（经 `RirCycleResult::emission_frame`） | `kIdentify` 且 RF 链解析成功时携带本周期实际发射（与 AR 同契约，供编排层汇集 RF scene）；解析失败为空帧，不虚构 |

## 反直觉点

1. **6 dB 回退模式**：`RirDetectionGateMode::kSnrFallback` 以 SNR ≥ 6 dB 替代
   CFAR 随机判决，且量测位置取真值（只携带误差协方差）；`kDetectorGate` 才
   执行蒙特卡洛判决与量测位置采样。
2. **KF 加速度口径**：hit 时加速度 = KF 后验速度与本周期场景速度种子之差/dt；
   miss 时 CV 外推速度不变、加速度归零。该口径与 AR 轻量跟踪子集一致。
3. **RF 链回退**：`ResolveRfCycle` 失败（hardware 不完整/前端饱和等）或 detection cell
   求解失败时，传播损耗/杂波/干扰仍按环境配置注入，但检测 SNR 回退阶段 1 旧公式
   口径；RF 链成功时走分项 SINR 账本（含外部 `rf_scene` 干扰）。
4. **第一个 profile 报告**：`feature_scores` 分项报告用型号的第一个 profile
   （`profiles.front()`），而非实际命中得分的 profile——多 profile 型号的分项
   报告可能与判定所用 profile 不一致（判定路径本身正确）。
5. **单候选 margin 恒过**：单候选时 `runner_up_score == 0`，margin 检查恒过。
6. **RCS 最近邻**：视角覆盖判定完全由数据库 profile 的
   `minimum_aspect_coverage_deg` 承担——样本网格仅需非空即产生 RCS 观测。
7. **关联键不回收**：航迹回收只删除内部航迹，`next_key` 继续单调递增；因此
   识别积累不需要检测 `hit_count` 回落，新键天然等于新目标。
8. **波束指向不追目标**：RIR 不会把给定波束中心重算或吸附到目标位置。方向图
   开启时，调度器指向与目标视线角的差值直接决定天线增益；指向偏离目标即按
   实际离轴角衰减，这是“调度器给指向、RIR 信指向”的可见后果。
9. **扫描策略与 AR 同口径**：空闲驻留波位序列由 common 扫描内核构建（限位/步长
   （波束宽度 × step_scale）/起点/顺序），第 N 周期取第 `(N-1) % size` 个波位——
   与 AR 同一扫描策略；`enable_directional_pattern=false` 时驻留中心不影响增益
   （阶段 1 缺省兼容），仅经 `RirCycleResult::dwell_center_deg` 暴露。
10. **识别达成即任务完成**：指定识别任务在识别状态达 `kCategoryConfirmed`/
    `kModelConfirmed` 后即完成（下一周期指定清零、回到扫描），不做持续跟随——
    与 AR（捕获后持续跟随航迹）的差异源于识别是离散结论而非连续跟踪。
11. **出口①透出点在质量门之前**：观测质量低于 `kMinimumObservationQuality` 的
    周期不计为有效积累，但其特征量测**照常出口**（出口①是量测产品，质量门只
    约束识别积累）；相反，超识别最大距离的键在观测构建之前即被跳过——特征帧
    里根本没有该键的记录（透出原则：只透出实际构建的观测）。
12. **出口①方位角自东起量**：`look_az_deg` 沿用内部 ENU 约定（az 自 +x 东起量），
    与 fusion 方位通道自北约定不同——换算归 `AdaptRirFeatureMeasurementsToDetectionRecords`
    （wrap(90° − az)），库内不做跨系转换。

## 非目标（刻意不实现的算法）

1. 在线残差驱动的自动后端切换/在线学习/自适应权重。
2. 信号级 IQ / 全波散射求解、ISAR 二维成像、微动特征。
3. CA-CFAR（参考单元滑窗/杂波图/OS-GO-SO）。
4. 战术决策、ECCM 决策与反欺骗（跟踪升级 N1-N7 已落地 LAPJV/池化/IMM，见
   `docs/review/remote_identification_radar_migration_status_2026-08-15.md`）。
5. 对外点迹/量测输出、外部雷达波束控制接口；驻留指向由库内驻留调度器派生且
   仅库内消费，不向任何外部雷达（含 AR）输出或反馈波束控制。

## 证据

- 检测/误差/关联/滤波/生命周期：`tests/unit/remote_identification_radar/rir_signal_detector_test.cpp`、
  `rir_measurement_error_test.cpp`、`rir_track_associator_test.cpp`、
  `rir_track_filter_test.cpp`、`rir_track_lifecycle_test.cpp`
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
- 提取器与门控：`tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp`
- 匹配与库契约：`tests/unit/remote_identification_radar/rir_recognition_database_test.cpp`
- 场景/型号效能：`tests/integration/remote_identification_radar/`
- replay V2：`tests/replay/remote_identification_radar/rir_replay_codec_roundtrip_test.cpp`
