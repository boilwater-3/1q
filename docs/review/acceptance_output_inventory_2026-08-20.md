# 验收输出项全仓统计（2026-08-20）

## 1. 文档说明

- **目的**：对照验收测试清单逐项统计"该信息当前能否被记录/输出"。与《红外载荷与远程识别雷达需求源码对应关系.md》互补：映射文档回答"功能是否实现"，本档回答"**信息是否被输出**"（公开结构体 / 日志 / replay 遥测），还是只算在内部变量里。
- **触发背景**：新增 RIR 验收日志宏 `RIR_ACCEPTANCE_LOG`（`[RirAccept]` 事件流，CMake 开关 `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG` 默认 OFF，`src/remote_identification_radar/runtime/RirAcceptanceLog.h`），本文档即其调用点接线依据。
- **勘误**（原始清单笔误按下列归一理解）：宽市场/展市场→宽/窄视场；拖把量→脱靶量；大负面→大幅面；斜方差→协方差；弟弟目标→低动态目标；dd 模型→弹道模型；"扫描操角"→扫描掠角（定义待确认，见 §6）。
- 行号以 2026-08-20 工作区为准，路径相对仓库根。

## 2. 统计口径与前提

| 档 | 含义 |
|---|---|
| A | 有公开结构体 / API / replay 输出（列头文件与字段） |
| B | 有日志输出（列通道与 file:line；注意开关门控） |
| C | 仅内部计算，无任何出口（列计算点） |
| D | 部分满足（说明缺什么） |
| E | 未实现 / 未找到 |

各档可并存。日志前提（三个验收日志开关默认 OFF，文件后端默认 ON）：

- `ONEQ_ENABLE_FILE_LOG`（默认 ON）：`PROJECT_LOG_*` 落盘 `1q_library.log`。
- `ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG`（默认 OFF）：`[SbirsAccept]` 事件流（`src/sbirs_sensor/pipeline/SbirsAcceptanceLog.h`）。
- `ONEQ_ENABLE_PRECISION_EVALUATION_LOG`（默认 OFF）：`[PrecisionEval]` 事件流（`src/precision_evaluation/PrecisionEvaluationLog.h`）。
- `ONEQ_ENABLE_RIR_ACCEPTANCE_LOG`（默认 OFF，2026-08-20 新增）：`[RirAccept]` 事件流，宏基础设施已就绪、**调用点尚未接线**（依本档 §5）。

总前提：`src/fusion`、`src/target_inference`、`src/threat_assessment` 三模块**零日志调用**、不接 trace/replay（`docs/target_inference/boundaries.md` 非目标），出口只有公开结构体 + 示例层 `CA_LOG_*` + 评估层日志。RIR 模块内 `PROJECT_LOG_*` 仅 4 条失败/异常 WARN（`runtime/RirController.cpp:357,401` 等），不记录量测类数值。

## 3. 总览

约 50 项按主要缺口归四类（混合档以"决定下一步动作"的缺口归类）：

| 类别 | 数量 | 含义 |
|---|---:|---|
| 已可输出（A 为主，含门控开关） | ≈19 | 公开结构体或既有验收日志已覆盖，验收时直接取数/开开关即可 |
| 部分满足（D，缺子项或语义限制） | ≈15 | 主体有输出，缺个别字段（航向、加速度向量、置信度、椭圆参数、等级评定等） |
| 仅内部计算（C，接入 `[RirAccept]` 即可输出） | ≈9 | 数值每周期都算，只差日志出口——新宏的第一批接线对象 |
| 未实现（E，需新建模型/产品） | ≈5 | 卫星定位误差统计、集群规模识别、关机点预测、初始化/加载计时 |

## 4. 分项明细

### 4.1 红外载荷（SBIRS，`src/sbirs_sensor`）

| 验收项 | 结论 | 现状与证据 |
|---|---|---|
| 最大探测距离 | A+B | 公开 `SbirsDetectionAttributionRecord.max_detection_range_m`（`include/1q/sbirs_sensor/session/SbirsOutputTypes.h:80-83`），WFOV 候选/首捕/跟踪三路径均填充并进 replay（`SbirsReplayFlatbufferCodec.cpp:483,551`）；日志 `wfov_candidate d_max_m`（`SbirsPipeline.cpp:1199`）。NFOV 门限版需按 `SbirsRadiometry.h:42-43` 公式自行换算 |
| 目标可探测性与动态参数 | A+B | 角度/SNR/检测标志公开（`SbirsOutputTypes.h:35-39`）；相对距离 `estimated_range_m`（:79）；真值/带误差角/角速度、探测器系目标角在验收日志（`SbirsPipeline.cpp:1050,1197-1201`）。按契约目标位置真值不进 raw 输出，仅验收日志留痕 |
| 卫星自身定位误差 | **E** | 卫星状态外部输入（`SbirsCycleInput.h:45-51`），无定位误差建模与统计输出；仅 `orbit_sigma_deg` 1-σ 配置参数（`SbirsPolicyConfig.h:34`）参与误差 RSS 合成且不单独输出 |
| 红外载荷测角误差 | B | 逐周期 `az_error_deg/el_error_deg`（`wfov_candidate`，`SbirsPipeline.cpp:1198`）、`output_az/el_error_deg`（`nfov_track`，:1052-1053）；统计汇总（mean/RMSE/P95/max）在 precision_evaluation 层（开关 `ONEQ_ENABLE_PRECISION_EVALUATION_LOG`）。模块自身无汇总 |
| 安装矩阵误差 | **C** | 运行期抽取值仅内部快照 `misalignment_yaw/pitch/roll_deg`（`SbirsPipeline.h:44-46`），消费于 boresight 链（`SbirsBoresightChain.cpp:42-65`）后不输出；replay 只记配置 bias/sigma/seed（`SbirsReplayFlatbufferCodec.cpp:229-232`） |
| 大幅面扫描与探测（幅宽/辐射/掠角） | B+D | 覆盖四角经纬度+驻留时间 `scan_footprint`（`SbirsPipeline.cpp:772-777`）；接收功率/信号能量 `wfov_candidate`/`nfov_track`（:1053-1054,1200）；幅宽是配置量（`SbirsMissionConfig.h:29-30`）；"对地掠角"未定义（只有 ECI/传感器系俯仰角，`SbirsCycleResult.h:25`） |
| 宽窄视场联合探测 | A+B | 宽场疑似列表=逐检测 `SbirsDetectionRecord`；窄场精确状态=`nfov_track` 全量字段；序列确认=`wfov_hit_gate`/`nfov_acquisition`（`SbirsPipeline.cpp:1388-1396,1509-1511`）+ 生命周期事件（`SbirsDetectionLifecycleRecorder.h:23-73`） |
| 宽视场扫描探测（覆盖多边形+驻留） | B | `scan_footprint`：`corners_latlon_deg`（4 角）、`center_latlon_deg`、`dwell_s`、`scan_rate_deg_s`（`SbirsPipeline.cpp:772-777`）。不进公开 API/遥测；开关 OFF 时无记录 |
| 窄视场跟踪探测（脱靶量/状态/能量/SNR） | A+B | SNR 公开 `infrared_snr_linear` + 日志 `snr=`（:1053）；脱靶量米+像素 `focal_offset_m/pix` 仅日志（:1051，原语 `SbirsGeometry.h:120-139` 注释明示"仅验收日志消费"）；跟踪状态公开门诊断字段（`SbirsOutputTypes.h:93-97`） |
| 协同工作机制 | A+B | `nfov_schedule`/`nfov_release` 事件（`SbirsPipeline.cpp:654-656,1569-1574`）+ `nfov_channel_id`/`capture_failure_reason` 公开字段（`SbirsOutputTypes.h:53-60`） |

### 4.2 融合 / 推演 / 威胁（`src/fusion`、`src/target_inference`、`src/threat_assessment`）

| 验收项 | 结论 | 现状与证据 |
|---|---|---|
| 多传感器目标跟踪（ECEF 位置+协方差） | A+D | `FusedKinematicEstimate{position(LLA), velocity_ecef, covariance_ecef[36]}`（`include/1q/fusion/FusedTarget.h:51-55`，填充 `FusionEngine.cpp:683-703`）；缺 ECEF 位置向量本体（公开面是 LLA）；受 `enable_track_filtering`（默认 false，`FusionConfig.h:31`）门控 |
| 多传感器接力跟踪 | A+D+E | 位置/速度/融合航迹=上述；加速度仅单传感器出口（AR `TrackStateSnapshot.h:46-50`）；**剩余覆盖时间、接力计划、交接指令全仓未实现**（grep `接力|handover|relay` 无多传感器协议命中） |
| 协同探测信息融合 | A | `FusedTarget.channels`（分源样本数/判决/质量/位置）+ `confidence`（`FusedTarget.h:22-32,62-63`，`FusionEngine.cpp:471-480`）；库内无日志，示例层有 `CA_LOG_EVENT`（`examples/.../fusion_component.cpp:92-111`） |
| UKF 滤波 | A+D | 开关 `enable_track_filtering` 开启后输出位置/速度/ECEF 6×6 协方差（同上）；对真值估计误差在 `[PrecisionEval]`（`velocity_error` 事件）；CV 6 维无加速度 |
| 集群目标识别（规模数量） | **E** | 无集群规模数量产品；旁证仅 ESR 相干计数用于假目标标注（`src/common/geometry/BearingCluster.h:110-127`） |
| 目标轨迹预报（轨迹/落点/误差椭圆） | A+D | `TrajectoryPrediction.waypoints`（默认 10 s 间隔/300 s 时域）+ `impact_point`（经纬高）+ `impact_position_sigma_m`（`include/1q/target_inference/InferenceResult.h:22-39`，计算 `TargetInferenceEngine.cpp:190-234,330-374`）；**误差椭圆参数无**（只有标量 1σ；发射点侧有 6×6 协方差可推导） |
| 落点预测（低动态目标初步落点） | A | 同上 `impact_point`；无"低动态"专门分支，统一弹道外推 |
| 发射点预测 | A+E(置信度) | `launch_point/launch_time_offset_sec/launch_position_sigma_m/launch_covariance_ecef[36]` 齐全公开（`InferenceResult.h:40-44`）；对真值误差在 `[PrecisionEval] keypoint_error`；**无置信度字段** |
| 关机点预测 | **E** | 无推力/燃烧段模型（`TargetInferenceEngine.cpp:34-53`），无关机时刻/位置/误差/置信度的任何计算（`docs/target_inference/boundaries.md` 非目标第 3 条，设计选择） |
| 特殊事件监测与提示 | A+D+E | 首次探测/跟踪中断事件：`SbirsDetectionLifecycleRecorder`（kFirstDetected/kCoasting/kLost 等，事件含 SNR/距离/NIS 诊断，`SbirsDetectionLifecycleRecorder.h:23-73`）+ 排除原因 A2/A3/A4（`SbirsExclusionCauseRecorder.h`）；AR/EOS 同构。缺：事件等级、空间位置字段、日志持久化（仅缓存最近周期）；**轨迹突变未实现** |
| 落点预报与信息发布 | A+D+E | 预测落点+误差范围=F6 证据；弹道模型仅输入配置不随产品记录；无置信度；**标准化封装与分发未实现**（库内无事件总线，`docs/fusion/boundaries.md` 非目标第 3 条） |

### 4.3 精度评估（`src/precision_evaluation`，`[PrecisionEval]` 开关）

| 验收项 | 结论 | 现状与证据 |
|---|---|---|
| 关键精度指标 | A+E(子项) | 已有：角误差（`AngularErrorSample`，`PrecisionEvaluationTypes.h:87-93`）、距离（三维）误差、`ErrorMetricSummary{count,mean,rmse,p95,max}`（:62-68）+ `metric_summary` 日志（`PrecisionEvaluationSession.cpp:564-577`）。**缺：三轴分轴位置误差、CEP、置信区间、各误差源贡献率**（`docs/precision_evaluation/boundaries.md:26` 明示非目标） |
| 层次分析法（AHP） | A+B+D | 单层 5 指标：权重向量+一致性（CR）、`metric_scores`、`metric_contributions`、`composite_score` 均公开（`PrecisionEvaluationTypes.h:70-84,131-140`）+ `ahp_score` 日志；**缺：等级评定（分档映射）、多层级层次树、贡献度排序输出** |

### 4.4 RIR 信号链（`src/remote_identification_radar` dwell/internal）

| 验收项 | 结论 | 现状与证据 |
|---|---|---|
| 天线方向图仿真 | A+D+C | 参数级公开：`RirAntennaConfig`（峰值增益/波束宽度/旁瓣/扫描损耗，`RirHardwareConfig.h:91-103`）+ 发射帧天线块（`RirEmissionFactory.cpp:75-81`→`RirCycleResult.h:49`）；**逐目标方向图评估值（离轴增益/主瓣衰减/扫描损耗）算完即弃**（`RirAntennaPatternRuntime.h:35-41`→`RirBeamControl.h:111-116`）；无三维方向图数据表 |
| 回波功率计算 | A(仅SNR)+**C** | 公开仅 `snr_db`（`RirFeatureMeasurementTypes.h:100`，进 replay）；回波/热噪/干扰功率在 `RirDetectionCellResult` 内部结构（`RirDetectionCellResolver.h:48-59`，计算 `RirDetectionCellResolver.cpp:229-249`），`RirController.cpp:416-419` 只取 snr/丢弃其余 |
| 干扰功率计算 | **C** | 逐源到达功率在公共类型 `RfIncidentLinkResult`（`include/1q/electromagnetics/RfScene.h:142-165`）但存于内部 `RirResolvedRfCycle`（`RirController.h:108`）不透出；聚合总干扰 `interference_power_w`（`RirDetectionCellResolver.cpp:80-178,249`）只进 SINR 分母 |
| 接收机噪声功率 | **C** | kTB·F 两路计算（`RirDetectionCellResolver.cpp:238-240`、`RirSignalDetector.cpp:15-27`），数值不输出 |
| 目标信号增益 | **C**+D | 脉压增益 `max(1,B·τ)`（:236-237）用后即弃；`target_processing_gain_db` 是配置（`RirHardwareConfig.h:169`）生效值不输出；相干积累 `ComputeIntegrationGain` 存在但运行链无调用（仅测试用，`tests/.../rir_radar_equations_test.cpp:80`）；无 MTI 概念 |
| 噪声增益 | C+**E** | 仅 dB 偏置 `noise_processing_gain_db`（配置，生效值不输出）；**无多普勒滤波器组/MTD 数据通路**，逐通道噪声功率概念不存在 |
| 杂波信号处理 | C+**E** | 杂波功率计算完整（`RirPropagationModel.cpp:44-53`→`RirDetectionCellResolver.cpp:252,263-267`）；`clutter_suppression_gain_db` 为配置偏置；MTI 剩余/MTD 逐通道分布/抑制比无模型无输出 |
| 干扰信号处理增益（检测/位置/Pd） | A(间接)+**C** | 位置经 `RirTrackAttributionRecord.position_enu_*` 公开；`detection_prob` 与逐目标 `detected` 布尔算完丢弃（`RirSignalDetector.h:46-48`→`RirController.cpp:416-419`）；`jamming_suppression_gain_db` 生效值不输出 |
| 波束扫描（波位表/轨迹序列） | A+D | 逐周期 `dwell_center_deg`（`RirCycleResult.h:69`，进 replay）；**完整波位排列表不输出**（`RirSession.cpp:142-163` 局部变量，可按公开配置确定性重建）；示例层逐周期打印（`examples/.../rir_sensor_component.cpp:202-206`） |
| 调度策略（事件执行数量） | A+D | `RirDwellBudgetSummary{scheduled/executed_dwell_count, dwell_budget/consumed_sec}`（`RirRecognitionResult.h:99-104`，计算 `RirController.cpp:539-551`）+ 识别维度计数（:110-114），均进 replay；缺按事件类型分类计数（scheduled 实为场景目标数，属事后统计） |

### 4.5 RIR 跟踪与识别（tracking/recognition）

| 验收项 | 结论 | 现状与证据 |
|---|---|---|
| 对指定空域搜索（目标清单） | A+D | 波位+检测计数公开（同上）；逐目标检测清单无产品（`Pd/detected` 内部）；近似替代=逐周期 `track_attributions` 全量快照（`RirController.cpp:635-654`） |
| 角度/距离测量（类型/定位/运动参数） | A+D+E | `RirFeatureMeasurementRecord`：range/az/el/高度/速度模/加速度模/大类/型号/置信度全有（`RirFeatureMeasurementTypes.h:93-107`）；**缺航向（速度向量仅内部 `RirTrackTypes.h:95`）与舰船/车辆类型**（类别枚举 8 类无此二类，`RirRecognitionResult.h:39-48`）；量测语义为真值经效能约束转换（角度无噪声，`RirFeatureMeasurementTypes.h:5-9`，RIR-OQ-1） |
| 典型/再入目标跟踪 | A+C+E | 公开=ENU 位置三分量+速度模+hit_count（`RirOutputTypes.h:83-92`）；速度/加速度向量与协方差仅内部（`RirTrackTypes.h:94-100`）；缺航向 |
| 多目标跟踪 | A+C | 多航迹逐周期快照天然覆盖（`RirCycleResult.track_attributions`）；运动全量参数同上限制 |
| 航迹关联 | **C** | 关联对/代价/漏检键全内部（`RirTrackAssociator.cpp:66-217`，结果结构 `RirTrackAssociator.h:61-65`）；公开面只有 `hit_count` 累计影子 |
| 跟踪滤波 | A(降维)+**C**+D | 位置+速度模公开；6 维滤波全量、协方差 P 阵、IMM 权重仅内部（`RirTrackLifecycle.cpp:347-358`）；无显式"下一时刻预测值"字段（失配周期预测间接覆盖航迹位置） |
| 按 RCS 实时探测+持续定位 | A+C+E | 定位参数大部分公开（同上）；Pd/detected 无输出；缺航向 |
| 运动特征处理→类别 | A | `RirMotionFeatureObservation` 公开（速度/高度/加速度/横向加速度/转弯半径/质量）+ 类别结论（`MotionFeatureExtractor.cpp:20-58`→`RirRecognitionResult`） |
| RCS 统计特征→类别 | A | `RirRcsFeatureObservation` 8 字段公开（均值/std/变化/峰谷/覆盖/质量，`RcsFeatureExtractor.cpp:64-119`）+ 结论 |
| 极化特征解算 | A | `RirPolarizationFeatureObservation` 4 字段 + 类别/置信度/分项得分公开；正确识别率=`RirRecognitionCycleSummary.category_accuracy/model_accuracy`（`RirRecognitionResult.h:121-123`，口径=已确认航迹 vs target_name 命中数据库，验收时需注明） |
| 宽带一维像特征解算 | A+D | 4 统计量公开（长度/峰数/能量集中率/分辨率，`RangeProfileFeatureExtractor.cpp:32-109`）；不透出逐散射中心/轮廓明细；无"识别精度"专属字段（最接近=全局 accuracy） |

### 4.6 性能测试（全仓）

| 验收项 | 结论 | 现状与证据 |
|---|---|---|
| 初始化时间（≤100ms） | **E** | 全仓无 `Session::Create`/建链计时（性能测试只计 Step，`tests/performance/cross_domain/rf_interference_performance_test.cpp:224-231`） |
| 单步执行时间（<20ms） | B+D | 测试内计时+断言：RF 干涉 P95<**100ms**（:249-269，阈值与本清单 20ms 不一致）、SAR 单步上限 30s（`sar_fft_performance_test.cpp:378-390`）；运行时仅 SAR 聚焦有 DEBUG 级分段计时（`SarRda.cpp:280-289`，默认级别 kInfo 不落盘） |
| 单个模型加载时间 | **E**(计时)/D(状态) | 识别库加载 `RecognitionFeatureDatabase.cpp` 无 chrono，仅失败有 ERROR 日志（`RirController.cpp:165-178`） |
| 多模型并行加载 | D | 仅失败状态输出（示例层 stderr，`demo_config.cpp:60-85`）；无并行加载实现、无耗时记录 |
| 连续运行次数与状态 | A+B | 示例摘要（周期数/产出计数，`component_attachment_demo.cpp:319-345`）+ 批量验证 cycles.csv/scenarios.csv（`sbirs_batch_validation.cpp:229-236,319-328`）+ 测试属性 `measured_cycle_count` |
| 典型场景与总仿真次数 | A | 批量验证框架完整：五模块 212 场景清单（`docs/practice/batch_validation.md:14-21`）+ scenarios.csv + 汇总脚本（`analyze_batch_results.py`） |
| 组件模型参数性能（含红外测角误差） | B+A | 测角偏差=`[SbirsAccept]` 逐周期字段（`SbirsPipeline.cpp:1198,1052-1053`，开关默认 OFF）+ `[PrecisionEval]` 汇总；表征测试有 RecordProperty（`sbirs_cue_ca_characterization_test.cpp:268-308`）；批量验证 cycles.csv **无测角误差列** |

## 5. `[RirAccept]` 接线记录（2026-08-20 已实施）

按"数值已算、只差出口"分批接线，全部调用点遵循 `if (RIR_ACCEPTANCE_LOG_ENABLED())` 包裹的零开销模式；实施后 ON 态 `unit::remote_identification_radar` + `unit::sbirs_sensor` 测试全绿，5191 条 `[RirAccept]` 事件实测落盘（含 6×6 协方差 36 元素完整输出）：

1. **detection_cell（`RirController.cpp` TryBuildMeasurement）**：逐目标方向图离轴增益/有效波束宽度/离轴角/峰值增益、回波（W 与 dBW）/热噪/干扰/杂波功率、脉压增益、SINR/SNR、Pd、detected、admitted；`has_cell=0` 标记 v1 效能级回退路径。
2. **interference_link（RunCycle，逐周期逐干扰源）**：发射身份三元组、路径长度、时/频重叠率、到达功率（前端口径；聚合后入 SINR 的总量在 detection_cell.interference_w）。
3. **association / association_match / association_missed（RunCycle）**：量测数/命中对数/漏检数摘要 + 逐命中对（键/量测索引/马氏代价）+ 逐漏检键。
4. **track（输出循环）**：航迹全量状态——status/hits/misses、位置/速度/加速度三分量、速度模、RCS、协方差迹 + 6×6 协方差全矩阵（行主序 [x,vx,y,vy,z,vz]）。IMM 权重不在快照层（裁定不新增，注释在 `RirImmFilter.h`）；无显式下一时刻预测值（注释在调用点）。
5. **measurement（出口①记录循环）**：四维特征量测全量字段（RCS 7 量/运动 6 量/极化 4 量/距离像 5 量）+ 视线角/距离/SNR/驻留/带宽/掩码。
6. **recognition（输出循环）**：state/category/model/confidence/best/runner_up/掩码/积累量/库版本。
7. **schedule（RunCycle 尾部）**：scheduled/executed 驻留计数、预算/耗时、识别效能摘要（参与/大类确认/型号确认/未知/未启用/平均置信度/真值正确率）。
8. **beam_pattern / beam_pattern_wave（RirSession）**：完整波位排列表一次性输出（mission 配置变更后重发；common 扫描内核确定性重建）。
9. **beam_scan（RirSession 逐周期）**：驻留波束中心 az/el 与来源（designate/scan）。

SBIRS 侧同步接线：**misalignment** 事件（`SbirsPipeline.cpp` 构造与 ApplyConfig）——安装失准角 yaw/pitch/roll 运行期一次抽取值（S5 项，`[SbirsAccept]` 通道）。

D 项缺失部分按裁定不输出、只写注释，落点：`RirFeatureMeasurementTypes.h`（航向）、`RirRecognitionResult.h`（舰船/车辆类型）、`RirAntennaPatternRuntime.h`（三维方向图表）、`RirImmFilter.h`（IMM 权重）、`RirController.cpp`（预测值，随 track 事件注释）、`FusedTarget.h`（ECEF 位置向量/加速度）、`InferenceResult.h`（误差椭圆）、`PrecisionEvaluationTypes.h`（等级评定/排序）、`SbirsDetectionLifecycleRecorder.h`（事件等级/位置）。

**已知注意事项**：`1q_library.log` 文件后端仅进程内互斥；多个测试可执行文件并行（ctest -j4、同工作目录）写同一日志文件时可能出现跨进程行交错（实测 5191 条中 1 条截断）。单进程运行（验收的正常形态：示例/批验证逐模块执行）不受影响。

## 6. 待确认清单（2026-08-20 已裁定）

1. "扫描掠角"新定义（对地掠射角）——**裁定：不需要**，沿用 ECI/传感器系俯仰角。
2. 低动态目标落点专门算法分支——**裁定：不需要**，通用弹道外推即验收口径。
3. 单步执行时间阈值——**裁定：以现有为准**（RF 干涉 P95<100ms、SAR 单步上限 30s）。
4. 整项未实现的 5 项（卫星定位误差统计、集群规模识别、关机点预测、初始化/加载计时、多模型并行加载）——**裁定：不计入验收**。
5. RIR 舰船/车辆类型与航向字段——**裁定：不增加**（注释说明落点见 §5）。
6. 批量验证 cycles.csv 测角误差列——**裁定：不增加**。

**实施总则（用户裁定）**：A+B+C+D 项输出已有的部分（本档 §5 已接线）；D 项缺失子项不输出、在对应类型定义处写注释说明；E 项不管。

[evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp]
[evidence: src/sbirs_sensor/pipeline/SbirsAcceptanceLog.h]
[evidence: src/remote_identification_radar/runtime/RirAcceptanceLog.h]
[evidence: src/remote_identification_radar/runtime/RirController.cpp]
[evidence: src/remote_identification_radar/dwell/RirDetectionCellResolver.cpp]
[evidence: include/1q/fusion/FusedTarget.h]
[evidence: include/1q/target_inference/InferenceResult.h]
[evidence: include/1q/precision_evaluation/PrecisionEvaluationTypes.h]
[evidence: include/1q/remote_identification_radar/session/RirOutputTypes.h]
[evidence: cmake/project/ProjectOptions.cmake]
