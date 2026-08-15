---
Status: active
Last-reviewed: 2026-08-15
Authority: RIR 算法登记与实现边界
Answers: RIR 每个算法做什么、实现边界在哪、哪些反直觉、哪些刻意不做
---

# Remote Identification Radar 算法

阶段 1 算法为 AR 远程识别子系统（kLrr）的平移改写；阶段 2-S 起识别积累改挂
RIR 自持链路生产的内部航迹，不再消费外部航迹供给。

## 算法登记表

| 算法 | 位置 | 输入 → 输出 | 边界与反直觉点 |
|---|---|---|---|
| 检测单元求解 | `dwell/RirDetectionCellResolver.cpp` | 目标回波事实 + RF 入射链路 + 增益偏置 → 分项 SINR 账本 | 干扰按目标单元时频重叠聚合；四增益偏置缺省 0 dB 等于保守账本；自身发射身份不计干扰 |
| 统计级 CFAR | `dwell/RirSignalDetector.cpp` | SNR + Swerling + Pfa → Pd → 蒙特卡洛判决 | 不是 CA-CFAR；`min_snr_db` 硬截断、`min_detection_margin_db` 可靠性门；同种子同判决 |
| 量测误差 | `dwell/RirMeasurementErrorModel.h` | SNR + 波束宽度 + 带宽 → 距离/角度标准差 | 距离偏置 20 m；角度两轴 RMS 合成；只供内部关联/滤波 |
| 门限最近邻关联 | `tracking/RirTrackAssociator.cpp` | 检测量测 + 航迹种子 → 关联键/命中/新键 | 马氏平方波门（缺省 9）；全局最小代价边唯一分配；键单调不回收复用 |
| 单目标 KF | `tracking/RirTrackFilter.cpp` | 量测 + 先验状态 → 预测/更新后验 | 6 维 CV 状态；动态 R 更新；LLT 失败跳过更新；IMM 不迁 |
| 航迹生命周期 | `tracking/RirTrackLifecycle.cpp` | 关联量测 + 周期上下文 → 内部航迹 | hit/miss 计数、confirm/lost/回收；lost 重捕获重置 KF；回收不回收关联键 |
| 驻留排序 | `runtime/RirController.cpp` | 上一周期内部航迹结论 + 场景目标 → 驻留候选顺序 | 未识别优先 + 斜距次近；威胁等级输入不参与 |
| 观测构造 | `recognition/RecognitionObservationBuilder.cpp` | 场景目标真值 + 内部航迹 + `RirObservationContext` → `RirFeatureSet` | 驻留质量因子作用于 RCS/极化/距离像（运动除外）；场景真值不得直接产生结论 |
| RCS 特征 | `recognition/RcsFeatureExtractor.cpp` | 视角样本 + 视线角 + SNR → `RirRcsObservation` | **最近邻插值不强制覆盖**；SNR < 6 dB 维度无效；覆盖下限由匹配阶段判定 |
| 运动特征 | `recognition/MotionFeatureExtractor.cpp` | 内部航迹 + 平台海拔 + 不确定度 → `RirMotionObservation` | 仅已确认航迹；横向加速度分解判直线/转弯半径；质量因子 = 10000/(10000+不确定度) |
| 极化特征（F1） | `recognition/PolarizationFeatureExtractor.cpp` | 双通道样本 + 视线角 + SNR/距离 → `RirPolarizationObservation` | 通道定义由数据库固定；强干扰（SNR 压低）维度不可用 |
| 距离像特征（F2） | `recognition/RangeProfileFeatureExtractor.cpp` | 散射中心列表 + 带宽 + SNR → `RirRangeProfileObservation` | 分辨率 c/(2B) 超上限维度无效；粗单元不合并峰标识 |
| 数据库加载 | `recognition/RecognitionFeatureDatabase.cpp` | SQLite 文件 → 全量内存模板 | schema v1.1 自描述校验；运行期无连接；units `rcs != 'dBsm'` 拒绝 |
| 匹配 | `recognition/RecognitionMatcher.cpp` | 特征集 + profile 适用条件 → 候选排序/大类分数 | `s = exp(-0.5·z²)`；质量 0 维度不进分子分母；类别得分 = 成员型号未归一化之和 |
| 积累判定 | `recognition/RecognitionTracker.cpp` | 逐周期观测 → 结论状态机 | 分数 ≥ `acceptance_score` 且 margin 足且有效维度 ≥ 2 → `kModelConfirmed`；运动不能单独确认型号 |

## 反直觉点

1. **6 dB 回退模式**：`RirDetectionGateMode::kSnrFallback` 以 SNR ≥ 6 dB 替代
   CFAR 随机判决，且量测位置取真值（只携带误差协方差）；`kDetectorGate` 才
   执行蒙特卡洛判决与量测位置采样。
2. **KF 加速度口径**：hit 时加速度 = KF 后验速度与本周期场景速度种子之差/dt；
   miss 时 CV 外推速度不变、加速度归零。该口径与 AR 轻量跟踪子集一致。
3. **无环境输入退化**：环境快照 `has_environment_data=false` 且无入射链路时，
   传播损耗/杂波/干扰均为 0，检测 SNR 回到阶段 1 旧公式口径；有环境事实时走
   detection cell 分项账本。
4. **第一个 profile 报告**：`feature_scores` 分项报告用型号的第一个 profile
   （`profiles.front()`），而非实际命中得分的 profile——多 profile 型号的分项
   报告可能与判定所用 profile 不一致（判定路径本身正确）。
5. **单候选 margin 恒过**：单候选时 `runner_up_score == 0`，margin 检查恒过。
6. **RCS 最近邻**：视角覆盖判定完全由数据库 profile 的
   `minimum_aspect_coverage_deg` 承担——样本网格仅需非空即产生 RCS 观测。
7. **关联键不回收**：航迹回收只删除内部航迹，`next_key` 继续单调递增；因此
   识别积累不需要检测 `hit_count` 回落，新键天然等于新目标。

## 非目标（刻意不实现的算法）

1. 在线残差驱动的自动后端切换/在线学习/自适应权重。
2. 信号级 IQ / 全波散射求解、ISAR 二维成像、微动特征。
3. CA-CFAR（参考单元滑窗/杂波图/OS-GO-SO）、IMM、LAPJV 全局关联、航迹池。
4. 对外点迹/量测输出、战斗级关联决策与战术决策。
5. 外部雷达波束控制接口；驻留指向不驱动任何 AR 波束。

## 证据

- 检测/误差/关联/滤波/生命周期：`tests/unit/remote_identification_radar/rir_signal_detector_test.cpp`、
  `rir_measurement_error_test.cpp`、`rir_track_associator_test.cpp`、
  `rir_track_filter_test.cpp`、`rir_track_lifecycle_test.cpp`
- 自持链路与输入面：`tests/unit/remote_identification_radar/rir_self_contained_pipeline_test.cpp`、
  `rir_self_contained_validation_test.cpp`
- 提取器与门控：`tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp`
- 匹配与库契约：`tests/unit/remote_identification_radar/rir_recognition_database_test.cpp`
- 场景/型号效能：`tests/integration/remote_identification_radar/`
- replay V2：`tests/replay/remote_identification_radar/rir_replay_codec_roundtrip_test.cpp`
