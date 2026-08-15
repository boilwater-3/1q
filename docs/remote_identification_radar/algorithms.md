---
Status: active
Last-reviewed: 2026-08-15
Authority: RIR 算法登记与实现边界
Answers: RIR 每个算法做什么、实现边界在哪、哪些反直觉、哪些刻意不做
---

# Remote Identification Radar 算法

算法为 AR 远程识别子系统（kLrr，审计基线 96de367c）的平移改写（命名/类型换
`Rir*`，行为一致；等价性由 `integration::cross_domain` 的
`ar_rir_recognition_equivalence_test` 逐字段锁定）。

## 算法登记表

| 算法 | 位置 | 输入 → 输出 | 边界与反直觉点 |
|---|---|---|---|
| 观测构造 | `recognition/RecognitionObservationBuilder.cpp` | 场景目标真值 + 航迹供给 + `RirObservationContext` → `RirFeatureSet` | 驻留质量因子作用于 RCS/极化/距离像（运动除外）；四提取器只消费效能化观测，场景真值不得直接产生结论 |
| RCS 特征 | `recognition/RcsFeatureExtractor.cpp` | 视角样本 + 视线角 + SNR → `RirRcsObservation` | **最近邻插值不强制覆盖**（视线角超出样本网格仍取最近样本）；SNR < 6 dB 维度无效；覆盖下限由匹配阶段按 profile 判定 |
| 运动特征 | `recognition/MotionFeatureExtractor.cpp` | `RirTrackFeedEntry` + 平台海拔 + 不确定度 → `RirMotionObservation` | 仅已确认航迹；横向加速度分解判直线/转弯半径；质量因子 = 10000/(10000+不确定度) |
| 极化特征（F1） | `recognition/PolarizationFeatureExtractor.cpp` | 双通道样本 + 视线角 + SNR/距离 → `RirPolarizationObservation` | 通道定义由数据库固定；强干扰（SNR 压低）维度不可用 |
| 距离像特征（F2） | `recognition/RangeProfileFeatureExtractor.cpp` | 散射中心列表 + 带宽 + SNR → `RirRangeProfileObservation` | 分辨率 c/(2B) 超 `max_range_resolution_m` 维度无效；粗单元不合并峰标识 |
| 数据库加载 | `recognition/RecognitionFeatureDatabase.cpp` | SQLite 文件 → 全量内存模板 | schema v1.1 自描述校验；运行期无连接；units `rcs != 'dBsm'` 拒绝 |
| 匹配 | `recognition/RecognitionMatcher.cpp` | 特征集 + profile 适用条件 → 候选排序/大类分数 | `s = exp(-0.5·z²)`，`z = |x − mean| / std`；质量 0 维度不进分子分母；型号得分 = 最佳适用 profile 加权相似度 × 先验；类别得分 = 成员型号未归一化之和；`confidence = best / Σ` |
| 积累判定 | `recognition/RecognitionTracker.cpp` | 逐周期观测 → 结论状态机 | 分数 ≥ `acceptance_score` 且 `best − runner_up ≥ minimum_margin` 且有效维度 ≥ 2（**运动不能单独确认型号**）→ `kModelConfirmed`；否则大类或 `kUnknown` |

## 反直觉点

1. **第一个 profile 报告**：`feature_scores` 分项报告用型号的第一个 profile
   （`profiles.front()`），而非实际命中得分的 profile——多 profile 型号的分项报告
   可能与判定所用 profile 不一致（判定路径本身正确）。
2. **单候选 margin 恒过**：单候选时 `runner_up_score == 0`，margin 检查恒过——
   单候选场景天然满足型号确认门限。
3. **匀速场景**：加速度观测 ≈ 0，运动相似度 ≈（速度+高度）两子特征撑起；模板保留
   真实机动量级，对机动目标仍有效；场景验证以 rcs+速度+高度证据为主。
4. **RCS 最近邻**：观测构造阶段的视角覆盖下限被控制器置 0（`minimum_aspect_coverage_deg
   = 0`），视角覆盖判定完全由数据库 profile 的 `minimum_aspect_coverage_deg` 承担——
   样本网格仅需非空即产生 RCS 观测。
5. **键重分配检测**：`hit_count` 回落视为新目标并清空该键全部状态（供给方语义，
   与 AR 航迹生命周期一致）。

## 非目标（刻意不实现的算法）

1. 在线残差驱动的自动后端切换/在线学习/自适应权重。
2. 信号级 IQ / 全波散射求解、ISAR 二维成像、微动特征。
3. 多 association 生产路径（单键关联；本模块不做关联）。
4. 波束驻留指向调度（阶段 1 不实现，见 boundaries.md 评估项）。

## 证据

- 提取器与门控：`tests/unit/remote_identification_radar/rir_recognition_feature_test.cpp`
- 匹配与库契约：`tests/unit/remote_identification_radar/rir_recognition_database_test.cpp`
- 积累/判定/保持/回滚：`tests/unit/remote_identification_radar/rir_recognition_integration_test.cpp`
- 场景/型号效能：`tests/integration/remote_identification_radar/`
- 数值等价性：`tests/integration/cross_domain/ar_rir_recognition_equivalence_test.cpp`
