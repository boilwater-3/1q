/**
 * @file RirIssueCodes.h
 * @brief 远程识别雷达 issue code 注册表：本模块全部 code 常量的单一事实来源
 *       （session_contract.md 规则 14c）。
 * @note 仅登记真实产生的 issue code；不产生为 issue code 的兜底串不在此登记。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_ISSUE_CODES_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_ISSUE_CODES_H_

namespace remote_identification_radar {
namespace session {
namespace codes {

// ===== 周期输入校验问题（"rir.validation.<snake_case>"）=====

/** @brief 周期步长非有限值。 */
constexpr char kNonFiniteCycleDeltaTime[] = "rir.validation.non_finite_cycle_delta_time";

/** @brief 周期步长非法（<= 0）。 */
constexpr char kInvalidCycleDeltaTime[] = "rir.validation.invalid_cycle_delta_time";

/** @brief 周期序号非法（为 0）。 */
constexpr char kInvalidCycleIndex[] = "rir.validation.invalid_cycle_index";

/** @brief 目标含非有限数值字段（位置/RCS/斜距/真值样本）。 */
constexpr char kNonFiniteTargetField[] = "rir.validation.non_finite_target_field";

/** @brief 目标斜距 <= 0 且无笛卡尔位置（二者须至少一为正）。 */
constexpr char kMissingRangeAndCartesianPosition[] =
    "rir.validation.missing_range_and_cartesian_position";

/** @brief 场景中外部目标 ID 重复。 */
constexpr char kDuplicateExternalTargetId[] = "rir.validation.duplicate_external_target_id";

/** @brief 目标速度/起伏模型含非有限或非法字段。 */
constexpr char kInvalidTargetMotionField[] = "rir.validation.invalid_target_motion_field";

/** @brief 环境快照字段非法（天气衰减非有限/负值）。 */
constexpr char kInvalidEnvironmentSnapshot[] = "rir.validation.invalid_environment_snapshot";

/** @brief 自身发射身份非法（任一 ID 为 0）。 */
constexpr char kInvalidOwnEmissionIdentity[] = "rir.validation.invalid_own_emission_identity";

/** @brief RF 入射链路含非法字段。 */
constexpr char kInvalidRfIncidentLink[] = "rir.validation.invalid_rf_incident_link";

// ===== 配置校验问题（"rir.validation.<snake_case>"）=====

/** @brief 识别特征权重非法（须有限、在 [0, 1] 且总和为 1）。 */
constexpr char kRecognitionWeightsInvalid[] = "rir.validation.recognition_weights_invalid";

/** @brief 识别数据库路径缺失（启用识别时须非空）。 */
constexpr char kRecognitionDatabasePathMissing[] =
    "rir.validation.recognition_database_path_missing";

/** @brief 识别门限非法（接受分数/最小裕度须在 [0, 1]）。 */
constexpr char kRecognitionThresholdInvalid[] = "rir.validation.recognition_threshold_invalid";

/** @brief 识别累积计数非法（须至少为 1）。 */
constexpr char kRecognitionAccumulationInvalid[] =
    "rir.validation.recognition_accumulation_invalid";

/** @brief 识别时间范围非法（保持时间须非负；最大距离/驻留/累积窗口须有限且为正）。 */
constexpr char kRecognitionTimeRangeInvalid[] = "rir.validation.recognition_time_range_invalid";

/** @brief 扫描策略非法（限位须有限有序且在合法域 az∈[-180,180]、el∈[-90,90]；步长系数须为正）。 */
constexpr char kScanStrategyInvalid[] = "rir.validation.scan_strategy_invalid";

/** @brief 信号处理增益偏置非法（四偏置须有限且在 [0, 40] dB；阶段 2-M M3）。 */
constexpr char kSignalProcessingGainsInvalid[] = "rir.validation.signal_processing_gains_invalid";

/** @brief 检测策略非法（Pfa/门限/脉冲数/种子）。 */
constexpr char kDetectionPolicyInvalid[] = "rir.validation.detection_policy_invalid";

/** @brief 关联策略非法（波门 sigma 非正）。 */
constexpr char kAssociationPolicyInvalid[] = "rir.validation.association_policy_invalid";

/** @brief 跟踪策略非法（KF 噪声参数非正）。 */
constexpr char kTrackingPolicyInvalid[] = "rir.validation.tracking_policy_invalid";

/** @brief 生命周期策略非法（confirm/lost 阈值）。 */
constexpr char kLifecyclePolicyInvalid[] = "rir.validation.lifecycle_policy_invalid";

}  // namespace codes
}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_ISSUE_CODES_H_
