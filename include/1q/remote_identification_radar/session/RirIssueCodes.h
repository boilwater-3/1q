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
constexpr char kRecognitionTimeRangeInvalid[] =
    "rir.validation.recognition_time_range_invalid";

}  // namespace codes
}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_ISSUE_CODES_H_
