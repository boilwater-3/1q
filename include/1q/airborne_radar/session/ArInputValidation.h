/**
 * @file ArInputValidation.h
 * @brief AR module primary validation entry points for cycle inputs.
 *
 * Primary header for input validation.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"

namespace airborne_radar {
namespace session {

using oneq::foundation::ValidationSeverity;
using oneq::foundation::ValidationLocation;
using oneq::foundation::ValidationLocationKind;

/**
 * @brief ValidationCode 表示结构化校验问题类型。
 */
enum class ONEQ_API ValidationCode {
  kNone = 0,                         /**< 无问题占位值 */
  kInvalidCycleDeltaTime,            /**< 周期步长非法（<= 0） */
  kNonFiniteCycleDeltaTime,          /**< 周期步长不是有限值 */
  kNonFinitePlatformNumericField,    /**< 平台位姿字段存在非有限值 */
  kNonFiniteTargetField,             /**< 目标字段存在非有限值 */
  kMissingRangeAndCartesianPosition, /**< 目标既没有有效斜距，也没有有效笛卡尔位置 */
  kUnknownExternalTargetId,          /**< 目标外部标识符未知 */
  kDuplicateExternalTargetId,        /**< 外部标识符重复 */
  kNegativeRcs,                      /**< 目标 RCS 为负值 */
  kInvalidEnvironmentObservation,    /**< 环境观测字段非法 */
  kEnvironmentSnapshotFlagMismatch   /**< 环境数据非默认但 has_environment=false */
};

/**
 * @brief ValidationIssue 描述一条结构化输入校验结果。
 */
struct ONEQ_API ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo}; /**< 严重级别 */
  ValidationCode code{ValidationCode::kNone};             /**< 问题类型编码 */
  ValidationLocation location{};                          /**< 结构化定位信息 */
  std::string field{};   /**< 触发问题的字段名；为空表示跨字段或域级问题 */
  std::string message{}; /**< 面向外部调用方的简短说明 */
};

/** @brief ValidationIssueList 表示输入校验问题列表。 */
using ValidationIssueList = std::vector<ValidationIssue>;

/**
 * @brief 校验周期步长字段。
 * @param[in] dt_sec 当前周期步长（单位：秒）。
 * @return 校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateArCycleDeltaTime(float dt_sec);

/**
 * @brief 校验完整周期输入。
 * @param[in] input 当前周期输入。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateArCycleInput(const ArCycleInput& input);

/**
 * @brief 校验场景目标列表。
 * @param[in] targets 当前周期场景目标列表。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateArSceneTargets(const ArSceneTargetList& targets);

/**
 * @brief 判断是否包含 error 级别问题。
 * @param[in] issues 校验问题列表。
 * @return 至少存在一个 `kError` 时返回 true。
 */
ONEQ_API bool HasValidationError(const ValidationIssueList& issues);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_
