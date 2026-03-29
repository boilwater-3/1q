/**
 * @file RadarInputValidation.h
 * @brief 定义雷达周期输入的显式校验接口。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace core {
namespace context {

/**
 * @brief ValidationSeverity 表示校验结果严重级别。
 */
enum class ValidationSeverity {
  kInfo = 0, /**< 仅提示语义，不阻断执行 */
  kWarning,  /**< 调用方应显式关注的潜在问题 */
  kError     /**< 明确的错误输入，建议阻断执行 */
};

/**
 * @brief ValidationCode 表示结构化校验问题类型。
 */
enum class ValidationCode {
  kNone = 0,                         /**< 无问题占位值 */
  kInvalidCycleDeltaTime,            /**< 周期步长非法（<= 0） */
  kNonFiniteCycleDeltaTime,          /**< 周期步长不是有限值 */
  kNonFiniteTargetField,             /**< 目标字段存在非有限值 */
  kMissingRangeAndCartesianPosition, /**< 目标既没有有效斜距，也没有有效笛卡尔位置 */
  kUnknownExternalTargetId,          /**< 目标外部标识符未知 */
  kDuplicateExternalTargetId,        /**< 外部标识符重复 */
  kNegativeRcs                       /**< 目标 RCS 为负值 */
};

/**
 * @brief ValidationIssue 描述一条结构化输入校验结果。
 */
struct ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo}; /**< 严重级别 */
  ValidationCode code{ValidationCode::kNone};             /**< 问题类型编码 */
  std::size_t target_index{
      static_cast<std::size_t>(-1)}; /**< 目标索引；若与具体目标无关，则为 `size_t(-1)` */
  std::string message{};             /**< 面向外部调用方的简短说明 */
};

/**
 * @brief 校验完整周期输入。
 * @param[in] input 当前周期输入。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API std::vector<ValidationIssue> ValidateRadarCycleInput(const RadarCycleInput& input);

/**
 * @brief 校验目标特征列表。
 * @param[in] targets 当前周期目标列表。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API std::vector<ValidationIssue> ValidateTargetFeatures(
    const common::TargetFeatureList& targets);

/**
 * @brief 判断是否包含 error 级别问题。
 * @param[in] issues 校验问题列表。
 * @return 至少存在一个 `kError` 时返回 true。
 */
ONEQ_API bool HasValidationError(const std::vector<ValidationIssue>& issues);

}  // namespace context
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_
