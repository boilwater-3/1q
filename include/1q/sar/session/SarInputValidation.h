/**
 * @file SarInputValidation.h
 * @brief 定义 SAR 周期输入校验接口。
 */

#ifndef ONEQ_SAR_SESSION_SAR_INPUT_VALIDATION_H_
#define ONEQ_SAR_SESSION_SAR_INPUT_VALIDATION_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"
#include "1q/sar/session/SarCycleInput.h"

namespace sar {
namespace session {

using oneq::foundation::ValidationSeverity;
using oneq::foundation::ValidationLocation;
using oneq::foundation::ValidationLocationKind;

/**
 * @brief ValidationCode 表示结构化校验编码。
 */
enum class ONEQ_API ValidationCode {
  kNone = 0,                      /**< 无问题占位值 */
  kInvalidCycleDeltaTime,         /**< 周期步长非法（<= 0） */
  kNonFiniteCycleDeltaTime,       /**< 周期步长非有限值 */
  kNonFinitePlatformField,        /**< 平台存在非有限数值字段 */
  kNonFiniteTargetField,          /**< 点目标存在非有限数值字段 */
  kInvalidPulseSequence,          /**< 脉冲序号不连续或时间非单调 */
  kNonFinitePulseField,           /**< 脉冲状态存在非有限数值字段 */
  kPulseCountMismatch             /**< pulse_count 与 pulse_states 数量不一致 */
};

/**
 * @brief ValidationIssue 描述单条输入校验结果。
 */
struct ONEQ_API ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo}; /**< 问题严重级别 */
  ValidationCode code{ValidationCode::kNone};             /**< 结构化编码 */
  ValidationLocation location{};                          /**< 结构化定位信息 */
  std::string field{};   /**< 触发问题的字段名；为空表示跨字段或域级问题 */
  std::string message{}; /**< 面向调用方的简短说明 */
};

/** @brief ValidationIssueList 表示输入校验问题列表。 */
using ValidationIssueList = std::vector<ValidationIssue>;

/**
 * @brief 校验单周期 SAR 输入。
 *
 * 覆盖范围：周期步长合法性、平台/目标数值有限性、外部脉冲状态（若存在）
 * 的数值有限性与脉冲序号连续性。本函数不修改输入，也不阻断会话执行；
 * 运行期阻断仍由 `SarSession::StepWithResult` 内部完成。
 *
 * @param[in] input 单周期输入。
 * @return 校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateSarCycleInput(const SarCycleInput& input);

/**
 * @brief 判断校验列表中是否存在 error 级问题。
 * @param[in] issues 校验问题列表。
 * @return 若存在 error 级问题则返回 `true`。
 */
ONEQ_API bool HasValidationError(const ValidationIssueList& issues);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_INPUT_VALIDATION_H_
