/**
 * @file SbirsInputValidation.h
 * @brief 定义 SBIRS-inspired 输入校验类型与入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"

namespace sbirs_sensor {
namespace session {

using ValidationSeverity = oneq::foundation::ValidationSeverity;
using ValidationLocation = oneq::foundation::ValidationLocation;
using ValidationLocationKind = oneq::foundation::ValidationLocationKind;

/** @brief 单条输入校验问题，包含严重级别、位置和人读消息。 */
struct ONEQ_API ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo}; /**< 问题严重级别 */
  ValidationLocation location{};                          /**< 问题位置（全局/平台/场景实体） */
  std::string message{};                                  /**< 人读问题描述 */
};

/** @brief 校验问题列表。 */
using ValidationIssueList = std::vector<ValidationIssue>;

struct SbirsCycleInput;

/**
 * @brief 校验单周期输入：步长、非零 ECEF、目标 ID 唯一性、物理取值域、速度 flag/data
 *        一致性及 environment override 的枚举与连续参数。
 * @param[in] input 待校验的单周期输入
 * @return 校验问题列表；为空表示输入通过校验
 * @note 该函数为纯校验，不修改输入，不抛异常。
 */
ONEQ_API ValidationIssueList ValidateSbirsCycleInput(const SbirsCycleInput& input);
/**
 * @brief 判断校验问题列表中是否存在错误级别问题。
 * @param[in] issues 校验问题列表
 * @return 存在 `kError` 级别问题返回 true，否则返回 false
 */
ONEQ_API bool HasValidationError(const ValidationIssueList& issues);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_
