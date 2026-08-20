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
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

/**
 * @brief 校验单周期 SAR 输入。
 *
 * 覆盖范围：周期步长合法性、平台/目标数值有限性、外部脉冲状态（若存在）
 * 的数值有限性与脉冲序号连续性。本函数不修改输入，也不阻断会话执行；
 * 运行期阻断仍由 `SarSession::StepWithResult` 内部完成。
 *
 * 返回统一问题列表（规则 14）：条目 phase 固定为 `kInputValidation`，
 * code 编码为 `"sar.validation.<snake_case>"`。
 *
 * @param[in] input 单周期输入。
 * @return 校验问题列表。
 */
ONEQ_API SarIssueList ValidateSarCycleInput(const SarCycleInput& input);

/**
 * @brief 判断校验问题列表中是否存在 error 级问题。
 * @param[in] issues 校验问题列表。
 * @return 若存在 `phase == kInputValidation && severity == kError` 的问题则返回 `true`。
 */
ONEQ_API bool HasValidationError(const SarIssueList& issues);

/**
 * @brief 检查 SAR 硬件与任务字段的基本合法性（正值、有限值、采样窗口容脉冲）。
 *
 * 供 `ValidateSarSessionConfig` 和 `ValidateRuntimeConfigForStep` 共享，避免重复校验逻辑。
 * 不检查策略域（policy）或环境域（environment）字段。
 *
 * @param[in] config 待检查的会话配置。
 * @return 所有硬件与任务字段合法时返回 `true`。
 */
ONEQ_API bool AreSarHardwareAndMissionFieldsValid(const config::SarSessionConfig& config);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_INPUT_VALIDATION_H_
