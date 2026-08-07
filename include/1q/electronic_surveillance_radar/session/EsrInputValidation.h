/**
 * @file EsrInputValidation.h
 * @brief 定义电子侦察周期输入校验接口。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_INPUT_VALIDATION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_INPUT_VALIDATION_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief 校验单周期电子侦察输入。
 * @param[in] input 单周期输入。
 * @return 问题条目列表（统一问题列表模型，规则 14；所有条目 phase=kInputValidation，
 *         code 形如 "esr.validation.<snake_case>"）。
 */
ONEQ_API EsrIssueList ValidateEsrCycleInput(const EsrCycleInput& input);

/**
 * @brief 判断问题列表中是否存在输入校验 error 级问题。
 * @param[in] issues 问题条目列表。
 * @return 存在 phase==kInputValidation 且 severity==kError 的条目时返回 `true`。
 */
ONEQ_API bool HasValidationError(const EsrIssueList& issues);

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_INPUT_VALIDATION_H_
