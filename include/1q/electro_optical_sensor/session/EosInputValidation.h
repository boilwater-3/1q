/**
 * @file EosInputValidation.h
 * @brief 定义光学传感器周期输入校验接口。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_INPUT_VALIDATION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_INPUT_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief 校验单周期光学传感器输入。
 * @param[in] input 单周期输入。
 * @param[in] frame_rate_hz 传感器帧率（Hz），用于 dt_sec 上界校验；必须正有限。
 * @return 问题条目列表（统一问题列表模型，规则 14；所有条目 phase=kInputValidation，
 *         code 形如 "eos.validation.<snake_case>"）。
 */
ONEQ_API EosIssueList ValidateEosCycleInput(
    const ::electro_optical_sensor::session::EosCycleInput& input, float frame_rate_hz);

/**
 * @brief 判断问题列表中是否存在输入校验 error 级问题。
 * @param[in] issues 问题条目列表。
 * @return 存在 phase==kInputValidation 且 severity==kError 的条目时返回 `true`。
 */
ONEQ_API bool HasValidationError(const EosIssueList& issues);

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_INPUT_VALIDATION_H_
