/**
 * @file ArInputValidation.h
 * @brief 机载雷达周期输入校验入口。
 *
 * 周期输入静态校验的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief 校验周期步长字段。
 * @param[in] dt_sec 当前周期步长（单位：秒）。
 * @return 问题条目列表（统一问题列表模型，规则 14；所有条目 phase=kInputValidation，
 *         code 形如 "ar.validation.<snake_case>"）。
 */
ONEQ_API ArIssueList ValidateArCycleDeltaTime(double dt_sec);

/**
 * @brief 校验完整周期输入。
 * @param[in] input 当前周期输入。
 * @return 按发现顺序返回的问题条目列表（所有条目 phase=kInputValidation）。
 */
ONEQ_API ArIssueList ValidateArCycleInput(const ArCycleInput& input);

/**
 * @brief 校验场景目标列表。
 * @param[in] targets 当前周期场景目标列表。
 * @return 按发现顺序返回的问题条目列表（所有条目 phase=kInputValidation）。
 */
ONEQ_API ArIssueList ValidateArSceneTargets(const ArSceneTargetList& targets);

/**
 * @brief 判断问题列表中是否存在输入校验 error 级问题。
 * @param[in] issues 问题条目列表。
 * @return 存在 phase==kInputValidation 且 severity==kError 的条目时返回 `true`。
 */
ONEQ_API bool HasValidationError(const ArIssueList& issues);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_
