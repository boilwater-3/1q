/**
 * @file DecisionControlTypes.h
 * @brief 定义 AR 决策控制公共 DTO。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_DECISION_CONTROL_TYPES_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_DECISION_CONTROL_TYPES_H_

#include <cstdint>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/** @brief 一个成功探测周期结束后发布给外部决策模块的观测。 */
struct ONEQ_API DecisionObservation {
  session::DecisionInputFrame input_frame{};
  session::ArControlProfile active_control_profile{};
};

/** @brief 外部 profile 覆盖提交。profile 为整包替换值，将在下一成功周期替换原生归约结果。 */
struct ONEQ_API ExternalDecisionOverride {
  session::ArControlProfile profile{};
};

enum class ONEQ_API ExternalDecisionSubmitStatus {
  kAccepted = 0,
  kNoPendingObservation,
  kAlreadySubmitted,
  kInvalidProfile
};

enum class ONEQ_API DecisionControlSource {
  kNone = 0,
  kInternal,
  kExternal
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_DECISION_CONTROL_TYPES_H_
