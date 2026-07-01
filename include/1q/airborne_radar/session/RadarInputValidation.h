/**
 * @file RadarInputValidation.h
 * @brief Deprecated compat wrapper — include ArInputValidation.h instead.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_INPUT_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_INPUT_VALIDATION_H_

#include "1q/airborne_radar/session/ArInputValidation.h"

namespace airborne_radar {
namespace session {

inline ValidationIssueList ValidateRadarCycleDeltaTime(float dt_sec) {
  return ValidateArCycleDeltaTime(dt_sec);
}

inline ValidationIssueList ValidateRadarCycleInput(const ArCycleInput& input) {
  return ValidateArCycleInput(input);
}

inline ValidationIssueList ValidateRadarSceneTargets(const ArSceneTargetList& targets) {
  return ValidateArSceneTargets(targets);
}

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_INPUT_VALIDATION_H_
