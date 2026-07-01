/**
 * @file ArInputValidation.h
 * @brief AR module primary validation entry points for cycle inputs.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"

namespace airborne_radar {
namespace session {

inline ValidationIssueList ValidateArCycleInput(const ArCycleInput& input) {
  return ValidateRadarCycleInput(input);
}

inline ValidationIssueList ValidateArSceneTargets(const ArSceneTargetList& targets) {
  return ValidateRadarSceneTargets(targets);
}

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_INPUT_VALIDATION_H_
