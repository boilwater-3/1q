/**
 * @file SbirsCycleInput.h
 * @brief 定义 SBIRS-inspired 单周期输入。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsEnvironmentInput.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace session {

struct ONEQ_API SbirsCycleInput {
  std::uint32_t cycle_index{0U};
  float dt_sec{1.0f};
  bool has_satellite_position{false};
  SbirsVector3M satellite_position_ecef_m{};
  SbirsSceneTargetList scene{};
  SbirsEnvironmentInput environment{};
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
