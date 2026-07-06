/**
 * @file SbirsCycleInputAdapter.h
 * @brief 定义 SBIRS-inspired 输入构造辅助。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_ADAPTER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_ADAPTER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"

namespace sbirs_sensor {
namespace session {

class ONEQ_API SbirsCycleInputBuilder {
 public:
  SbirsCycleInputBuilder& WithCycleIndex(std::uint32_t cycle_index);
  SbirsCycleInputBuilder& WithDeltaTimeSec(float dt_sec);
  SbirsCycleInputBuilder& WithSatellitePosition(const SbirsVector3M& position_ecef_m);
  SbirsCycleInputBuilder& WithEnvironment(const SbirsEnvironmentInput& environment);
  SbirsCycleInputBuilder& AddTarget(const SbirsSceneTarget& target);
  SbirsCycleInput Build() const;

 private:
  SbirsCycleInput input_{};
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_ADAPTER_H_
