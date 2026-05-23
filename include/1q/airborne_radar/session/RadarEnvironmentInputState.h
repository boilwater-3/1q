/**
 * @file RadarEnvironmentInputState.h
 * @brief 定义 AR 调用方侧环境输入状态维护对象。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_STATE_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_STATE_H_

#include "1q/airborne_radar/session/RadarEnvironmentInputPatch.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarEnvironmentInputState 维护调用方侧当前环境事实状态。
 */
class ONEQ_API RadarEnvironmentInputState {
 public:
  RadarEnvironmentInputState() = default;
  explicit RadarEnvironmentInputState(const RadarEnvironmentInput& snapshot)
      : snapshot_(snapshot) {}

  RadarEnvironmentInputState& Reset(const RadarEnvironmentInput& snapshot) {
    snapshot_ = snapshot;
    return *this;
  }

  RadarEnvironmentInputState& Update(const RadarEnvironmentInputPatch& patch) {
    if (patch.has_atmospheric_observation) {
      snapshot_.atmospheric_observation = patch.atmospheric_observation;
    }
    if (patch.has_atmospheric_context) {
      snapshot_.atmospheric_context = patch.atmospheric_context;
    }
    if (patch.has_surface_observation) {
      snapshot_.surface_observation = patch.surface_observation;
    }
    if (patch.has_jammer_sources) {
      snapshot_.jammer_sources = patch.jammer_sources;
    }
    return *this;
  }

  RadarEnvironmentInput Snapshot() const { return snapshot_; }

 private:
  RadarEnvironmentInput snapshot_{};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_STATE_H_
