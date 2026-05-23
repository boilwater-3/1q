/**
 * @file EsrEnvironmentInputState.h
 * @brief 定义 ESR 调用方侧环境输入状态维护对象。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_STATE_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_STATE_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInputPatch.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrEnvironmentInputState 维护调用方侧当前环境事实状态。
 */
class ONEQ_API EsrEnvironmentInputState {
 public:
  EsrEnvironmentInputState() = default;
  explicit EsrEnvironmentInputState(const EsrEnvironmentInput& snapshot) : snapshot_(snapshot) {}

  EsrEnvironmentInputState& Reset(const EsrEnvironmentInput& snapshot) {
    snapshot_ = snapshot;
    return *this;
  }

  EsrEnvironmentInputState& Update(const EsrEnvironmentInputPatch& patch) {
    if (patch.has_propagation_profile) {
      snapshot_.propagation_profile = patch.propagation_profile;
    }
    if (patch.has_clutter_density) {
      snapshot_.clutter_density = patch.clutter_density;
    }
    if (patch.has_spectrum_occupancy_ratio) {
      snapshot_.spectrum_occupancy_ratio = patch.spectrum_occupancy_ratio;
    }
    if (patch.has_atmospheric_observation) {
      snapshot_.atmospheric_observation = patch.atmospheric_observation;
    }
    if (patch.has_jammer_sources) {
      snapshot_.jammer_sources = patch.jammer_sources;
    }
    return *this;
  }

  EsrEnvironmentInput Snapshot() const { return snapshot_; }

 private:
  EsrEnvironmentInput snapshot_{};
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_STATE_H_
