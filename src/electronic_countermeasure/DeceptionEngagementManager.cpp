/**
 * @file DeceptionEngagementManager.cpp
 * @brief DeceptionEngagementManager 实现：欺骗状态机和 RNG 管理。
 */

#include "electronic_countermeasure/DeceptionEngagementManager.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "electronic_countermeasure/EcmInternalTypes.h"

namespace electronic_countermeasure {
namespace session {

DeceptionEngagementManager::DeceptionEngagementManager(std::uint32_t random_seed)
    : deception_rng_(DeriveStreamSeed(random_seed, kDeceptionDomain)) {}

void DeceptionEngagementManager::AdvanceStates(const config::EcmSessionConfig& config,
                                               double dt_sec) {
  const double c = 299792458.0;
  for (EcmDeceptionState& state : states_) {
    if (!state.engaged) {
      continue;
    }
    state.phase_elapsed_s += dt_sec;
    state.cycle_count++;

    switch (state.phase) {
      case EcmDeceptionPhase::kTowing: {
        const double max_delay = 2.0 * config.deception_rgpo_max_range_m / c;
        if (state.mode == EcmDeceptionMode::kRgpo || state.mode == EcmDeceptionMode::kRgpoVgpo) {
          state.current_delay_s += 2.0 * (config.deception_rgpo_rate_m_per_s * dt_sec) / c;
          if (state.current_delay_s >= max_delay) {
            state.current_delay_s = max_delay;
            if (state.mode != EcmDeceptionMode::kRgpoVgpo) {
              state.phase = EcmDeceptionPhase::kHolding;
              state.phase_elapsed_s = 0.0;
            }
          }
        }
        if (state.mode == EcmDeceptionMode::kVgpo || state.mode == EcmDeceptionMode::kRgpoVgpo) {
          state.current_doppler_offset_hz += config.deception_vgpo_rate_hz_per_s * dt_sec;
          if (std::abs(state.current_doppler_offset_hz) >= config.deception_vgpo_max_doppler_hz) {
            state.current_doppler_offset_hz = config.deception_vgpo_max_doppler_hz;
            if (state.mode != EcmDeceptionMode::kRgpoVgpo) {
              state.phase = EcmDeceptionPhase::kHolding;
              state.phase_elapsed_s = 0.0;
            }
          }
        }
        if (state.mode == EcmDeceptionMode::kRgpoVgpo) {
          if (state.current_delay_s >= max_delay &&
              std::abs(state.current_doppler_offset_hz) >= config.deception_vgpo_max_doppler_hz) {
            state.phase = EcmDeceptionPhase::kHolding;
            state.phase_elapsed_s = 0.0;
          }
        }
        if (state.mode == EcmDeceptionMode::kFalseTarget) {
          state.engaged = false;
          state.phase = EcmDeceptionPhase::kIdle;
        }
        break;
      }

      case EcmDeceptionPhase::kHolding:
        if (state.phase_elapsed_s >= config.deception_hold_time_s) {
          state.phase = EcmDeceptionPhase::kStopped;
          state.phase_elapsed_s = 0.0;
        }
        break;

      case EcmDeceptionPhase::kStopped:
        if (state.phase_elapsed_s >= dt_sec) {
          state.engaged = false;
          state.phase = EcmDeceptionPhase::kIdle;
        }
        break;

      case EcmDeceptionPhase::kIdle:
        break;
    }
  }
  states_.erase(std::remove_if(states_.begin(), states_.end(),
                               [](const EcmDeceptionState& s) {
                                 return !s.engaged && s.phase == EcmDeceptionPhase::kIdle;
                               }),
                states_.end());
}

EcmDeceptionState* DeceptionEngagementManager::FindOrCreate(std::uint64_t threat_id,
                                                            EcmDeceptionMode mode,
                                                            std::uint32_t max_active) {
  for (EcmDeceptionState& state : states_) {
    if (state.threat_id == threat_id && state.engaged) {
      return &state;
    }
  }
  const std::size_t engaged_count = static_cast<std::size_t>(std::count_if(
      states_.begin(), states_.end(), [](const EcmDeceptionState& s) { return s.engaged; }));
  if (engaged_count >= max_active) {
    return nullptr;
  }
  EcmDeceptionState new_state;
  new_state.threat_id = threat_id;
  new_state.mode = mode;
  new_state.phase = EcmDeceptionPhase::kTowing;
  new_state.engaged = true;
  states_.push_back(new_state);
  return &states_.back();
}

void DeceptionEngagementManager::Clear() { states_.clear(); }

bool DeceptionEngagementManager::ModeChangeInvalidates(
    const config::EcmSessionConfig& old_config, const config::EcmSessionConfig& new_config) {
  return !new_config.power_on || new_config.default_technique != EcmTechnique::kDeception ||
         new_config.default_deception_mode != old_config.default_deception_mode;
}

std::vector<EcmDeceptionState> DeceptionEngagementManager::CaptureStates() const {
  return states_;
}

void DeceptionEngagementManager::RestoreStates(const std::vector<EcmDeceptionState>& states) {
  states_ = states;
}

std::mt19937& DeceptionEngagementManager::deception_rng() { return deception_rng_; }

const std::mt19937& DeceptionEngagementManager::deception_rng() const { return deception_rng_; }

void DeceptionEngagementManager::SetDeceptionRng(std::mt19937 rng) { deception_rng_ = rng; }

std::string DeceptionEngagementManager::SerializeDeceptionRng() const {
  std::ostringstream stream;
  stream << deception_rng_;
  return stream.str();
}

bool DeceptionEngagementManager::DeserializeDeceptionRng(const std::string& state) {
  std::istringstream stream(state);
  std::mt19937 candidate;
  if (!(stream >> candidate)) {
    return false;
  }
  deception_rng_ = candidate;
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
