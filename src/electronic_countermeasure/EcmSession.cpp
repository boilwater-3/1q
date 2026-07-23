#include "1q/electronic_countermeasure/EcmSession.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <utility>

namespace electronic_countermeasure {
namespace session {
namespace {

const std::uint32_t kRuntimeStateSchemaVersion = 1U;
const std::uint32_t kMaximumGlideSuccessfulCycles = 2U;

bool IsKnownTechnique(EcmTechnique technique) {
  return technique == EcmTechnique::kSpot || technique == EcmTechnique::kBarrage ||
         technique == EcmTechnique::kSweep;
}

bool IsValidConfig(const config::EcmSessionConfig& config) {
  return config.transmitter_equipment_id != 0U && config.channel_count > 0U &&
         std::isfinite(config.minimum_frequency_hz) &&
         std::isfinite(config.maximum_frequency_hz) && config.minimum_frequency_hz > 0.0 &&
         config.maximum_frequency_hz > config.minimum_frequency_hz &&
         std::isfinite(config.maximum_total_transmit_power_w) &&
         config.maximum_total_transmit_power_w > 0.0 &&
         std::isfinite(config.maximum_channel_transmit_power_w) &&
         config.maximum_channel_transmit_power_w > 0.0 &&
         config.maximum_channel_transmit_power_w <= config.maximum_total_transmit_power_w &&
         std::isfinite(config.thermal_capacity_j) && config.thermal_capacity_j > 0.0 &&
         std::isfinite(config.cooling_power_w) && config.cooling_power_w >= 0.0 &&
         std::isfinite(config.spot_bandwidth_hz) && config.spot_bandwidth_hz > 0.0 &&
         std::isfinite(config.barrage_bandwidth_hz) && config.barrage_bandwidth_hz > 0.0 &&
         std::isfinite(config.sweep_bandwidth_hz) && config.sweep_bandwidth_hz > 0.0 &&
         config.sweep_segment_count > 0U && IsKnownTechnique(config.default_technique);
}

bool IsValidSensorObservation(const EcmSensorObservation& observation) {
  return std::isfinite(observation.estimated_center_frequency_hz) &&
         observation.estimated_center_frequency_hz > 0.0 &&
         std::isfinite(observation.estimated_bandwidth_hz) &&
         observation.estimated_bandwidth_hz > 0.0 &&
         std::isfinite(observation.estimated_pri_s) && observation.estimated_pri_s >= 0.0 &&
         std::isfinite(observation.estimated_pulse_width_s) &&
         observation.estimated_pulse_width_s >= 0.0 &&
         std::isfinite(observation.center_frequency_std_hz) &&
         observation.center_frequency_std_hz >= 0.0 &&
         std::isfinite(observation.bandwidth_std_hz) && observation.bandwidth_std_hz >= 0.0 &&
         std::isfinite(observation.bearing_az_deg) &&
         std::isfinite(observation.bearing_el_deg) &&
         std::isfinite(observation.bearing_std_deg) && observation.bearing_std_deg >= 0.0 &&
         std::isfinite(observation.threat_score) && observation.threat_score >= 0.0f &&
         observation.threat_score <= 1.0f && std::isfinite(observation.confidence) &&
         observation.confidence >= 0.0f && observation.confidence <= 1.0f;
}

bool IsValidInput(const EcmCycleInput& input) {
  if (input.cycle_index == 0U || input.platform_entity_id == 0U ||
      !std::isfinite(input.cycle_start_time_s) || input.cycle_start_time_s < 0.0 ||
      !std::isfinite(input.dt_sec) || input.dt_sec <= 0.0 ||
      !std::isfinite(input.platform_position_ecef_m.x_m) ||
      !std::isfinite(input.platform_position_ecef_m.y_m) ||
      !std::isfinite(input.platform_position_ecef_m.z_m) ||
      !std::isfinite(input.platform_velocity_ecef_mps.x_mps) ||
      !std::isfinite(input.platform_velocity_ecef_mps.y_mps) ||
      !std::isfinite(input.platform_velocity_ecef_mps.z_mps)) {
    return false;
  }
  if (input.input_mode == EcmInputMode::kSensorDriven) {
    if (!input.truth_threats.empty() ||
        (!input.has_sensor_observation_frame &&
         (!input.sensor_observation_frame.observations.empty() ||
          input.sensor_observation_frame.source_esr_success_cycle_index != 0U))) {
      return false;
    }
    std::set<std::uint64_t> ids;
    for (const EcmSensorObservation& observation : input.sensor_observation_frame.observations) {
      if (!IsValidSensorObservation(observation) ||
          !ids.insert(observation.source_hypothesis_id).second) {
        return false;
      }
    }
    return !input.has_sensor_observation_frame ||
           input.sensor_observation_frame.source_esr_success_cycle_index < input.cycle_index;
  }
  if (input.input_mode != EcmInputMode::kTruthAssisted || input.has_sensor_observation_frame ||
      !input.sensor_observation_frame.observations.empty()) {
    return false;
  }
  std::set<std::uint64_t> ids;
  for (const EcmTruthThreat& threat : input.truth_threats) {
    if (!ids.insert(threat.truth_entity_id).second || !std::isfinite(threat.center_frequency_hz) ||
        threat.center_frequency_hz <= 0.0 || !std::isfinite(threat.bandwidth_hz) ||
        threat.bandwidth_hz <= 0.0 || !std::isfinite(threat.threat_score) ||
        threat.threat_score < 0.0f || threat.threat_score > 1.0f) {
      return false;
    }
  }
  return true;
}

struct SchedulingThreat {
  std::uint64_t observation_id{0U};
  std::uint64_t truth_entity_id{0U};
  double center_frequency_hz{0.0};
  double bandwidth_hz{0.0};
  float score{0.0f};
};

bool IsFeasibleThreat(const SchedulingThreat& threat,
                      const config::EcmSessionConfig& config) {
  double occupied_bandwidth_hz = config.spot_bandwidth_hz;
  if (config.default_technique == EcmTechnique::kBarrage) {
    occupied_bandwidth_hz = std::max(config.barrage_bandwidth_hz, threat.bandwidth_hz);
  } else if (config.default_technique == EcmTechnique::kSweep) {
    occupied_bandwidth_hz = config.sweep_bandwidth_hz;
  }
  return threat.center_frequency_hz - 0.5 * occupied_bandwidth_hz >=
             config.minimum_frequency_hz &&
         threat.center_frequency_hz + 0.5 * occupied_bandwidth_hz <=
             config.maximum_frequency_hz;
}

bool TryBuildEmission(
    const EcmCycleInput& input, const config::EcmSessionConfig& config,
    const SchedulingThreat& threat, double allocated_power_w, std::uint32_t channel_index,
    std::uint64_t emission_id, std::mt19937* scheduling_rng,
    oneq::electromagnetics::RfSceneEmission* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = input.platform_entity_id;
  emission.identity.equipment_id = config.transmitter_equipment_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m = input.platform_position_ecef_m;
  emission.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  emission.antenna = input.transmit_antenna;
  emission.polarization = input.transmit_polarization;
  const EcmTechnique technique = config.default_technique;
  if (technique == EcmTechnique::kSweep) {
    bool reverse_sweep = false;
    if (scheduling_rng != nullptr) {
      std::uniform_int_distribution<std::uint32_t> distribution(0U, 1U);
      reverse_sweep = distribution(*scheduling_rng) != 0U;
    }
    double start_frequency_hz = threat.center_frequency_hz - 0.5 * config.sweep_bandwidth_hz;
    double stop_frequency_hz = threat.center_frequency_hz + 0.5 * config.sweep_bandwidth_hz;
    if (reverse_sweep) {
      std::swap(start_frequency_hz, stop_frequency_hz);
    }
    const double instantaneous_bandwidth_hz =
        config.sweep_bandwidth_hz / static_cast<double>(config.sweep_segment_count);
    if (!oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
            input.cycle_start_time_s, input.dt_sec, start_frequency_hz,
            stop_frequency_hz, instantaneous_bandwidth_hz, allocated_power_w,
            input.dt_sec, &emission.waveform)) {
      return false;
    }
  } else {
    const double bandwidth_hz = technique == EcmTechnique::kBarrage
                                    ? std::max(config.barrage_bandwidth_hz,
                                               threat.bandwidth_hz)
                                    : config.spot_bandwidth_hz;
    if (!oneq::electromagnetics::TryCreateRfNoiseWaveform(
            input.cycle_start_time_s, input.dt_sec, threat.center_frequency_hz,
            bandwidth_hz, allocated_power_w, &emission.waveform)) {
      return false;
    }
  }
  (void)channel_index;
  *output = emission;
  return true;
}

}  // namespace

struct EcmSession::Impl {
  explicit Impl(config::EcmSessionConfig value) : active_config(std::move(value)),
                                                   scheduling_rng(active_config.random_seed) {}
  config::EcmSessionConfig active_config{};
  bool has_successful_cycle{false};
  bool has_last_sensor_frame{false};
  EcmSensorObservationFrame last_sensor_frame{};
  std::uint32_t observation_age_successful_ecm_cycles{0U};
  std::uint32_t last_successful_cycle_index{0U};
  bool has_world_chronology{false};
  std::uint32_t last_world_cycle_index{0U};
  double last_world_window_end_time_s{0.0};
  std::uint64_t next_emission_id{1U};
  double thermal_energy_j{0.0};
  std::mt19937 scheduling_rng{};
};

EcmSession::EcmSession() : impl_(new Impl(config::EcmSessionConfig())) {}
EcmSession::~EcmSession() = default;
EcmSession::EcmSession(EcmSession&&) noexcept = default;
EcmSession& EcmSession::operator=(EcmSession&&) noexcept = default;
EcmSession::EcmSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EcmSession EcmSession::Create(const config::EcmSessionConfig& config) {
  return EcmSession(std::unique_ptr<Impl>(new Impl(config)));
}

EcmCycleResult EcmSession::StepWithResult(const EcmCycleInput& input) {
  EcmCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.input_mode = input.input_mode;
  result.truth_assisted = input.input_mode == EcmInputMode::kTruthAssisted;
  result.emission_frame.world_cycle_index = input.cycle_index;
  result.emission_frame.window_start_time_s = input.cycle_start_time_s;
  result.emission_frame.window_duration_s = input.dt_sec;
  if (!IsValidConfig(impl_->active_config)) {
    result.status = EcmCycleStatus::kRejectedInvalidConfig;
    result.thermal_energy_j = impl_->thermal_energy_j;
    return result;
  }
  if (!IsValidInput(input) ||
      (impl_->has_successful_cycle && input.cycle_index <= impl_->last_successful_cycle_index) ||
      (impl_->has_world_chronology &&
       (input.cycle_index <= impl_->last_world_cycle_index ||
        input.cycle_start_time_s < impl_->last_world_window_end_time_s))) {
    result.status = EcmCycleStatus::kRejectedInvalidInput;
    result.thermal_energy_j = impl_->thermal_energy_j;
    return result;
  }
  if (!impl_->active_config.power_on) {
    result.status = EcmCycleStatus::kPoweredOff;
    result.thermal_energy_j = impl_->thermal_energy_j;
    impl_->has_world_chronology = true;
    impl_->last_world_cycle_index = input.cycle_index;
    impl_->last_world_window_end_time_s = input.cycle_start_time_s + input.dt_sec;
    return result;
  }

  bool candidate_has_last_frame = impl_->has_last_sensor_frame;
  EcmSensorObservationFrame candidate_last_frame = impl_->last_sensor_frame;
  std::uint32_t candidate_age = impl_->observation_age_successful_ecm_cycles;
  std::uint64_t candidate_next_emission_id = impl_->next_emission_id;
  std::mt19937 candidate_rng = impl_->scheduling_rng;
  double candidate_thermal_energy_j = std::max(
      0.0, impl_->thermal_energy_j - impl_->active_config.cooling_power_w * input.dt_sec);
  std::vector<SchedulingThreat> threats;

  if (input.input_mode == EcmInputMode::kSensorDriven) {
    if (input.has_sensor_observation_frame) {
      if (candidate_has_last_frame &&
          input.sensor_observation_frame.source_esr_success_cycle_index <=
              candidate_last_frame.source_esr_success_cycle_index) {
        result.status = EcmCycleStatus::kRejectedInvalidInput;
        result.thermal_energy_j = impl_->thermal_energy_j;
        return result;
      }
      candidate_last_frame = input.sensor_observation_frame;
      candidate_has_last_frame = true;
      candidate_age = 0U;
    } else if (candidate_has_last_frame) {
      ++candidate_age;
      result.used_glided_observation =
          candidate_age <= kMaximumGlideSuccessfulCycles;
    }
    if (candidate_has_last_frame && candidate_age <= kMaximumGlideSuccessfulCycles) {
      result.source_esr_success_cycle_index =
          candidate_last_frame.source_esr_success_cycle_index;
      for (const EcmSensorObservation& observation : candidate_last_frame.observations) {
        SchedulingThreat threat;
        threat.observation_id = observation.source_hypothesis_id;
        threat.center_frequency_hz = observation.estimated_center_frequency_hz;
        threat.bandwidth_hz = observation.estimated_bandwidth_hz;
        threat.score = observation.threat_score;
        threats.push_back(threat);
      }
    }
  } else {
    for (const EcmTruthThreat& truth : input.truth_threats) {
      SchedulingThreat threat;
      threat.truth_entity_id = truth.truth_entity_id;
      threat.center_frequency_hz = truth.center_frequency_hz;
      threat.bandwidth_hz = truth.bandwidth_hz;
      threat.score = truth.threat_score;
      threats.push_back(threat);
    }
  }

  std::stable_sort(threats.begin(), threats.end(), [](const SchedulingThreat& lhs,
                                                       const SchedulingThreat& rhs) {
    if (lhs.score != rhs.score) {
      return lhs.score > rhs.score;
    }
    return std::make_pair(lhs.observation_id, lhs.truth_entity_id) <
           std::make_pair(rhs.observation_id, rhs.truth_entity_id);
  });
  threats.erase(std::remove_if(threats.begin(), threats.end(), [&](const SchedulingThreat& threat) {
                  return !IsFeasibleThreat(threat, impl_->active_config);
                }),
                threats.end());

  const std::size_t selected_count =
      std::min<std::size_t>(threats.size(), impl_->active_config.channel_count);
  const double thermal_power_limit_w =
      std::max(0.0, impl_->active_config.thermal_capacity_j - candidate_thermal_energy_j) /
      input.dt_sec;
  double remaining_power_w =
      std::min(impl_->active_config.maximum_total_transmit_power_w, thermal_power_limit_w);
  for (std::size_t index = 0U; index < selected_count && remaining_power_w > 0.0; ++index) {
    const double allocated_power_w = std::min(
        impl_->active_config.maximum_channel_transmit_power_w,
        remaining_power_w / static_cast<double>(selected_count - index));
    if (allocated_power_w <= 0.0) {
      break;
    }
    EcmResourceDecision decision;
    decision.source_observation_id = threats[index].observation_id;
    decision.truth_entity_id = threats[index].truth_entity_id;
    decision.technique = impl_->active_config.default_technique;
    decision.channel_index = static_cast<std::uint32_t>(index);
    decision.allocated_power_w = allocated_power_w;
    decision.reason = "highest-threat feasible channel allocation";
    result.decisions.push_back(decision);
    oneq::electromagnetics::RfSceneEmission emission;
    if (!TryBuildEmission(input, impl_->active_config, threats[index], allocated_power_w,
                          static_cast<std::uint32_t>(index), candidate_next_emission_id,
                          &candidate_rng, &emission)) {
      result.status = EcmCycleStatus::kRejectedInvalidConfig;
      result.decisions.clear();
      result.emission_frame.emissions.clear();
      result.thermal_energy_j = impl_->thermal_energy_j;
      return result;
    }
    ++candidate_next_emission_id;
    result.emission_frame.emissions.push_back(emission);
    remaining_power_w -= allocated_power_w;
    candidate_thermal_energy_j += allocated_power_w * input.dt_sec;
  }

  if (!oneq::electromagnetics::TryValidateRfSceneFrame(result.emission_frame)) {
    result.status = EcmCycleStatus::kRejectedInvalidConfig;
    result.decisions.clear();
    result.emission_frame.emissions.clear();
    result.thermal_energy_j = impl_->thermal_energy_j;
    return result;
  }
  result.executed_this_cycle = true;
  result.status = result.emission_frame.emissions.empty()
                      ? EcmCycleStatus::kSafeStopNoFreshObservation
                      : EcmCycleStatus::kExecuted;
  result.observation_age_successful_ecm_cycles = candidate_age;
  result.thermal_energy_j = candidate_thermal_energy_j;

  impl_->has_successful_cycle = true;
  impl_->has_last_sensor_frame = candidate_has_last_frame;
  impl_->last_sensor_frame = candidate_last_frame;
  impl_->observation_age_successful_ecm_cycles = candidate_age;
  impl_->last_successful_cycle_index = input.cycle_index;
  impl_->has_world_chronology = true;
  impl_->last_world_cycle_index = input.cycle_index;
  impl_->last_world_window_end_time_s = input.cycle_start_time_s + input.dt_sec;
  impl_->next_emission_id = candidate_next_emission_id;
  impl_->thermal_energy_j = candidate_thermal_energy_j;
  impl_->scheduling_rng = candidate_rng;
  return result;
}

EcmRuntimeConfigApplyResult EcmSession::ApplyRuntimeConfig(
    const config::EcmRuntimeConfigPatch& patch) {
  EcmRuntimeConfigApplyResult result;
  result.has_requested_update = patch.has_power_on ||
                                patch.has_maximum_total_transmit_power_w ||
                                patch.has_default_technique;
  if (!result.has_requested_update) {
    return result;
  }
  config::EcmSessionConfig candidate = impl_->active_config;
  if (patch.has_power_on) {
    candidate.power_on = patch.power_on;
  }
  if (patch.has_maximum_total_transmit_power_w) {
    candidate.maximum_total_transmit_power_w = patch.maximum_total_transmit_power_w;
  }
  if (patch.has_default_technique) {
    candidate.default_technique = patch.default_technique;
  }
  if (!IsValidConfig(candidate)) {
    return result;
  }
  impl_->active_config = candidate;
  result.applied = true;
  return result;
}

EcmRuntimeState EcmSession::CaptureRuntimeState() const {
  EcmRuntimeState state;
  state.owner_identity = impl_.get();
  state.schema_version = kRuntimeStateSchemaVersion;
  state.active_config = impl_->active_config;
  state.has_successful_cycle = impl_->has_successful_cycle;
  state.has_last_sensor_frame = impl_->has_last_sensor_frame;
  state.last_sensor_frame = impl_->last_sensor_frame;
  state.observation_age_successful_ecm_cycles =
      impl_->observation_age_successful_ecm_cycles;
  state.last_successful_cycle_index = impl_->last_successful_cycle_index;
  state.has_world_chronology = impl_->has_world_chronology;
  state.last_world_cycle_index = impl_->last_world_cycle_index;
  state.last_world_window_end_time_s = impl_->last_world_window_end_time_s;
  state.next_emission_id = impl_->next_emission_id;
  state.thermal_energy_j = impl_->thermal_energy_j;
  std::ostringstream stream;
  stream << impl_->scheduling_rng;
  state.scheduling_rng_state = stream.str();
  return state;
}

bool EcmSession::RestoreRuntimeState(const EcmRuntimeState& state) {
  if (state.owner_identity != impl_.get() || state.schema_version != kRuntimeStateSchemaVersion ||
      !IsValidConfig(state.active_config) || !std::isfinite(state.thermal_energy_j) ||
      !std::isfinite(state.last_world_window_end_time_s) ||
      (state.has_world_chronology && state.last_world_window_end_time_s <= 0.0) ||
      (!state.has_world_chronology && state.last_world_cycle_index != 0U) ||
      state.thermal_energy_j < 0.0 || state.thermal_energy_j > state.active_config.thermal_capacity_j ||
      state.next_emission_id == 0U ||
      (!state.has_last_sensor_frame &&
       (!state.last_sensor_frame.observations.empty() ||
        state.observation_age_successful_ecm_cycles != 0U))) {
    return false;
  }
  std::istringstream stream(state.scheduling_rng_state);
  std::mt19937 candidate_rng;
  if (!(stream >> candidate_rng)) {
    return false;
  }
  impl_->active_config = state.active_config;
  impl_->has_successful_cycle = state.has_successful_cycle;
  impl_->has_last_sensor_frame = state.has_last_sensor_frame;
  impl_->last_sensor_frame = state.last_sensor_frame;
  impl_->observation_age_successful_ecm_cycles =
      state.observation_age_successful_ecm_cycles;
  impl_->last_successful_cycle_index = state.last_successful_cycle_index;
  impl_->has_world_chronology = state.has_world_chronology;
  impl_->last_world_cycle_index = state.last_world_cycle_index;
  impl_->last_world_window_end_time_s = state.last_world_window_end_time_s;
  impl_->next_emission_id = state.next_emission_id;
  impl_->thermal_energy_j = state.thermal_energy_j;
  impl_->scheduling_rng = candidate_rng;
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
