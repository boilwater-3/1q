#include "1q/electronic_countermeasure/EcmSession.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <utility>

namespace electronic_countermeasure {
namespace session {
namespace {

const std::uint32_t kRuntimeStateSchemaVersion = 2U;
const std::uint32_t kMaximumGlideSuccessfulCycles = 2U;

// Derive independent RNG sub-streams from the single config seed via a splitmix32
// finalizer keyed by a per-consumer domain tag, mirroring the SBIRS
// DeriveMeasurementSeed convention (src/sbirs_sensor/pipeline/SbirsPipeline.cpp).
// Two streams with the same base seed but distinct tags are uncorrelated.
std::uint32_t DeriveStreamSeed(std::uint32_t base_seed, std::uint32_t domain_tag) {
  std::uint32_t value = base_seed ^ domain_tag;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value == 0U ? 1U : value;
}

// FourCC-style domain tags: scheduling drives sweep direction, tie-break drives
// equal-score ordering. Each consumer keeps its own mt19937 seeded independently.
const std::uint32_t kSchedulingDomain = UINT32_C(0x53434844);  // "SCHD"
const std::uint32_t kTieBreakDomain = UINT32_C(0x54494542);    // "TIEB"
const std::uint32_t kDeceptionDomain = UINT32_C(0x44455054);   // "DEPT"

bool IsKnownTechnique(EcmTechnique technique) {
  return technique == EcmTechnique::kSpot || technique == EcmTechnique::kBarrage ||
         technique == EcmTechnique::kSweep || technique == EcmTechnique::kDeception;
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
         config.sweep_segment_count > 0U && IsKnownTechnique(config.default_technique) &&
         std::isfinite(config.deception_rgpo_rate_m_per_s) &&
         config.deception_rgpo_rate_m_per_s >= 0.0 &&
         std::isfinite(config.deception_rgpo_max_range_m) &&
         config.deception_rgpo_max_range_m >= 0.0 &&
         std::isfinite(config.deception_vgpo_rate_hz_per_s) &&
         config.deception_vgpo_rate_hz_per_s >= 0.0 &&
         std::isfinite(config.deception_vgpo_max_doppler_hz) &&
         config.deception_vgpo_max_doppler_hz >= 0.0 &&
         std::isfinite(config.deception_hold_time_s) &&
         config.deception_hold_time_s >= 0.0 &&
         std::isfinite(config.deception_power_scale) &&
         config.deception_power_scale > 0.0 && config.deception_power_scale <= 1.0 &&
         config.deception_max_active > 0U &&
         std::isfinite(config.deception_false_target_delay_s) &&
         config.deception_false_target_delay_s >= 0.0 &&
         std::isfinite(config.deception_false_target_delay_s) &&
         config.deception_false_target_delay_s >= 0.0 &&
         std::isfinite(config.deception_false_target_doppler_hz) &&
         config.deception_max_false_targets_per_threat > 0U;
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
          input.sensor_observation_frame.source_esr_batch_id != 0U)) ||
        (input.has_sensor_observation_frame &&
         input.sensor_observation_frame.source_esr_batch_id == 0U)) {
      // A present sensor frame must carry a real ESR batch_id (fresh-frame
      // provenance); an absent frame must not carry a stale batch_id.
      return false;
    }
    std::set<std::uint64_t> ids;
    for (const EcmSensorObservation& observation : input.sensor_observation_frame.observations) {
      if (!IsValidSensorObservation(observation) ||
          !ids.insert(observation.source_hypothesis_id).second) {
        return false;
      }
    }
    return true;
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
        threat.threat_score < 0.0f || threat.threat_score > 1.0f ||
        !std::isfinite(threat.estimated_pri_s) || threat.estimated_pri_s <= 0.0 ||
        !std::isfinite(threat.estimated_pulse_width_s) ||
        threat.estimated_pulse_width_s < 0.0) {
      return false;
    }
  }
  return true;
}

// Validate the internal consistency of a runtime-state snapshot beyond the
// scalar bounds already checked in RestoreRuntimeState. Mirrors the design §3
// requirement to "完整校验所有嵌套 observation、重复 ID、provenance、模式组合和
// 随机状态". Returns true only when the snapshot could have been produced by a
// well-formed session (so an externally/replay-constructed dirty snapshot is
// rejected before it can pollute impl_).
bool SnapshotInternallyConsistent(const EcmRuntimeState& state) {
  // has_successful_cycle must agree with last_successful_cycle_index: a session
  // that has never succeeded has index 0, and any successful cycle recorded a
  // non-zero cycle_index (cycle_index == 0U is an invalid input).
  if (state.has_successful_cycle == (state.last_successful_cycle_index == 0U)) {
    return false;
  }
  if (!state.has_last_sensor_frame) {
    // Without a retained sensor frame there must be no cached observations and
    // the glide age must be zero (preserves the prior shallow guard).
    return state.last_sensor_frame.observations.empty() &&
           state.observation_age_successful_ecm_cycles == 0U;
  }
  // Nested observation validation: every observation must be individually valid
  // and source_hypothesis_id must be unique within the frame (matches the input
  // contract enforced by IsValidInput for kSensorDriven).
  std::set<std::uint64_t> hypothesis_ids;
  for (const EcmSensorObservation& observation : state.last_sensor_frame.observations) {
    if (!IsValidSensorObservation(observation) ||
        !hypothesis_ids.insert(observation.source_hypothesis_id).second) {
      return false;
    }
  }
  for (const EcmDeceptionState& deception_state : state.deception_states) {
    if (deception_state.engaged) {
      if (deception_state.current_delay_s < 0.0 ||
          deception_state.phase_elapsed_s < 0.0 ||
          !std::isfinite(deception_state.current_delay_s) ||
          !std::isfinite(deception_state.current_doppler_offset_hz) ||
          !std::isfinite(deception_state.phase_elapsed_s)) {
        return false;
      }
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
  // Deterministic per-threat pseudo-random key used as the tie-break in the
  // sort comparator. Pre-derived (see AssignTieBreakKeys) so the comparator
  // remains a pure function; the draw order is independent of threat input order.
  std::uint32_t tie_break_key{0U};
  // Deception-specific: estimated PRI and pulse width from the threat source.
  double estimated_pri_s{1.0e-3};
  double estimated_pulse_width_s{1.0e-6};
};

// Stable per-threat identity: sensor-driven threats carry observation_id, truth
// threats carry truth_entity_id. Only one is ever non-zero per threat.
std::uint64_t ThreatStableId(const SchedulingThreat& threat) {
  return threat.observation_id != 0U ? threat.observation_id : threat.truth_entity_id;
}

// Derive a deterministic, input-order-independent pseudo-random tie-break key for
// each threat. The tie_break_rng is consumed exactly once per *unique* threat id,
// in canonical (sorted) id order — NOT in input order — so the same set of threats
// always yields the same {id -> key} mapping regardless of how they were supplied.
// This satisfies design §3: tie-break draws depend only on the threat set, never on
// input order, and the sort comparator stays a pure function (no RNG in the lambda).
void AssignTieBreakKeys(std::vector<SchedulingThreat>* threats, std::mt19937* tie_break_rng) {
  if (threats == nullptr || tie_break_rng == nullptr || threats->empty()) {
    return;
  }
  // Collect unique stable ids and canonicalize their order (pure id comparison).
  std::set<std::uint64_t> unique_ids;
  for (const SchedulingThreat& threat : *threats) {
    unique_ids.insert(ThreatStableId(threat));
  }
  // Draw one 32-bit word per unique id in canonical order; combine id + draw into
  // a final key so even ids drawn from correlated words diverge.
  std::map<std::uint64_t, std::uint32_t> keys;
  std::uniform_int_distribution<std::uint32_t> distribution;
  for (std::uint64_t id : unique_ids) {
    const std::uint32_t draw = distribution(*tie_break_rng);
    keys[id] = DeriveStreamSeed(static_cast<std::uint32_t>(id), draw);
  }
  for (SchedulingThreat& threat : *threats) {
    threat.tie_break_key = keys[ThreatStableId(threat)];
  }
}

bool IsFeasibleThreat(const SchedulingThreat& threat,
                      const config::EcmSessionConfig& config) {
  if (config.default_technique == EcmTechnique::kDeception) {
    return threat.center_frequency_hz >= config.minimum_frequency_hz &&
           threat.center_frequency_hz <= config.maximum_frequency_hz;
  }
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

void AdvanceDeceptionStates(std::vector<EcmDeceptionState>* states,
                            const config::EcmSessionConfig& config,
                            double dt_sec) {
  if (states == nullptr) {
    return;
  }
  const double c = 299792458.0;
  for (EcmDeceptionState& state : *states) {
    if (!state.engaged) {
      continue;
    }
    state.phase_elapsed_s += dt_sec;
    state.cycle_count++;

    switch (state.phase) {
      case EcmDeceptionPhase::kTowing:
        if (state.mode == EcmDeceptionMode::kRgpo ||
            state.mode == EcmDeceptionMode::kRgpoVgpo) {
          const double max_delay =
              2.0 * config.deception_rgpo_max_range_m / c;
          state.current_delay_s +=
              (config.deception_rgpo_rate_m_per_s * dt_sec) / c;
          if (state.current_delay_s >= max_delay) {
            state.current_delay_s = max_delay;
            if (state.mode != EcmDeceptionMode::kRgpoVgpo) {
              state.phase = EcmDeceptionPhase::kHolding;
              state.phase_elapsed_s = 0.0;
            }
          }
        }
        if (state.mode == EcmDeceptionMode::kVgpo ||
            state.mode == EcmDeceptionMode::kRgpoVgpo) {
          state.current_doppler_offset_hz +=
              config.deception_vgpo_rate_hz_per_s * dt_sec;
          if (std::abs(state.current_doppler_offset_hz) >=
              config.deception_vgpo_max_doppler_hz) {
            state.current_doppler_offset_hz =
                config.deception_vgpo_max_doppler_hz;
            if (state.mode != EcmDeceptionMode::kRgpoVgpo) {
              state.phase = EcmDeceptionPhase::kHolding;
              state.phase_elapsed_s = 0.0;
            }
          }
        }
        // For kRgpoVgpo, transition to kHolding only when both RGPO delay
        // and VGPO doppler have reached their respective maxima.
        if (state.mode == EcmDeceptionMode::kRgpoVgpo) {
          const double max_delay =
              2.0 * config.deception_rgpo_max_range_m / c;
          if (state.current_delay_s >= max_delay &&
              std::abs(state.current_doppler_offset_hz) >=
                  config.deception_vgpo_max_doppler_hz) {
            state.phase = EcmDeceptionPhase::kHolding;
            state.phase_elapsed_s = 0.0;
          }
        }
        if (state.mode == EcmDeceptionMode::kFalseTarget) {
          state.phase = EcmDeceptionPhase::kStopped;
          state.phase_elapsed_s = 0.0;
        }
        break;

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
  states->erase(std::remove_if(states->begin(), states->end(),
                    [](const EcmDeceptionState& s) {
                      return !s.engaged && s.phase == EcmDeceptionPhase::kIdle;
                    }),
                states->end());
}

// Returns a pointer into the states vector to the existing or newly created
// engagement for the given threat. The caller must not retain the pointer
// across vector mutations.
EcmDeceptionState* FindOrCreateDeceptionState(
    std::vector<EcmDeceptionState>* states, std::uint64_t threat_id,
    EcmDeceptionMode mode) {
  if (states == nullptr) {
    return nullptr;
  }
  for (EcmDeceptionState& state : *states) {
    if (state.threat_id == threat_id && state.engaged) {
      return &state;
    }
  }
  EcmDeceptionState new_state;
  new_state.threat_id = threat_id;
  new_state.mode = mode;
  new_state.phase = EcmDeceptionPhase::kTowing;
  new_state.engaged = true;
  states->push_back(new_state);
  return &states->back();
}

bool TryBuildDeceptionEmission(
    const EcmCycleInput& input, const config::EcmSessionConfig& config,
    const SchedulingThreat& threat, const EcmDeceptionState& deception_state,
    double allocated_power_w, std::uint64_t emission_id,
    std::mt19937* scheduling_rng,
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

  double center_frequency_hz =
      threat.center_frequency_hz + deception_state.current_doppler_offset_hz;
  double bandwidth_hz = std::max(1.0e6, threat.bandwidth_hz);
  double pri_s = std::max(1.0e-6, threat.estimated_pri_s);
  double pulse_width_s =
      std::min(threat.estimated_pulse_width_s, pri_s * 0.45);
  pulse_width_s = std::max(1.0e-9, pulse_width_s);
  double first_pulse_time_s =
      input.cycle_start_time_s + deception_state.current_delay_s;

  std::uint32_t pulse_count =
      static_cast<std::uint32_t>(std::floor(input.dt_sec / pri_s));
  if (pulse_count == 0U) {
    pulse_count = 1U;
  }
  double jitter_fraction = 0.05;
  std::uint64_t timing_seed = 0U;
  std::uint64_t timing_epoch = 0U;
  if (scheduling_rng != nullptr) {
    std::uniform_int_distribution<std::uint64_t> dist;
    timing_seed = dist(*scheduling_rng);
    timing_epoch = dist(*scheduling_rng);
  }

  bool ok = oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      first_pulse_time_s, center_frequency_hz, bandwidth_hz,
      allocated_power_w, pulse_width_s, pri_s, pulse_count, jitter_fraction,
      timing_seed, timing_epoch, &emission.waveform);
  if (ok) {
    *output = emission;
  }
  return ok;
}

bool TryBuildFalseTargetEmission(
    const EcmCycleInput& input, const config::EcmSessionConfig& config,
    const SchedulingThreat& threat, double allocated_power_w,
    std::uint64_t emission_id, std::mt19937* scheduling_rng,
    std::uint32_t ft_index,
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

  double center_frequency_hz =
      threat.center_frequency_hz + config.deception_false_target_doppler_hz;
  double bandwidth_hz = std::max(1.0e6, threat.bandwidth_hz);
  double pri_s = std::max(1.0e-6, threat.estimated_pri_s);
  double pulse_width_s =
      std::min(threat.estimated_pulse_width_s, pri_s * 0.45);
  pulse_width_s = std::max(1.0e-9, pulse_width_s);

  // Stagger false target delays so they don't coincide in time.
  double stagger_offset_s =
      static_cast<double>(ft_index) * 1.0e-6;
  double first_pulse_time_s =
      input.cycle_start_time_s + config.deception_false_target_delay_s +
      stagger_offset_s;

  std::uint32_t pulse_count =
      static_cast<std::uint32_t>(std::floor(input.dt_sec / pri_s));
  if (pulse_count == 0U) {
    pulse_count = 1U;
  }
  double jitter_fraction = 0.05;
  std::uint64_t timing_seed = 0U;
  std::uint64_t timing_epoch = 0U;
  if (scheduling_rng != nullptr) {
    std::uniform_int_distribution<std::uint64_t> dist;
    timing_seed = dist(*scheduling_rng);
    timing_epoch = dist(*scheduling_rng);
  }

  bool ok = oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      first_pulse_time_s, center_frequency_hz, bandwidth_hz,
      allocated_power_w, pulse_width_s, pri_s, pulse_count, jitter_fraction,
      timing_seed, timing_epoch, &emission.waveform);
  if (ok) {
    *output = emission;
  }
  return ok;
}

}  // namespace

struct EcmSession::Impl {
  explicit Impl(config::EcmSessionConfig value)
      : active_config(std::move(value)),
        scheduling_rng(DeriveStreamSeed(active_config.random_seed, kSchedulingDomain)),
        tie_break_rng(DeriveStreamSeed(active_config.random_seed, kTieBreakDomain)),
        deception_rng(DeriveStreamSeed(active_config.random_seed, kDeceptionDomain)) {}
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
  std::mt19937 tie_break_rng{};
  std::mt19937 deception_rng{};
  std::vector<EcmDeceptionState> deception_states{};
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
  std::mt19937 candidate_scheduling_rng = impl_->scheduling_rng;
  std::mt19937 candidate_tie_break_rng = impl_->tie_break_rng;
  std::mt19937 candidate_deception_rng = impl_->deception_rng;
  std::vector<EcmDeceptionState> candidate_deception_states =
      impl_->deception_states;
  double candidate_thermal_energy_j = std::max(
      0.0, impl_->thermal_energy_j - impl_->active_config.cooling_power_w * input.dt_sec);
  std::vector<SchedulingThreat> threats;

  if (input.input_mode == EcmInputMode::kSensorDriven) {
    if (input.has_sensor_observation_frame) {
      if (candidate_has_last_frame &&
          input.sensor_observation_frame.source_esr_batch_id <=
              candidate_last_frame.source_esr_batch_id) {
        // Fresh-frame provenance: a new frame must come from a strictly later
        // ESR success batch than the last consumed one (replays/stale frames
        // rejected). batch_id is the ESR monotonic success sequence.
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
      result.source_esr_batch_id =
          candidate_last_frame.source_esr_batch_id;
      for (const EcmSensorObservation& observation : candidate_last_frame.observations) {
        SchedulingThreat threat;
        threat.observation_id = observation.source_hypothesis_id;
        threat.center_frequency_hz = observation.estimated_center_frequency_hz;
        threat.bandwidth_hz = observation.estimated_bandwidth_hz;
        threat.score = observation.threat_score;
        threat.estimated_pri_s = observation.estimated_pri_s;
        threat.estimated_pulse_width_s = observation.estimated_pulse_width_s;
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
      threat.estimated_pri_s = truth.estimated_pri_s;
      threat.estimated_pulse_width_s = truth.estimated_pulse_width_s;
      threats.push_back(threat);
    }
  }

  // Advance deception state machine for existing engagements before scheduling.
  AdvanceDeceptionStates(&candidate_deception_states, impl_->active_config,
                         input.dt_sec);

  // Filter infeasible threats first: only threats that actually compete for
  // channels participate in the tie-break, so tie_break_rng consumption equals
  // the count of feasible candidates (input-order-independent).
  threats.erase(std::remove_if(threats.begin(), threats.end(), [&](const SchedulingThreat& threat) {
                  return !IsFeasibleThreat(threat, impl_->active_config);
                }),
                threats.end());
  // Pre-derive a deterministic tie-break key per threat before sorting, so the
  // comparator below is a pure function (no RNG inside it) and the draw sequence
  // depends only on the feasible threat set, not on input order.
  AssignTieBreakKeys(&threats, &candidate_tie_break_rng);
  std::stable_sort(threats.begin(), threats.end(), [](const SchedulingThreat& lhs,
                                                       const SchedulingThreat& rhs) {
    if (lhs.score != rhs.score) {
      return lhs.score > rhs.score;
    }
    // Same score: order by the pre-derived pseudo-random key (pure comparison).
    // Keys are unique per threat id, so this fully breaks ties deterministically.
    return lhs.tie_break_key < rhs.tie_break_key;
  });

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
    const EcmTechnique technique = impl_->active_config.default_technique;
    EcmResourceDecision decision;
    decision.source_observation_id = threats[index].observation_id;
    decision.truth_entity_id = threats[index].truth_entity_id;
    decision.technique = technique;
    decision.channel_index = static_cast<std::uint32_t>(index);
    decision.reason = "highest-threat feasible channel allocation";

    if (technique == EcmTechnique::kDeception) {
      EcmDeceptionState* state = FindOrCreateDeceptionState(
          &candidate_deception_states, ThreatStableId(threats[index]),
          impl_->active_config.default_deception_mode);
      if (state == nullptr) {
        result.status = EcmCycleStatus::kRejectedInvalidConfig;
        result.decisions.clear();
        result.emission_frame.emissions.clear();
        result.thermal_energy_j = impl_->thermal_energy_j;
        return result;
      }
      decision.deception_mode = state->mode;
      decision.deception_phase = state->phase;
      if (state->phase == EcmDeceptionPhase::kStopped ||
          state->phase == EcmDeceptionPhase::kIdle) {
        decision.reason = "deception engagement released or idle";
        result.decisions.push_back(decision);
        continue;
      }

      if (state->mode == EcmDeceptionMode::kFalseTarget) {
        const std::uint32_t ft_count =
            impl_->active_config.deception_max_false_targets_per_threat;
        const double ft_power_w =
            allocated_power_w / static_cast<double>(ft_count);
        for (std::uint32_t ft_idx = 0U; ft_idx < ft_count; ++ft_idx) {
          oneq::electromagnetics::RfSceneEmission emission;
          if (!TryBuildFalseTargetEmission(
                  input, impl_->active_config, threats[index], ft_power_w,
                  candidate_next_emission_id, &candidate_deception_rng, ft_idx,
                  &emission)) {
            result.status = EcmCycleStatus::kRejectedInvalidConfig;
            result.decisions.clear();
            result.emission_frame.emissions.clear();
            result.thermal_energy_j = impl_->thermal_energy_j;
            return result;
          }
          ++candidate_next_emission_id;
          result.emission_frame.emissions.push_back(emission);
        }
        decision.allocated_power_w = allocated_power_w;
      } else {
        const double deception_power_w =
            allocated_power_w * impl_->active_config.deception_power_scale;
        decision.allocated_power_w = deception_power_w;
        oneq::electromagnetics::RfSceneEmission emission;
        if (!TryBuildDeceptionEmission(
                input, impl_->active_config, threats[index], *state,
                deception_power_w, candidate_next_emission_id,
                &candidate_deception_rng, &emission)) {
          result.status = EcmCycleStatus::kRejectedInvalidConfig;
          result.decisions.clear();
          result.emission_frame.emissions.clear();
          result.thermal_energy_j = impl_->thermal_energy_j;
          return result;
        }
        ++candidate_next_emission_id;
        result.emission_frame.emissions.push_back(emission);
      }
    } else {
      decision.allocated_power_w = allocated_power_w;
      oneq::electromagnetics::RfSceneEmission emission;
      if (!TryBuildEmission(input, impl_->active_config, threats[index],
                            allocated_power_w,
                            static_cast<std::uint32_t>(index),
                            candidate_next_emission_id,
                            &candidate_scheduling_rng, &emission)) {
        result.status = EcmCycleStatus::kRejectedInvalidConfig;
        result.decisions.clear();
        result.emission_frame.emissions.clear();
        result.thermal_energy_j = impl_->thermal_energy_j;
        return result;
      }
      ++candidate_next_emission_id;
      result.emission_frame.emissions.push_back(emission);
    }
    result.decisions.push_back(decision);
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
  impl_->scheduling_rng = candidate_scheduling_rng;
  impl_->tie_break_rng = candidate_tie_break_rng;
  impl_->deception_rng = candidate_deception_rng;
  impl_->deception_states = candidate_deception_states;
  return result;
}

EcmRuntimeConfigApplyResult EcmSession::ApplyRuntimeConfig(
    const config::EcmRuntimeConfigPatch& patch) {
  EcmRuntimeConfigApplyResult result;
  result.has_requested_update = patch.has_power_on ||
                                patch.has_maximum_total_transmit_power_w ||
                                patch.has_default_technique ||
                                patch.has_default_deception_mode;
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
  if (patch.has_default_deception_mode) {
    candidate.default_deception_mode = patch.default_deception_mode;
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
  std::ostringstream scheduling_stream;
  scheduling_stream << impl_->scheduling_rng;
  state.scheduling_rng_state = scheduling_stream.str();
  std::ostringstream tie_break_stream;
  tie_break_stream << impl_->tie_break_rng;
  state.tie_break_rng_state = tie_break_stream.str();
  std::ostringstream deception_stream;
  deception_stream << impl_->deception_rng;
  state.deception_rng_state = deception_stream.str();
  state.deception_states = impl_->deception_states;
  return state;
}

bool EcmSession::RestoreRuntimeState(const EcmRuntimeState& state) {
  if (state.owner_identity != impl_.get() || state.schema_version != kRuntimeStateSchemaVersion ||
      !IsValidConfig(state.active_config) || !std::isfinite(state.thermal_energy_j) ||
      !std::isfinite(state.last_world_window_end_time_s) ||
      (state.has_world_chronology && state.last_world_window_end_time_s <= 0.0) ||
      (!state.has_world_chronology && state.last_world_cycle_index != 0U) ||
      state.thermal_energy_j < 0.0 || state.thermal_energy_j > state.active_config.thermal_capacity_j ||
      state.next_emission_id == 0U || !SnapshotInternallyConsistent(state)) {
    return false;
  }
  // Parse both RNG streams into local candidates before mutating impl_; on any
  // parse failure the session is left untouched (fail-closed, no partial restore).
  std::istringstream scheduling_stream(state.scheduling_rng_state);
  std::mt19937 candidate_scheduling_rng;
  if (!(scheduling_stream >> candidate_scheduling_rng)) {
    return false;
  }
  std::istringstream tie_break_stream(state.tie_break_rng_state);
  std::mt19937 candidate_tie_break_rng;
  if (!(tie_break_stream >> candidate_tie_break_rng)) {
    return false;
  }
  std::istringstream deception_stream(state.deception_rng_state);
  std::mt19937 candidate_deception_rng;
  if (!(deception_stream >> candidate_deception_rng)) {
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
  impl_->scheduling_rng = candidate_scheduling_rng;
  impl_->tie_break_rng = candidate_tie_break_rng;
  impl_->deception_rng = candidate_deception_rng;
  impl_->deception_states = state.deception_states;
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
