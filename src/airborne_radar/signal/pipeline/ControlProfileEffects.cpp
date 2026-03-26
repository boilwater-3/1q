#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "common/timing/TimingRegimeModel.h"
#include "1q/airborne_radar/common/MathUtils.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

// 数值稳定性约束（非用户可调，保持为内部常量）
constexpr float kImmNoiseCoeffMin  = 0.001f; /**< IMM 噪声协方差系数下界（防止退化）*/
constexpr float kSpeedDecayRatioMax = 0.995f; /**< 速度衰减系数上界 */
constexpr float kRcsDecayRatioMax   = 0.999f; /**< RCS 衰减系数上界 */

float ClampProfileScale(float scale, float fallback) {
  if (!std::isfinite(scale) || scale <= 0.0f) {
    return fallback;
  }
  return scale;
}

using common::ClampFloat;

void NormalizeImmInitialWeights(std::vector<float>* weights) {
  if (weights == nullptr || weights->empty()) {
    return;
  }

  float sum = 0.0f;
  for (std::size_t i = 0; i < weights->size(); ++i) {
    (*weights)[i] = ClampFloat((*weights)[i], 0.0f, 1.0f);
    sum += (*weights)[i];
  }

  if (sum <= 1e-6f) {
    const float uniform_weight = 1.0f / static_cast<float>(weights->size());
    for (std::size_t i = 0; i < weights->size(); ++i) {
      (*weights)[i] = uniform_weight;
    }
    return;
  }

  for (std::size_t i = 0; i < weights->size(); ++i) {
    (*weights)[i] /= sum;
  }
}

float ToDbDelta(float linear_scale) {
  const float clamped_scale = ClampProfileScale(linear_scale, 1.0f);
  return 10.0f * std::log10(clamped_scale);
}

oneq::internal::timing::IntegrationMode ToTimingIntegrationMode(bool coherent_integration) {
  if (coherent_integration) {
    return oneq::internal::timing::IntegrationMode::kCoherent;
  }
  return oneq::internal::timing::IntegrationMode::kNonCoherent;
}

oneq::internal::timing::ResolvedCycleTimingState ResolveDetectionTimingState(
    const common::RadarControlProfile& control_profile,
    const SignalDetectionConfig& detection_config) {
  oneq::internal::timing::CycleTimingBaseParams base_params;
  base_params.base_pulse_count = detection_config.pulse_count;
  base_params.base_prf_hz = detection_config.radar_system.transmitter.prf_hz;
  base_params.integration_mode = ToTimingIntegrationMode(detection_config.coherent_integration);

  oneq::internal::timing::CycleTimingControlAdjustments adjustments;
  adjustments.dwell_scale = control_profile.lpi_dwell_scale;
  adjustments.enable_rejitter = control_profile.enable_eccm_rejitter;
  return oneq::internal::timing::ResolveCycleTimingState(base_params, adjustments);
}

float ResolveBeamwidthScale(const ControlProfileEffectsConfig& cfg,
                            const common::RadarControlProfile& control_profile) {
  float beamwidth_scale = 1.0f;
  if (control_profile.enable_lpi_beamforming) {
    beamwidth_scale = std::min(beamwidth_scale, cfg.lpi_beamwidth_scale);
  }
  if (control_profile.enable_adaptive_beamforming) {
    beamwidth_scale = std::min(beamwidth_scale, cfg.adaptive_beamwidth_scale);
  }
  return beamwidth_scale;
}

}  // namespace

float ComputeHeuristicSignalAdjustmentDb(const ControlProfileEffectsConfig& cfg,
                                         const common::RadarControlProfile& control_profile) {
  float adjustment_db = 0.0f;
  if (control_profile.enable_lpi_power_control) {
    adjustment_db += ToDbDelta(control_profile.lpi_power_scale);
  }
  if (control_profile.enable_lpi_beamforming) {
    adjustment_db += cfg.lpi_beam_signal_gain_db;
  }
  if (control_profile.enable_adaptive_beamforming) {
    adjustment_db += cfg.adaptive_beam_signal_gain_db;
  }
  return adjustment_db;
}

float ComputeHeuristicEnvironmentReliefDb(
    const JammingEffectsConfig& cfg, const common::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot) {
  if (HasMultiSourceJammingFacts(environment_snapshot)) {
    float relief_db = 0.0f;
    for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
      const environment::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
      const float residual_factor = ComputeResidualJammerFactor(control_profile, source);
      relief_db += ComputeHeuristicSourcePenaltyDb(cfg, source) * (1.0f - residual_factor);
    }
    return relief_db;
  }

  float relief_db = 0.0f;
  if (environment_snapshot.jamming_detected) {
    if (control_profile.enable_sidelobe_canceller) {
      relief_db += environment_snapshot.jammer_in_sidelobe ? 3.0f : 0.8f;
    }
    if (control_profile.enable_agility_frequency) {
      relief_db +=
          0.6f + 1.8f * ClampFloat(environment_snapshot.jammer_frequency_overlap_ratio, 0.0f, 1.0f);
    }
    if (control_profile.enable_eccm_rejitter) {
      relief_db += 0.4f + 1.4f * ClampFloat(environment_snapshot.jammer_prf_lock_risk, 0.0f, 1.0f);
    }
  }
  if (control_profile.enable_adaptive_beamforming) {
    relief_db += environment_snapshot.jammer_in_sidelobe ? 1.2f : 0.8f;
  }
  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    const float jammer_scale =
        0.5f + 0.06f * ClampFloat(environment_snapshot.jammer_power_db, 0.0f, 20.0f);
    relief_db += ToDbDelta(control_profile.eccm_burnthrough_gain) * ClampFloat(jammer_scale, 0.5f, 1.7f);
  }
  return relief_db;
}

void ApplyControlProfileToConfig(const common::RadarControlProfile& control_profile,
                                 SignalPipelineConfig* runtime_config) {
  if (runtime_config == nullptr) {
    return;
  }
  const oneq::internal::timing::ResolvedCycleTimingState timing_state =
      ResolveDetectionTimingState(control_profile, runtime_config->detection);

  const ControlProfileEffectsConfig& cfg = runtime_config->control_profile_effects;

  if (control_profile.enable_lpi_power_control) {
    runtime_config->detection.radar_system.transmitter.peak_power_w *=
        ClampProfileScale(control_profile.lpi_power_scale, 1.0f);
    runtime_config->tracking.kalman_measurement_noise_std *= cfg.lpi_power_kalman_noise_scale;
    runtime_config->association.unassigned_cost *= cfg.lpi_power_assign_cost_scale;
  }

  if (control_profile.lpi_dwell_scale != 1.0f) {
    runtime_config->detection.pulse_count = static_cast<int>(timing_state.effective_pulse_count);
  }

  if (control_profile.enable_agility_frequency) {
    const float hop_factor = (control_profile.version % 2U == 0U) ? 1.015f : 0.985f;
    runtime_config->detection.radar_system.transmitter.frequency_hz *= hop_factor;
    runtime_config->association.unassigned_cost *= cfg.agility_freq_assign_cost_scale;
    runtime_config->tracking.kalman_noise_diff_coeff *= cfg.agility_freq_kalman_diff_scale;
  }

  if (control_profile.enable_eccm_rejitter) {
    runtime_config->detection.radar_system.transmitter.prf_hz = timing_state.effective_prf_hz;
    runtime_config->association.unassigned_cost *= cfg.rejitter_assign_cost_scale;
    runtime_config->tracking.kalman_noise_diff_coeff *= cfg.rejitter_kalman_diff_scale;
  }

  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    const float gain_db = ToDbDelta(control_profile.eccm_burnthrough_gain);
    runtime_config->detection.radar_system.receiver.noise_figure_db =
        std::max(0.0f, runtime_config->detection.radar_system.receiver.noise_figure_db - gain_db);
    runtime_config->association.unassigned_cost *=
        ClampFloat(control_profile.eccm_burnthrough_gain, 1.0f, cfg.burnthrough_assign_cost_max);
    runtime_config->tracking.kalman_measurement_noise_std *= cfg.burnthrough_kalman_noise_scale;
  }

  if (control_profile.enable_sidelobe_canceller) {
    runtime_config->detection.radar_system.antenna.enable_directional_pattern = true;
    runtime_config->detection.radar_system.antenna.pattern.max_sidelobe_level_db -= cfg.sidelobe_level_reduction_db;
    runtime_config->association.unassigned_cost *= cfg.sidelobe_assign_cost_scale;
  }

  const float beamwidth_scale = ResolveBeamwidthScale(cfg, control_profile);
  if (beamwidth_scale < 0.999f) {
    runtime_config->beam_control.radar_orientation.commanded_beamwidth_enabled = true;
    runtime_config->beam_control.radar_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg =
        std::max(0.5f, runtime_config->detection.radar_system.antenna.nominal_az_beamwidth_deg * beamwidth_scale);
    runtime_config->beam_control.radar_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
        std::max(0.5f, runtime_config->detection.radar_system.antenna.nominal_el_beamwidth_deg * beamwidth_scale);
  }

  if (control_profile.enable_adaptive_beamforming) {
    runtime_config->detection.radar_system.antenna.main_beam_gain_db += cfg.adaptive_beam_gain_boost_db;
    runtime_config->association.unassigned_cost *= cfg.adaptive_beam_assign_cost_scale;
    runtime_config->tracking.kalman_measurement_noise_std *= cfg.adaptive_beam_kalman_noise_scale;
  }

  if (control_profile.enable_lpi_beamforming) {
    runtime_config->tracking.kalman_measurement_noise_std *= cfg.lpi_beam_kalman_noise_scale;
  }

  if (control_profile.enable_sidelobe_canceller || control_profile.enable_agility_frequency ||
      control_profile.enable_eccm_rejitter || control_profile.eccm_burnthrough_gain > 1.0f) {
    runtime_config->tracking.speed_decay_ratio_on_loss = ClampFloat(
        runtime_config->tracking.speed_decay_ratio_on_loss + cfg.eccm_speed_decay_bonus, 0.0f,
        kSpeedDecayRatioMax);
    runtime_config->tracking.rcs_decay_ratio_on_loss = ClampFloat(
        runtime_config->tracking.rcs_decay_ratio_on_loss + cfg.eccm_rcs_decay_bonus, 0.0f,
        kRcsDecayRatioMax);
  }

  if (!runtime_config->lifecycle.imm_model_noise_diff_coeffs.empty()) {
    float imm_noise_scale = 1.0f;
    if (control_profile.enable_agility_frequency) {
      imm_noise_scale *= cfg.imm_agility_noise_scale;
    }
    if (control_profile.enable_eccm_rejitter) {
      imm_noise_scale *= cfg.imm_rejitter_noise_scale;
    }
    if (control_profile.eccm_burnthrough_gain > 1.0f) {
      imm_noise_scale *= ClampFloat(control_profile.eccm_burnthrough_gain, 1.0f, cfg.burnthrough_assign_cost_max);
    }
    for (std::size_t i = 0; i < runtime_config->lifecycle.imm_model_noise_diff_coeffs.size(); ++i) {
      runtime_config->lifecycle.imm_model_noise_diff_coeffs[i] =
          std::max(kImmNoiseCoeffMin,
                   runtime_config->lifecycle.imm_model_noise_diff_coeffs[i] * imm_noise_scale);
    }
  }

  if (!runtime_config->lifecycle.imm_initial_weights.empty() &&
      runtime_config->lifecycle.imm_initial_weights.size() > 1U &&
      (control_profile.enable_agility_frequency || control_profile.enable_eccm_rejitter ||
       control_profile.eccm_burnthrough_gain > 1.0f)) {
    const std::size_t last_index = runtime_config->lifecycle.imm_initial_weights.size() - 1U;
    const float bonus = control_profile.eccm_burnthrough_gain > 1.0f ? cfg.imm_weight_bonus_burnthrough
                                                                      : cfg.imm_weight_bonus_normal;
    runtime_config->lifecycle.imm_initial_weights[last_index] += bonus;
    NormalizeImmInitialWeights(&runtime_config->lifecycle.imm_initial_weights);
  }
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
