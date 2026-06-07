#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/utils/MathUtils.h"
#include "common/timing/TimingRegimeModel.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using ControlProfileEffectsConfig =
  ::airborne_radar::config::execution::ControlProfileEffectsConfig;
using JammingEffectsConfig = ::airborne_radar::config::execution::JammingEffectsConfig;

namespace {

constexpr float kImmNoiseCoeffMin = 0.001f;
constexpr float kSpeedDecayRatioMax = 0.995f;
constexpr float kRcsDecayRatioMax = 0.999f;

constexpr float kLpiPowerKalmanNoiseScale = 1.15f;
constexpr float kLpiPowerAssignCostScale = 0.90f;

constexpr float kAgilityFreqAssignCostScale = 1.25f;
constexpr float kAgilityFreqKalmanDiffScale = 1.10f;

constexpr float kRejitterAssignCostScale = 1.35f;
constexpr float kRejitterKalmanDiffScale = 1.10f;

constexpr float kBurnthroughAssignCostMax = 2.0f;
constexpr float kBurnthroughKalmanNoiseScale = 0.90f;

constexpr float kSidelobeAssignCostScale = 1.10f;

constexpr float kAdaptiveBeamAssignCostScale = 1.10f;
constexpr float kAdaptiveBeamKalmanNoiseScale = 0.80f;

constexpr float kLpiBeamKalmanNoiseScale = 0.90f;

constexpr float kImmWeightBonusNormal = 0.10f;
constexpr float kImmWeightBonusBurnthrough = 0.18f;

constexpr float kImmAgilityNoiseScale = 1.15f;
constexpr float kImmRejitterNoiseScale = 1.20f;

float ClampProfileScale(float scale, float fallback) {
  if (!std::isfinite(scale) || scale <= 0.0f) {
    return fallback;
  }
  return scale;
}

void NormalizeImmInitialWeights(std::vector<float>* weights) {
  if (weights == nullptr || weights->empty()) {
    return;
  }

  float sum = 0.0f;
  for (std::size_t i = 0; i < weights->size(); ++i) {
    (*weights)[i] = utils::ClampFloat((*weights)[i], 0.0f, 1.0f);
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

oneq::internal::timing::ResolvedCycleTimingState ResolveDetectionTimingState(
    const extension::control::RadarControlProfile& control_profile,
    const config::engineering::DetectionConfig& detection_config) {
  oneq::internal::timing::CycleTimingBaseParams base_params;
  base_params.base_pulse_count = detection_config.pulse_count;
  base_params.base_prf_hz = detection_config.transmitter.prf_hz;
  base_params.integration_mode = oneq::internal::timing::IntegrationMode::kCoherent;

  oneq::internal::timing::CycleTimingControlAdjustments adjustments;
  adjustments.dwell_scale = control_profile.lpi_dwell_scale;
  adjustments.enable_rejitter = control_profile.enable_eccm_rejitter;
  return oneq::internal::timing::ResolveCycleTimingState(base_params, adjustments);
}

float ResolveBeamwidthScale(const ControlProfileEffectsConfig& cfg,
                            const extension::control::RadarControlProfile& control_profile) {
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

float ComputeHeuristicSignalAdjustmentDb(
    const ControlProfileEffectsConfig& cfg,
    const extension::control::RadarControlProfile& control_profile) {
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
    const JammingEffectsConfig& cfg, const extension::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot) {
  if (!HasMultiSourceJammingFacts(environment_snapshot)) {
    return 0.0f;
  }

  float relief_db = 0.0f;
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    const environment::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
    const float residual_factor = ComputeResidualJammerFactor(control_profile, source);
    relief_db += ComputeHeuristicSourcePenaltyDb(cfg, source) * (1.0f - residual_factor);
  }
  return relief_db;
}

void ApplyControlProfileToConfig(const extension::control::RadarControlProfile& control_profile,
                                 ExecutionConfig* config) {
  if (config == nullptr) {
    return;
  }
  const oneq::internal::timing::ResolvedCycleTimingState timing_state =
      ResolveDetectionTimingState(control_profile, config->detection.engineering);

  const ControlProfileEffectsConfig& cfg = config->control_profile_effects;

  if (control_profile.enable_lpi_power_control) {
    config->detection.engineering.transmitter.peak_power_w *=
        ClampProfileScale(control_profile.lpi_power_scale, 1.0f);
    config->tracking.engineering.kalman_measurement_noise_std *= kLpiPowerKalmanNoiseScale;
    config->association.unassigned_cost *= kLpiPowerAssignCostScale;
  }

  if (control_profile.lpi_dwell_scale != 1.0f) {
    config->detection.engineering.pulse_count =
        static_cast<int>(timing_state.effective_pulse_count);
  }

  if (control_profile.enable_agility_frequency) {
    const float hop_factor =
        (control_profile.agility_frequency_hop_phase % 2U == 0U) ? 1.015f : 0.985f;
    config->detection.engineering.transmitter.frequency_hz *= hop_factor;
    config->association.unassigned_cost *= kAgilityFreqAssignCostScale;
    config->tracking.kalman_noise_diff_coeff *= kAgilityFreqKalmanDiffScale;
  }

  if (control_profile.enable_eccm_rejitter) {
    config->detection.engineering.transmitter.prf_hz = timing_state.effective_prf_hz;
    config->association.unassigned_cost *= kRejitterAssignCostScale;
    config->tracking.kalman_noise_diff_coeff *= kRejitterKalmanDiffScale;
  }

  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    const float gain_db = ToDbDelta(control_profile.eccm_burnthrough_gain);
    config->detection.engineering.receiver.noise_figure_db =
        std::max(0.0f, config->detection.engineering.receiver.noise_figure_db - gain_db);
    config->association.unassigned_cost *=
        utils::ClampFloat(control_profile.eccm_burnthrough_gain, 1.0f, kBurnthroughAssignCostMax);
    config->tracking.engineering.kalman_measurement_noise_std *= kBurnthroughKalmanNoiseScale;
  }

  if (control_profile.enable_sidelobe_canceller) {
    config->detection.engineering.antenna.enable_directional_pattern = true;
    config->detection.engineering.antenna.pattern.max_sidelobe_level_db -=
        cfg.sidelobe_level_reduction_db;
    config->association.unassigned_cost *= kSidelobeAssignCostScale;
  }

  const float beamwidth_scale = ResolveBeamwidthScale(cfg, control_profile);
  if (beamwidth_scale < 0.999f) {
    config->detection.orientation.commanded_beamwidth_enabled = true;
    config->detection.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg = std::max(
        0.5f, config->detection.engineering.antenna.nominal_az_beamwidth_deg * beamwidth_scale);
    config->detection.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg = std::max(
        0.5f, config->detection.engineering.antenna.nominal_el_beamwidth_deg * beamwidth_scale);
  }

  if (control_profile.enable_adaptive_beamforming) {
    config->detection.engineering.antenna.main_beam_gain_db += cfg.adaptive_beam_gain_boost_db;
    config->association.unassigned_cost *= kAdaptiveBeamAssignCostScale;
    config->tracking.engineering.kalman_measurement_noise_std *= kAdaptiveBeamKalmanNoiseScale;
  }

  if (control_profile.enable_lpi_beamforming) {
    config->tracking.engineering.kalman_measurement_noise_std *= kLpiBeamKalmanNoiseScale;
  }

  if (control_profile.enable_sidelobe_canceller || control_profile.enable_agility_frequency ||
      control_profile.enable_eccm_rejitter || control_profile.eccm_burnthrough_gain > 1.0f) {
    config->tracking.engineering.speed_decay_ratio_on_loss =
        utils::ClampFloat(config->tracking.engineering.speed_decay_ratio_on_loss + cfg.eccm_speed_decay_bonus,
                          0.0f, kSpeedDecayRatioMax);
    config->tracking.engineering.rcs_decay_ratio_on_loss =
        utils::ClampFloat(config->tracking.engineering.rcs_decay_ratio_on_loss + cfg.eccm_rcs_decay_bonus, 0.0f,
                          kRcsDecayRatioMax);
  }

  if (!config->lifecycle.imm_model_noise_diff_coeffs.empty()) {
    float imm_noise_scale = 1.0f;
    if (control_profile.enable_agility_frequency) {
      imm_noise_scale *= kImmAgilityNoiseScale;
    }
    if (control_profile.enable_eccm_rejitter) {
      imm_noise_scale *= kImmRejitterNoiseScale;
    }
    if (control_profile.eccm_burnthrough_gain > 1.0f) {
      imm_noise_scale *=
          utils::ClampFloat(control_profile.eccm_burnthrough_gain, 1.0f, kBurnthroughAssignCostMax);
    }
    for (std::size_t i = 0; i < config->lifecycle.imm_model_noise_diff_coeffs.size(); ++i) {
      config->lifecycle.imm_model_noise_diff_coeffs[i] =
          std::max(kImmNoiseCoeffMin, config->lifecycle.imm_model_noise_diff_coeffs[i] * imm_noise_scale);
    }
  }

  if (!config->lifecycle.imm_initial_weights.empty() && config->lifecycle.imm_initial_weights.size() > 1U &&
      (control_profile.enable_agility_frequency || control_profile.enable_eccm_rejitter ||
       control_profile.eccm_burnthrough_gain > 1.0f)) {
    const std::size_t last_index = config->lifecycle.imm_initial_weights.size() - 1U;
    const float bonus = control_profile.eccm_burnthrough_gain > 1.0f ? kImmWeightBonusBurnthrough
                                                                     : kImmWeightBonusNormal;
    config->lifecycle.imm_initial_weights[last_index] += bonus;
    NormalizeImmInitialWeights(&config->lifecycle.imm_initial_weights);
  }
}


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
