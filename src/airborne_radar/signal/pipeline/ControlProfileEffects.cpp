#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/utils/MathUtils.h"
#include "common/timing/TimingRegimeModel.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using ControlProfileEffectsConfig =
    ::airborne_radar::config::execution::ControlProfileEffectsConfig;

namespace {

float ClampProfileScale(float scale, float fallback) {
  if (!std::isfinite(scale) || scale <= 0.0f) {
    return fallback;
  }
  return scale;
}

oneq::common::timing::ResolvedCycleTimingState ResolveDetectionTimingState(
    const session::ArControlProfile& control_profile,
    const config::engineering::DetectionConfig& detection_config) {
  oneq::common::timing::CycleTimingBaseParams base_params;
  base_params.base_pulse_count = detection_config.pulse_count;
  base_params.base_prf_hz = detection_config.transmitter.prf_hz;
  base_params.integration_mode = oneq::common::timing::IntegrationMode::kCoherent;

  oneq::common::timing::CycleTimingControlAdjustments adjustments;
  adjustments.dwell_scale = control_profile.lpi_dwell_scale;
  adjustments.enable_rejitter = control_profile.enable_eccm_rejitter;
  return oneq::common::timing::ResolveCycleTimingState(base_params, adjustments);
}

float ResolveBeamwidthScale(const ControlProfileEffectsConfig& cfg,
                            const session::ArControlProfile& control_profile) {
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

void ApplyControlProfileToConfig(const session::ArControlProfile& control_profile,
                                 ExecutionConfig* config) {
  if (config == nullptr) {
    return;
  }
  const oneq::common::timing::ResolvedCycleTimingState timing_state =
      ResolveDetectionTimingState(control_profile, config->detection.engineering);

  const ControlProfileEffectsConfig& cfg = config->control_profile_effects;

  if (control_profile.enable_lpi_power_control) {
    config->detection.engineering.transmitter.peak_power_w *=
        ClampProfileScale(control_profile.lpi_power_scale, 1.0f);
  }

  if (control_profile.lpi_dwell_scale != 1.0f) {
    config->detection.engineering.pulse_count =
        static_cast<int>(timing_state.effective_pulse_count);
  }

  if (control_profile.enable_agility_frequency) {
    const float hop_factor =
        (control_profile.agility_frequency_hop_phase % 2U == 0U) ? 1.015f : 0.985f;
    config->detection.engineering.transmitter.frequency_hz *= hop_factor;
  }

  if (control_profile.enable_eccm_rejitter) {
    config->detection.engineering.transmitter.prf_hz = timing_state.effective_prf_hz;
  }

  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    config->detection.engineering.transmitter.peak_power_w *=
        ClampProfileScale(control_profile.eccm_burnthrough_gain, 1.0f);
  }

  if (control_profile.enable_sidelobe_canceller) {
    config->detection.engineering.antenna.enable_directional_pattern = true;
    config->detection.engineering.antenna.pattern.max_sidelobe_level_db -=
        cfg.sidelobe_level_reduction_db;
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
  }

  config->enable_anti_vgpo_acceleration_bound =
      control_profile.enable_anti_vgpo_acceleration_bound;
  config->enable_anti_false_target_discrimination =
      control_profile.enable_anti_false_target_discrimination;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
