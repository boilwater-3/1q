#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

#include <cmath>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace config {

namespace {

bool IsFinitePositive(float value) { return std::isfinite(value) && value > 0.0f; }

void WarnInvalidPhysicsDetectionConfig(const config::SignalDetectionConfig& detection) {
  if (!detection.enable_physics_detection) {
    return;
  }

  const config::TransmitterConfig& tx = detection.transmitter;
  if (!IsFinitePositive(tx.peak_power_w)) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] physics detection enabled but peak_power_w={} is invalid.",
        tx.peak_power_w);
  }
  if (!IsFinitePositive(tx.pulse_width_s)) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] physics detection enabled but pulse_width_s={} is invalid.",
        tx.pulse_width_s);
  }
  if (!IsFinitePositive(tx.bandwidth_hz)) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] physics detection enabled but bandwidth_hz={} is invalid.",
        tx.bandwidth_hz);
  }
  if (!IsFinitePositive(tx.prf_hz)) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] physics detection enabled but prf_hz={} is invalid.",
        tx.prf_hz);
  }
  if (detection.pulse_count < 1) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] physics detection enabled but pulse_count={} is invalid.",
        detection.pulse_count);
  }
  const float duty_cycle = tx.prf_hz * tx.pulse_width_s;
  if (!std::isfinite(duty_cycle) || duty_cycle > 1.0f) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] physics detection enabled but duty_cycle(prf_hz*pulse_width_s)={} "
        "is invalid (>1 or non-finite).",
        duty_cycle);
  }
}

}  // namespace

session::RadarSessionConfig RadarSessionConfigBuilder::Build() const {
  const auto& tx = config_.detection.transmitter;
  if (tx.peak_power_w <= 0.0f) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] peak_power_w={} is non-positive; "
        "physics detection results will be invalid.",
        tx.peak_power_w);
  }
  if (tx.frequency_hz <= 0.0f) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] frequency_hz={} is non-positive; "
        "radar equation calculations will be invalid.",
        tx.frequency_hz);
  }
  if (tx.bandwidth_hz <= 0.0f) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] bandwidth_hz={} is non-positive; "
        "range resolution will be invalid.",
        tx.bandwidth_hz);
  }
  const float nf_db = config_.detection.receiver.noise_figure_db;
  if (nf_db < 0.0f) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] noise_figure_db={} is negative; "
        "receiver noise model will be invalid.",
        nf_db);
  }

  const auto& rcs = config_.detection.rcs_physics;
  if (rcs.enable_physical_rcs) {
    if (rcs.physics_mix_ratio < 0.0f || rcs.physics_mix_ratio > 1.0f) {
      PROJECT_LOG_WARN(
          "[RadarSessionConfigBuilder] rcs_physics.physics_mix_ratio={} is outside [0,1]; "
          "runtime will clamp internally.",
          rcs.physics_mix_ratio);
    }
    if (rcs.cylinder_weight < 0.0f || rcs.cylinder_weight > 1.0f) {
      PROJECT_LOG_WARN(
          "[RadarSessionConfigBuilder] rcs_physics.cylinder_weight={} is outside [0,1]; "
          "runtime will clamp internally.",
          rcs.cylinder_weight);
    }
    if (rcs.min_equivalent_radius_m <= 0.0f || rcs.max_equivalent_radius_m <= 0.0f ||
        rcs.max_equivalent_radius_m < rcs.min_equivalent_radius_m) {
      PROJECT_LOG_WARN(
          "[RadarSessionConfigBuilder] rcs_physics equivalent radius range [{}, {}] is invalid; "
          "runtime will clamp/reorder internally.",
          rcs.min_equivalent_radius_m, rcs.max_equivalent_radius_m);
    }
    if (rcs.min_rcs_m2 < 0.0f || rcs.max_rcs_m2 < 0.0f || rcs.max_rcs_m2 < rcs.min_rcs_m2) {
      PROJECT_LOG_WARN(
          "[RadarSessionConfigBuilder] rcs_physics output range [{}, {}] is invalid; "
          "runtime will clamp/reorder internally.",
          rcs.min_rcs_m2, rcs.max_rcs_m2);
    }
    if (rcs.frequency_hz <= 0.0f && tx.frequency_hz <= 0.0f) {
      PROJECT_LOG_WARN(
          "[RadarSessionConfigBuilder] both rcs_physics.frequency_hz={} and "
          "transmitter.frequency_hz={} are non-positive; physical RCS model will be bypassed.",
          rcs.frequency_hz, tx.frequency_hz);
    }
  }

  const auto& detection_policy = config_.detection.detection_policy;
  WarnInvalidPhysicsDetectionConfig(config_.detection);
  if (!config_.detection.enable_physics_detection) {
    constexpr float kDefaultCfarPfa = 1e-6f;
    constexpr float kDefaultMinSnrDb = -10.0f;
    if (std::fabs(detection_policy.cfar_pfa - kDefaultCfarPfa) > 1.0e-7f ||
        std::fabs(detection_policy.min_snr_db - kDefaultMinSnrDb) > 1.0e-5f) {
      PROJECT_LOG_WARN(
          "[RadarSessionConfigBuilder] cfar_pfa/min_snr_db configured while "
          "enable_physics_detection=false; these parameters have no effect "
          "in the heuristic detection path.");
    }
  }

  return config_;
}

}  // namespace config
}  // namespace airborne_radar
