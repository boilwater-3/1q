#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace common {
namespace config {

core::session::RadarSessionConfig RadarSessionConfigBuilder::Build() const {
  const auto& tx = config_.signal_pipeline_config.detection.radar_system.transmitter;
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
  const float nf_db =
      config_.signal_pipeline_config.detection.radar_system.receiver.noise_figure_db;
  if (nf_db < 0.0f) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] noise_figure_db={} is negative; "
        "receiver noise model will be invalid.",
        nf_db);
  }

  const auto& rcs = config_.signal_pipeline_config.detection.rcs_physics;
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
  return config_;
}

}  // namespace config
}  // namespace common
}  // namespace airborne_radar
