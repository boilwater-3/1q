#include "1q/airborne_radar/common/RadarSessionConfigBuilder.h"

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace common {

core::session::RadarSessionConfig RadarSessionConfigBuilder::Build() const {
  const auto& tx =
      config_.signal_pipeline_config.detection.radar_system.transmitter;
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
  return config_;
}

}  // namespace common
}  // namespace airborne_radar
