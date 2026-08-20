// @file SarSessionConfigBuilder.cpp
// @brief Implementation of SarSessionConfigBuilder (thin wrapper).

#include "1q/sar/config/SarSessionConfigBuilder.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

#include "1q/sar/config/SarSessionConfigValidation.h"
#include "1q/sar/session/SarIssueCodes.h"

namespace sar {
namespace config {

config::SarSessionConfig SarSessionConfigBuilder::Build() const noexcept { return config_; }

session::SarIssueList ValidateSarSessionConfig(const config::SarSessionConfig& config) noexcept {
  session::SarIssueList issues;
  const auto push = [&issues](const char* code, const char* field, const char* msg) {
    session::SarIssue issue;
    issue.severity = session::SarIssueSeverity::kError;
    issue.phase = session::SarIssuePhase::kInputValidation;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(std::move(issue));
  };

  if (config.hardware.carrier_frequency_hz <= 0.0) {
    push(session::codes::kCarrierFrequencyNotPositive,
         "hardware.carrier_frequency_hz", "Carrier frequency must be positive.");
  }
  if (config.hardware.bandwidth_hz <= 0.0) {
    push(session::codes::kBandwidthNotPositive,
         "hardware.bandwidth_hz", "Bandwidth must be positive.");
  }
  if (config.hardware.pulse_repetition_frequency_hz <= 0.0) {
    push(session::codes::kPulseRepetitionFrequencyNotPositive,
         "hardware.pulse_repetition_frequency_hz", "Pulse repetition frequency must be positive.");
  }
  if (config.hardware.sample_rate_hz <= 0.0) {
    push(session::codes::kSampleRateNotPositive,
         "hardware.sample_rate_hz", "Sample rate must be positive.");
  }
  if (config.hardware.antenna_length_m <= 0.0) {
    push(session::codes::kAntennaLengthNotPositive,
         "hardware.antenna_length_m", "Antenna length must be positive.");
  }
  if (!std::isfinite(config.hardware.peak_power_w) || config.hardware.peak_power_w <= 0.0 ||
      !std::isfinite(config.hardware.antenna_gain_db) ||
      !std::isfinite(config.hardware.receiver_noise_figure_db) ||
      config.hardware.receiver_noise_figure_db < 0.0 ||
      !std::isfinite(config.hardware.system_loss_db) || config.hardware.system_loss_db < 0.0) {
    push(session::codes::kHardwareLinkBudgetInvalid,
         "hardware.peak_power_w / antenna_gain_db / receiver_noise_figure_db / system_loss_db",
         "Hardware link-budget fields must be finite; power must be positive and noise figure / "
         "loss must be non-negative.");
  }
  if (config.mission.nominal_slant_range_m <= 0.0) {
    push(session::codes::kNominalSlantRangeNotPositive,
         "mission.nominal_slant_range_m", "Nominal slant range must be positive.");
  }
  if (config.mission.platform_speed_mps <= 0.0) {
    push(session::codes::kPlatformSpeedNotPositive,
         "mission.platform_speed_mps", "Platform speed must be positive.");
  }
  if (config.mission.azimuth_pulse_count == 0U) {
    push(session::codes::kAzimuthPulseCountZero,
         "mission.azimuth_pulse_count", "Azimuth pulse count must be non-zero.");
  }
  if (config.mission.range_sample_count == 0U) {
    push(session::codes::kRangeSampleCountZero,
         "mission.range_sample_count", "Range sample count must be non-zero.");
  }
  if (config.mission.desired_ground_range_resolution_m <= 0.0) {
    push(session::codes::kDesiredResolutionNotPositive,
         "mission.desired_ground_range_resolution_m",
         "Desired ground range resolution must be positive.");
  }
  if (config.mission.desired_azimuth_resolution_m <= 0.0) {
    push(session::codes::kDesiredResolutionNotPositive,
         "mission.desired_azimuth_resolution_m",
         "Desired azimuth resolution must be positive.");
  }
  if (config.policy.retain_raw_phase_history &&
      !config.policy.enable_raw_echo_generation) {
    push(session::codes::kRetainRawHistoryRequiresRawEcho,
         "policy.retain_raw_phase_history / enable_raw_echo_generation",
         "Retaining raw phase history requires raw echo generation.");
  }
  if (!std::isfinite(config.policy.max_allowed_squint_angle_deg) ||
      config.policy.max_allowed_squint_angle_deg < 0.0 ||
      config.policy.max_allowed_squint_angle_deg >= 90.0) {
    push(session::codes::kSquintAngleInvalid,
         "policy.max_allowed_squint_angle_deg",
         "Maximum allowed squint angle must be finite and in [0, 90) degrees.");
  }
  if (!std::isfinite(config.environment.terrain_reference_altitude_m) ||
      !std::isfinite(config.environment.atmospheric_loss_db_per_km) ||
      config.environment.atmospheric_loss_db_per_km < 0.0 ||
      !std::isfinite(config.environment.surface_backscatter_sigma0_db)) {
    push(session::codes::kEnvironmentConfigInvalid,
         "environment.terrain_reference_altitude_m / atmospheric_loss_db_per_km / "
         "surface_backscatter_sigma0_db",
         "Environment scalar fields must be finite and atmospheric loss must be non-negative.");
  }

  // 跨字段物理约束：距离采样窗口必须能容纳完整 LFM 脉冲宽度（与 SarWaveform.cpp:68 的
  // 波形样本数公式 ceil(pulse_width_s * sample_rate_hz) 一致）。窗口过小会导致回波被
  // 裁剪、成像失败。
  if (config.hardware.pulse_width_s > 0.0 && config.hardware.sample_rate_hz > 0.0) {
    const std::size_t waveform_samples = static_cast<std::size_t>(
        std::ceil(config.hardware.pulse_width_s * config.hardware.sample_rate_hz));
    if (waveform_samples > config.mission.range_sample_count) {
      push(session::codes::kSampleWindowTooSmallForPulse,
           "mission.range_sample_count",
           "Range sample window cannot hold the full LFM pulse width.");
    }
  }

  return issues;
}

}  // namespace config
}  // namespace sar
