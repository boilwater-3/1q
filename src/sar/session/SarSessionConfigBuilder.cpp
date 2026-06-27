// @file SarSessionConfigBuilder.cpp
// @brief Implementation of SarSessionConfigBuilder with profile translation.

#include "1q/sar/config/SarSessionConfigBuilder.h"

#include "1q/sar/config/SarSessionConfigValidation.h"

namespace sar {
namespace config {

namespace {

void ApplySarMissionSemanticConfig(SarMissionProfile profile, SarMissionConfig* mission) {
  if (mission == nullptr) {
    return;
  }
  auto& m = *mission;

  switch (profile) {
    case SarMissionProfile::kStripmapSurvey:
      m.nominal_slant_range_m = 15000.0;
      m.synthetic_aperture_time_s = 2.0;
      m.platform_speed_mps = 180.0;
      m.azimuth_pulse_count = 1024U;
      m.range_sample_count = 2048U;
      m.desired_ground_range_resolution_m = 1.5;
      m.desired_azimuth_resolution_m = 1.5;
      break;
    case SarMissionProfile::kHighResolutionImaging:
      m.nominal_slant_range_m = 10000.0;
      m.synthetic_aperture_time_s = 4.0;
      m.platform_speed_mps = 150.0;
      m.azimuth_pulse_count = 2048U;
      m.range_sample_count = 4096U;
      m.desired_ground_range_resolution_m = 0.5;
      m.desired_azimuth_resolution_m = 0.5;
      break;
    case SarMissionProfile::kLongRangeSurveillance:
      m.nominal_slant_range_m = 50000.0;
      m.synthetic_aperture_time_s = 3.0;
      m.platform_speed_mps = 200.0;
      m.azimuth_pulse_count = 512U;
      m.range_sample_count = 1024U;
      m.desired_ground_range_resolution_m = 3.0;
      m.desired_azimuth_resolution_m = 3.0;
      break;
  }
}

void ApplySarProcessingSemanticConfig(SarProcessingProfile profile, SarPolicyConfig* policy) {
  if (policy == nullptr) {
    return;
  }
  auto& p = *policy;

  switch (profile) {
    case SarProcessingProfile::kRawEchoOnly:
      p.enable_raw_echo_generation = true;
      p.enable_range_compression = false;
      p.enable_l1_rda_imaging = false;
      p.enable_l2_motion_compensation = false;
      p.enable_l3_bp_imaging = false;
      p.retain_focused_image = false;
      break;
    case SarProcessingProfile::kRangeCompressedL1:
      p.enable_raw_echo_generation = true;
      p.enable_range_compression = true;
      p.enable_l1_rda_imaging = true;
      p.enable_l2_motion_compensation = false;
      p.enable_l3_bp_imaging = false;
      p.retain_focused_image = true;
      break;
    case SarProcessingProfile::kFullPipelineL3:
      p.enable_raw_echo_generation = true;
      p.enable_range_compression = true;
      p.enable_l1_rda_imaging = false;
      p.enable_l2_motion_compensation = true;
      p.enable_l3_bp_imaging = true;
      p.retain_focused_image = true;
      break;
  }
}

}  // namespace

config::SarSessionConfig SarSessionConfigBuilder::Build() const noexcept {
  config::SarSessionConfig result = config_;

  if (mission_profile_dirty_) {
    ApplySarMissionSemanticConfig(mission_profile_, &result.mission);
  }
  if (processing_profile_dirty_) {
    ApplySarProcessingSemanticConfig(processing_profile_, &result.policy);
  }

  return result;
}

ValidationIssueList ValidateSarSessionConfig(const config::SarSessionConfig& config) noexcept {
  ValidationIssueList issues;
  const auto push = [&issues](ConfigValidationCode code, const char* field, const char* msg) {
    ConfigValidationIssue issue;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };

  if (config.hardware.carrier_frequency_hz <= 0.0) {
    push(ConfigValidationCode::kCarrierFrequencyNotPositive,
         "hardware.carrier_frequency_hz", "Carrier frequency must be positive.");
  }
  if (config.hardware.bandwidth_hz <= 0.0) {
    push(ConfigValidationCode::kBandwidthNotPositive,
         "hardware.bandwidth_hz", "Bandwidth must be positive.");
  }
  if (config.hardware.pulse_repetition_frequency_hz <= 0.0) {
    push(ConfigValidationCode::kPulseRepetitionFrequencyNotPositive,
         "hardware.pulse_repetition_frequency_hz", "Pulse repetition frequency must be positive.");
  }
  if (config.hardware.sample_rate_hz <= 0.0) {
    push(ConfigValidationCode::kSampleRateNotPositive,
         "hardware.sample_rate_hz", "Sample rate must be positive.");
  }
  if (config.hardware.antenna_length_m <= 0.0) {
    push(ConfigValidationCode::kAntennaLengthNotPositive,
         "hardware.antenna_length_m", "Antenna length must be positive.");
  }
  if (config.mission.nominal_slant_range_m <= 0.0) {
    push(ConfigValidationCode::kNominalSlantRangeNotPositive,
         "mission.nominal_slant_range_m", "Nominal slant range must be positive.");
  }
  if (config.mission.platform_speed_mps <= 0.0) {
    push(ConfigValidationCode::kPlatformSpeedNotPositive,
         "mission.platform_speed_mps", "Platform speed must be positive.");
  }
  if (config.mission.azimuth_pulse_count == 0U) {
    push(ConfigValidationCode::kAzimuthPulseCountZero,
         "mission.azimuth_pulse_count", "Azimuth pulse count must be non-zero.");
  }
  if (config.mission.range_sample_count == 0U) {
    push(ConfigValidationCode::kRangeSampleCountZero,
         "mission.range_sample_count", "Range sample count must be non-zero.");
  }
  if (config.mission.desired_ground_range_resolution_m <= 0.0) {
    push(ConfigValidationCode::kDesiredResolutionNotPositive,
         "mission.desired_ground_range_resolution_m",
         "Desired ground range resolution must be positive.");
  }
  if (config.mission.desired_azimuth_resolution_m <= 0.0) {
    push(ConfigValidationCode::kDesiredResolutionNotPositive,
         "mission.desired_azimuth_resolution_m",
         "Desired azimuth resolution must be positive.");
  }

  return issues;
}

}  // namespace config
}  // namespace sar
