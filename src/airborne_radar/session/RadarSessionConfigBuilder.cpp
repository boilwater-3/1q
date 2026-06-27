#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

#include "1q/airborne_radar/config/RadarSessionConfigValidation.h"

namespace airborne_radar {
namespace config {

namespace {

void ApplyDetectionSemanticConfig(bool enable_physics_detection,
                                  profiles::RadarHardwareProfile hardware_profile,
                                  profiles::DetectionIntentProfile intent_profile,
                                  profiles::AntennaPatternProfile antenna_profile,
                                  const config::AzimuthElevationDeg& antenna_boresight_offset_deg,
                                  profiles::RcsFusionProfile rcs_fusion_profile,
                                  DetectionConfig* detection_config) {
  if (detection_config == nullptr) {
    return;
  }
  auto& d = *detection_config;
  d.enable_physics_detection = enable_physics_detection;

  switch (hardware_profile) {
    case profiles::RadarHardwareProfile::kLongRangeHighPower:
      d.transmitter.peak_power_w = 5.0e6f;
      d.transmitter.frequency_hz = 9.3e9f;
      d.transmitter.bandwidth_hz = 3.0e6f;
      d.transmitter.pulse_width_s = 18e-6f;
      d.transmitter.prf_hz = 220.0f;
      d.antenna.main_beam_gain_db = 38.0f;
      d.receiver.noise_figure_db = 3.0f;
      break;
    case profiles::RadarHardwareProfile::kLightweightLpi:
      d.transmitter.peak_power_w = 3.5e5f;
      d.transmitter.frequency_hz = 10.0e9f;
      d.transmitter.bandwidth_hz = 8.0e6f;
      d.transmitter.pulse_width_s = 8e-6f;
      d.transmitter.prf_hz = 600.0f;
      d.antenna.main_beam_gain_db = 31.0f;
      d.antenna.nominal_az_beamwidth_deg = 5.0f;
      d.antenna.nominal_el_beamwidth_deg = 5.0f;
      d.receiver.noise_figure_db = 5.0f;
      break;
    case profiles::RadarHardwareProfile::kGenericAirborneXBand:
    default:
      break;
  }

  switch (intent_profile) {
    case profiles::DetectionIntentProfile::kDetectionPriority:
      d.pulse_count = 16;
      d.detection_policy.cfar_pfa = 2e-6f;
      d.detection_policy.min_snr_db = -12.0f;
      d.min_detection_margin_db = -100.0f;
      break;
    case profiles::DetectionIntentProfile::kTrackStabilityPriority:
      d.pulse_count = 8;
      d.detection_policy.cfar_pfa = 5e-7f;
      d.detection_policy.min_snr_db = -8.0f;
      d.min_detection_margin_db = -20.0f;
      break;
    case profiles::DetectionIntentProfile::kBalanced:
    default:
      d.min_detection_margin_db = -2.0f;
      break;
  }

  switch (antenna_profile) {
    case profiles::AntennaPatternProfile::kLowSidelobe:
      d.antenna.pattern.max_sidelobe_level_db = -30.0f;
      d.antenna.pattern.backlobe_level_db = -42.0f;
      break;
    case profiles::AntennaPatternProfile::kWideCoverage:
      d.antenna.pattern.model_type = AntennaPatternModelType::kParabolicMainLobe;
      d.antenna.pattern.max_sidelobe_level_db = -18.0f;
      d.antenna.pattern.max_scan_loss_db = 8.0f;
      break;
    case profiles::AntennaPatternProfile::kStandard:
    default:
      break;
  }
  d.antenna.pattern.boresight_offset_deg = antenna_boresight_offset_deg;

  switch (rcs_fusion_profile) {
    case profiles::RcsFusionProfile::kConservative:
      d.rcs_physics.enable_physical_rcs = true;
      d.rcs_physics.physics_mix_ratio = 0.25f;
      break;
    case profiles::RcsFusionProfile::kEnhanced:
      d.rcs_physics.enable_physical_rcs = true;
      d.rcs_physics.physics_mix_ratio = 0.60f;
      d.rcs_physics.cylinder_weight = 0.65f;
      break;
    case profiles::RcsFusionProfile::kDisabled:
    default:
      break;
  }
}

void ApplyTrackingSemanticConfig(bool enable_tracking_filter,
                                 profiles::TrackingPolicyProfile tracking_profile,
                                 TrackingConfig* tracking_config,
                                 AssociationConfig* association_config) {
  if (tracking_config == nullptr || association_config == nullptr) {
    return;
  }

  auto& t = *tracking_config;
  t.enable_kalman_filter = enable_tracking_filter;
  switch (tracking_profile) {
    case profiles::TrackingPolicyProfile::kFastAssociation:
      t.kalman_measurement_noise_std = 6.0f;
      t.kalman_update_backend = KalmanUpdateBackend::kStandardKfJoseph;
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      break;
    case profiles::TrackingPolicyProfile::kRobustAntiJamming:
      t.kalman_measurement_noise_std = 12.0f;
      t.kalman_update_backend = KalmanUpdateBackend::kUdKf;
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      association_config->unassigned_cost = 12.0f;
      break;
    case profiles::TrackingPolicyProfile::kBalanced:
    default:
      break;
  }
}

void ApplyLifecycleSemanticConfig(bool enable_imm_fusion,
                                  profiles::LifecyclePolicyProfile lifecycle_profile,
                                  LifecycleConfig* lifecycle_config) {
  if (lifecycle_config == nullptr) {
    return;
  }

  auto& lc = *lifecycle_config;
  lc.enable_imm_lifecycle = enable_imm_fusion;
  switch (lifecycle_profile) {
    case profiles::LifecyclePolicyProfile::kFastConfirm:
      lc.confirm_hits = 1U;
      lc.max_miss_before_lost = 1U;
      lc.max_lost_cycles = 3U;
      break;
    case profiles::LifecyclePolicyProfile::kHighPersistence:
      lc.confirm_hits = 3U;
      lc.max_miss_before_lost = 3U;
      lc.max_lost_cycles = 8U;
      break;
    case profiles::LifecyclePolicyProfile::kBalanced:
    default:
      break;
  }
}

config::RadarSessionConfig BuildDefaultSemanticSessionConfig() {
  config::RadarSessionConfig config;
  ApplyDetectionSemanticConfig(
      false, profiles::RadarHardwareProfile::kGenericAirborneXBand,
      profiles::DetectionIntentProfile::kBalanced, profiles::AntennaPatternProfile::kStandard,
      config::AzimuthElevationDeg(), profiles::RcsFusionProfile::kDisabled, &config.hardware);
  ApplyTrackingSemanticConfig(false, profiles::TrackingPolicyProfile::kBalanced,
                              &config.policy.tracking, &config.policy.association);
  ApplyLifecycleSemanticConfig(false, profiles::LifecyclePolicyProfile::kBalanced,
                               &config.policy.lifecycle);
  return config;
}

}  // namespace

RadarSessionConfigBuilder::RadarSessionConfigBuilder()
    : base_config_(BuildDefaultSemanticSessionConfig()) {}

RadarSessionConfigBuilder::RadarSessionConfigBuilder(const config::RadarSessionConfig& config)
    : base_config_(config) {}

config::RadarSessionConfig RadarSessionConfigBuilder::Build() const {
  config::RadarSessionConfig result = base_config_;
  const config::RadarSessionConfig default_semantic = BuildDefaultSemanticSessionConfig();

  if (detection_dirty_) {
    result.hardware = default_semantic.hardware;
    ApplyDetectionSemanticConfig(enable_physics_detection_, hardware_profile_, intent_profile_,
                                 antenna_profile_, antenna_boresight_offset_deg_,
                                 rcs_fusion_profile_, &result.hardware);
  }

  if (tracking_dirty_) {
    result.policy.tracking = default_semantic.policy.tracking;
    result.policy.association = default_semantic.policy.association;
    ApplyTrackingSemanticConfig(enable_tracking_filter_, tracking_profile_, &result.policy.tracking,
                                &result.policy.association);
  }

  if (lifecycle_dirty_) {
    result.policy.lifecycle = default_semantic.policy.lifecycle;
    ApplyLifecycleSemanticConfig(enable_imm_fusion_, lifecycle_profile_, &result.policy.lifecycle);
  }

  return result;
}

ValidationIssueList ValidateRadarSessionConfig(const config::RadarSessionConfig& config) noexcept {
  ValidationIssueList issues;
  const auto push = [&issues](ConfigValidationCode code, const char* field, const char* msg) {
    ConfigValidationIssue issue;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };
  const config::RadarOrientationConfig& orientation = config.mission.orientation;

  if (orientation.commanded_beamwidth_enabled) {
    if (orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg <= 0.0f) {
      push(ConfigValidationCode::kCommandedBeamwidthAzNotPositive,
           "mission.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg",
           "Commanded azimuth beamwidth must be positive when enabled.");
    }
    if (orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg <= 0.0f) {
      push(ConfigValidationCode::kCommandedBeamwidthElNotPositive,
           "mission.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg",
           "Commanded elevation beamwidth must be positive when enabled.");
    }
  }

  if (orientation.mechanical_scan_limits_deg.az_min_deg >
      orientation.mechanical_scan_limits_deg.az_max_deg) {
    push(ConfigValidationCode::kMechanicalScanLimitsSwappedAz,
         "mission.orientation.mechanical_scan_limits_deg",
         "Mechanical azimuth scan min exceeds max.");
  }
  if (orientation.mechanical_scan_limits_deg.el_min_deg >
      orientation.mechanical_scan_limits_deg.el_max_deg) {
    push(ConfigValidationCode::kMechanicalScanLimitsSwappedEl,
         "mission.orientation.mechanical_scan_limits_deg",
         "Mechanical elevation scan min exceeds max.");
  }

  if (orientation.electronic_scan_limits_deg.az_min_deg >
      orientation.electronic_scan_limits_deg.az_max_deg) {
    push(ConfigValidationCode::kElectronicScanLimitsSwappedAz,
         "mission.orientation.electronic_scan_limits_deg",
         "Electronic azimuth scan min exceeds max.");
  }
  if (orientation.electronic_scan_limits_deg.el_min_deg >
      orientation.electronic_scan_limits_deg.el_max_deg) {
    push(ConfigValidationCode::kElectronicScanLimitsSwappedEl,
         "mission.orientation.electronic_scan_limits_deg",
         "Electronic elevation scan min exceeds max.");
  }

  if (config.policy.tracking.kalman_update_backend == config::KalmanUpdateBackend::kUdKf &&
      !config.policy.lifecycle.enable_imm_lifecycle) {
    push(ConfigValidationCode::kRobustTrackingWithoutImm,
         "policy.tracking.kalman_update_backend / policy.lifecycle.enable_imm_lifecycle",
         "Robust anti-jamming tracking should enable IMM lifecycle fusion.");
  }

  return issues;
}

}  // namespace config
}  // namespace airborne_radar
