#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"

#include <cmath>

#include "1q/airborne_radar/config/ArSessionConfigValidation.h"
#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace config {

namespace {

void ApplyDetectionSemanticConfig(profiles::ArHardwareProfile hardware_profile,
                                  profiles::DetectionIntentProfile intent_profile,
                                  profiles::AntennaPatternProfile antenna_profile,
                                  const config::AzimuthElevationDeg& antenna_boresight_offset_deg,
                                  profiles::RcsFusionProfile rcs_fusion_profile,
                                  DetectionConfig* detection_config,
                                  ArDetectionPolicyConfig* detection_policy) {
  if (detection_config == nullptr || detection_policy == nullptr) {
    return;
  }
  auto& d = *detection_config;
  auto& policy = *detection_policy;
  switch (hardware_profile) {
    case profiles::ArHardwareProfile::kLongRangeHighPower:
      d.transmitter.peak_power_w = 5.0e6f;
      d.transmitter.frequency_hz = 9.3e9f;
      d.transmitter.bandwidth_hz = 3.0e6f;
      d.transmitter.pulse_width_s = 18e-6f;
      d.transmitter.prf_hz = 220.0f;
      d.antenna.main_beam_gain_db = 38.0f;
      d.receiver.noise_figure_db = 3.0f;
      break;
    case profiles::ArHardwareProfile::kLightweightLpi:
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
    case profiles::ArHardwareProfile::kGenericAirborneXBand:
    default:
      break;
  }

  switch (intent_profile) {
    case profiles::DetectionIntentProfile::kDetectionPriority:
      policy.pulse_count = 16;
      policy.pfa = 2e-6f;
      policy.minimum_snr_db = -12.0f;
      policy.minimum_detection_margin_db = -100.0f;
      break;
    case profiles::DetectionIntentProfile::kTrackStabilityPriority:
      policy.pulse_count = 8;
      policy.pfa = 5e-7f;
      policy.minimum_snr_db = -8.0f;
      policy.minimum_detection_margin_db = -20.0f;
      break;
    case profiles::DetectionIntentProfile::kBalanced:
    default:
      policy.minimum_detection_margin_db = -2.0f;
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
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      break;
    case profiles::TrackingPolicyProfile::kRobustAntiJamming:
      t.kalman_measurement_noise_std = 12.0f;
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      association_config->distance_gate_sigma = std::sqrt(12.0f);
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

config::ArSessionConfig BuildDefaultSemanticSessionConfig() {
  config::ArSessionConfig config;
  ApplyDetectionSemanticConfig(profiles::ArHardwareProfile::kGenericAirborneXBand,
                               profiles::DetectionIntentProfile::kBalanced,
                               profiles::AntennaPatternProfile::kStandard,
                               config::AzimuthElevationDeg(), profiles::RcsFusionProfile::kDisabled,
                               &config.hardware, &config.policy.detection);
  ApplyTrackingSemanticConfig(false, profiles::TrackingPolicyProfile::kBalanced,
                              &config.policy.tracking, &config.policy.association);
  ApplyLifecycleSemanticConfig(false, profiles::LifecyclePolicyProfile::kBalanced,
                               &config.policy.lifecycle);
  return config;
}

}  // namespace

ArSessionConfigBuilder::ArSessionConfigBuilder()
    : base_config_(BuildDefaultSemanticSessionConfig()) {}

ArSessionConfigBuilder::ArSessionConfigBuilder(const config::ArSessionConfig& config)
    : base_config_(config) {}

config::ArSessionConfig ArSessionConfigBuilder::Build() const {
  config::ArSessionConfig result = base_config_;
  const config::ArSessionConfig default_semantic = BuildDefaultSemanticSessionConfig();

  if (detection_dirty_) {
    result.hardware = default_semantic.hardware;
    ApplyDetectionSemanticConfig(hardware_profile_, intent_profile_, antenna_profile_,
                                 antenna_boresight_offset_deg_, rcs_fusion_profile_,
                                 &result.hardware, &result.policy.detection);
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

ValidationIssueList ValidateArSessionConfig(const config::ArSessionConfig& config) noexcept {
  ValidationIssueList issues;
  const auto push = [&issues](ConfigValidationCode code, const char* field, const char* msg) {
    ConfigValidationIssue issue;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };
  const config::ArOrientationConfig& orientation = config.mission.orientation;
  const config::detection::AntennaConfig& antenna = config.hardware.antenna;
  const config::detection::ReceiverConfig& receiver = config.hardware.receiver;
  const float transmitter_frequency_hz = config.hardware.transmitter.frequency_hz;

  if (!oneq::common::validation::IsFinite(transmitter_frequency_hz) ||
      transmitter_frequency_hz <= 0.0f) {
    push(ConfigValidationCode::kTransmitterFrequencyInvalid, "hardware.transmitter.frequency_hz",
         "Transmitter frequency must be finite and positive.");
  }
  const bool known_receiver_polarization =
      receiver.polarization == oneq::electromagnetics::RfPolarization::kHorizontal ||
      receiver.polarization == oneq::electromagnetics::RfPolarization::kVertical ||
      receiver.polarization == oneq::electromagnetics::RfPolarization::kRightHandCircular ||
      receiver.polarization == oneq::electromagnetics::RfPolarization::kLeftHandCircular ||
      receiver.polarization == oneq::electromagnetics::RfPolarization::kUnpolarized;
  if (!known_receiver_polarization ||
      !oneq::common::validation::IsFinite(receiver.cross_polarization_isolation_db) ||
      receiver.cross_polarization_isolation_db < 0.0f ||
      !oneq::common::validation::IsFinite(receiver.minimum_far_field_range_m) ||
      receiver.minimum_far_field_range_m <= 0.0f ||
      (receiver.has_co_site_isolation &&
       (!oneq::common::validation::IsFinite(receiver.co_site_isolation_db) ||
        receiver.co_site_isolation_db < 0.0f)) ||
      !oneq::common::validation::IsFinite(receiver.maximum_linear_input_power_w) ||
      receiver.maximum_linear_input_power_w <= 0.0f) {
    push(ConfigValidationCode::kReceiverRfHardwareInvalid, "hardware.receiver",
         "Receiver RF polarization, isolation, far-field range and linear input limit must be "
         "valid.");
  }
  const auto axis_geometry_valid = [transmitter_frequency_hz](float nominal_beamwidth_deg,
                                                              float aperture_m,
                                                              bool commanded_override_enabled) {
    if (!oneq::common::validation::IsFinite(nominal_beamwidth_deg) ||
        !oneq::common::validation::IsFinite(aperture_m) || nominal_beamwidth_deg < 0.0f ||
        aperture_m < 0.0f) {
      return false;
    }
    if (commanded_override_enabled || nominal_beamwidth_deg > 0.0f) {
      return true;
    }
    return aperture_m > 0.0f && oneq::common::validation::IsFinite(transmitter_frequency_hz) &&
           transmitter_frequency_hz > 0.0f;
  };

  if (orientation.commanded_beamwidth_enabled) {
    if (!oneq::common::validation::IsFinite(
            orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg) ||
        orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg <= 0.0f) {
      push(ConfigValidationCode::kCommandedBeamwidthAzNotPositive,
           "mission.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg",
           "Commanded azimuth beamwidth must be finite and positive when enabled.");
    }
    if (!oneq::common::validation::IsFinite(
            orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg) ||
        orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg <= 0.0f) {
      push(ConfigValidationCode::kCommandedBeamwidthElNotPositive,
           "mission.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg",
           "Commanded elevation beamwidth must be finite and positive when enabled.");
    }
  }

  if (!axis_geometry_valid(antenna.nominal_az_beamwidth_deg, antenna.antenna_length_m,
                           orientation.commanded_beamwidth_enabled)) {
    push(ConfigValidationCode::kAntennaAzGeometryInvalid,
         "hardware.antenna.nominal_az_beamwidth_deg / antenna_length_m",
         "Azimuth beamwidth requires a positive nominal value or a valid physical aperture.");
  }
  if (!axis_geometry_valid(antenna.nominal_el_beamwidth_deg, antenna.antenna_width_m,
                           orientation.commanded_beamwidth_enabled)) {
    push(ConfigValidationCode::kAntennaElGeometryInvalid,
         "hardware.antenna.nominal_el_beamwidth_deg / antenna_width_m",
         "Elevation beamwidth requires a positive nominal value or a valid physical aperture.");
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

  return issues;
}

}  // namespace config
}  // namespace airborne_radar
