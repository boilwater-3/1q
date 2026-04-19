#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace config {

namespace {

void ApplyDetectionSemanticConfig(bool enable_physics_detection,
                                  semantic::RadarHardwareProfile hardware_profile,
                                  semantic::DetectionIntentProfile intent_profile,
                                  semantic::AntennaPatternProfile antenna_profile,
                                  const model::AzimuthElevationDeg& antenna_boresight_offset_deg,
                                  semantic::RcsFusionProfile rcs_fusion_profile,
                                  DetectionConfig* detection_config) {
  if (detection_config == nullptr) {
    return;
  }
  auto& d = *detection_config;
  d.enable_physics_detection = enable_physics_detection;

  // 硬件档位 → 发射机 / 天线 / 接收机物理参数
  switch (hardware_profile) {
    case semantic::RadarHardwareProfile::kLongRangeHighPower:
      d.transmitter.peak_power_w = 5.0e6f;
      d.transmitter.frequency_hz = 9.3e9f;
      d.transmitter.bandwidth_hz = 3.0e6f;
      d.transmitter.pulse_width_s = 18e-6f;
      d.transmitter.prf_hz = 220.0f;
      d.antenna.main_beam_gain_db = 38.0f;
      d.receiver.noise_figure_db = 3.0f;
      break;
    case semantic::RadarHardwareProfile::kLightweightLpi:
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
    case semantic::RadarHardwareProfile::kGenericAirborneXBand:
    default:
      break;
  }

  // 探测意图档位 → 积累数 / 虚警率 / 最小信噪比 / 最小裕量
  switch (intent_profile) {
    case semantic::DetectionIntentProfile::kDetectionPriority:
      d.pulse_count = 16;
      d.detection_policy.cfar_pfa = 2e-6f;
      d.detection_policy.min_snr_db = -12.0f;
      d.min_detection_margin_db = -100.0f;
      break;
    case semantic::DetectionIntentProfile::kTrackStabilityPriority:
      d.pulse_count = 8;
      d.detection_policy.cfar_pfa = 5e-7f;
      d.detection_policy.min_snr_db = -8.0f;
      d.min_detection_margin_db = -20.0f;
      break;
    case semantic::DetectionIntentProfile::kBalanced:
    default:
      d.min_detection_margin_db = -2.0f;
      break;
  }

  // 方向图档位 → 旁瓣 / 覆盖模型参数
  switch (antenna_profile) {
    case semantic::AntennaPatternProfile::kLowSidelobe:
      d.antenna.pattern.max_sidelobe_level_db = -30.0f;
      d.antenna.pattern.backlobe_level_db = -42.0f;
      break;
    case semantic::AntennaPatternProfile::kWideCoverage:
      d.antenna.pattern.model_type = AntennaPatternModelType::kParabolicMainLobe;
      d.antenna.pattern.max_sidelobe_level_db = -18.0f;
      d.antenna.pattern.max_scan_loss_db = 8.0f;
      break;
    case semantic::AntennaPatternProfile::kStandard:
    default:
      break;
  }
  d.antenna.pattern.boresight_offset_deg = antenna_boresight_offset_deg;

  // RCS 融合档位 → 物理 RCS 混合参数
  switch (rcs_fusion_profile) {
    case semantic::RcsFusionProfile::kConservative:
      d.rcs_physics.enable_physical_rcs = true;
      d.rcs_physics.physics_mix_ratio = 0.25f;
      break;
    case semantic::RcsFusionProfile::kEnhanced:
      d.rcs_physics.enable_physical_rcs = true;
      d.rcs_physics.physics_mix_ratio = 0.60f;
      d.rcs_physics.cylinder_weight = 0.65f;
      break;
    case semantic::RcsFusionProfile::kDisabled:
    default:
      break;
  }
}

void ApplyTrackingSemanticConfig(bool enable_tracking_filter,
                                 semantic::TrackingPolicyProfile tracking_profile,
                                 TrackingConfig* tracking_config,
                                 AssociationConfig* association_config) {
  if (tracking_config == nullptr || association_config == nullptr) {
    return;
  }

  // 跟踪档位 → Kalman 参数 + 衰减系数
  auto& t = *tracking_config;
  t.enable_kalman_filter = enable_tracking_filter;
  switch (tracking_profile) {
    case semantic::TrackingPolicyProfile::kFastAssociation:
      t.kalman_measurement_noise_std = 6.0f;
      t.kalman_update_backend = KalmanUpdateBackend::kStandardKfJoseph;
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      break;
    case semantic::TrackingPolicyProfile::kRobustAntiJamming:
      t.kalman_measurement_noise_std = 12.0f;
      t.kalman_update_backend = KalmanUpdateBackend::kUdKf;
      t.speed_decay_ratio_on_loss = 0.95f;
      t.rcs_decay_ratio_on_loss = 0.92f;
      association_config->unassigned_cost = 12.0f;
      break;
    case semantic::TrackingPolicyProfile::kBalanced:
    default:
      break;
  }
}

void ApplyLifecycleSemanticConfig(bool enable_imm_fusion,
                                  semantic::LifecyclePolicyProfile lifecycle_profile,
                                  LifecycleConfig* lifecycle_config) {
  if (lifecycle_config == nullptr) {
    return;
  }

  // 生命周期档位 → 确认门限 / 丢失门限
  auto& lc = *lifecycle_config;
  lc.enable_imm_lifecycle = enable_imm_fusion;
  switch (lifecycle_profile) {
    case semantic::LifecyclePolicyProfile::kFastConfirm:
      lc.confirm_hits = 1U;
      lc.max_miss_before_lost = 1U;
      lc.max_lost_cycles = 3U;
      break;
    case semantic::LifecyclePolicyProfile::kHighPersistence:
      lc.confirm_hits = 3U;
      lc.max_miss_before_lost = 3U;
      lc.max_lost_cycles = 8U;
      break;
    case semantic::LifecyclePolicyProfile::kBalanced:
    default:
      break;
  }
}

session::RadarSessionConfig BuildDefaultSemanticSessionConfig() {
  session::RadarSessionConfig config;
  ApplyDetectionSemanticConfig(false, semantic::RadarHardwareProfile::kGenericAirborneXBand,
                               semantic::DetectionIntentProfile::kBalanced,
                               semantic::AntennaPatternProfile::kStandard,
                               model::AzimuthElevationDeg(), semantic::RcsFusionProfile::kDisabled,
                               &config.hardware.detection);
  ApplyTrackingSemanticConfig(false, semantic::TrackingPolicyProfile::kBalanced,
                              &config.policy.tracking, &config.policy.association);
  ApplyLifecycleSemanticConfig(false, semantic::LifecyclePolicyProfile::kBalanced,
                               &config.policy.lifecycle);
  return config;
}

}  // namespace

RadarSessionConfigBuilder::RadarSessionConfigBuilder()
    : base_config_(BuildDefaultSemanticSessionConfig()),
      orientation_(base_config_.mission.orientation),
      env_(base_config_.environment) {}

RadarSessionConfigBuilder::RadarSessionConfigBuilder(const session::RadarSessionConfig& config)
    : base_config_(config), orientation_(config.mission.orientation), env_(config.environment) {}

session::RadarSessionConfig RadarSessionConfigBuilder::Build() const {
  session::RadarSessionConfig result = base_config_;
  const session::RadarSessionConfig default_semantic = BuildDefaultSemanticSessionConfig();
  result.mission.orientation = orientation_;
  result.environment = env_;

  if (detection_dirty_) {
    result.hardware.detection = default_semantic.hardware.detection;
    ApplyDetectionSemanticConfig(enable_physics_detection_, hardware_profile_, intent_profile_,
                                 antenna_profile_, antenna_boresight_offset_deg_,
                                 rcs_fusion_profile_, &result.hardware.detection);
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

  if (tracking_dirty_ && tracking_profile_ == semantic::TrackingPolicyProfile::kRobustAntiJamming &&
      !enable_imm_fusion_) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] robust tracking policy is set while IMM fusion is disabled; "
        "consider enabling IMM for stronger anti-jamming stability.");
  }
  return result;
}

}  // namespace config
}  // namespace airborne_radar
