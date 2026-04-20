/**
 * @file EosDetailedSessionConfigBuilder.h
 * @brief EOS 细粒度会话配置构造器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_DETAILED_SESSION_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_DETAILED_SESSION_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosDetailedSessionConfigBuilder 提供四域详细配置构造入口。
 * @note 该构造器面向高级调用方，可直接编辑
 *       hardware/mission/policy/environment 以及策略详细参数。
 */
class ONEQ_API EosDetailedSessionConfigBuilder {
 public:
  explicit EosDetailedSessionConfigBuilder(
      const session::EosSessionConfig& config = {}) noexcept : config_(config) {}

  EosDetailedSessionConfigBuilder& WithSessionConfig(
      const session::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithHardware(
      const EosHardwareConfig& hardware) noexcept {
    config_.hardware = hardware;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithMission(
      const EosMissionConfig& mission) noexcept {
    config_.mission = mission;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithPolicy(const EosPolicyConfig& policy) noexcept {
    config_.policy = policy;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithEnvironment(
      const EosEnvironmentConfig& environment) noexcept {
    config_.environment = environment;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithWorkMode(session::EosWorkMode mode) noexcept {
    config_.mission.work_mode = mode;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithScanRateDegPerSec(float value) noexcept {
    config_.mission.scan_rate_deg_per_sec = value;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithFrameRateHz(float value) noexcept {
    config_.mission.frame_rate_hz = value;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithDetectionProfile(EosDetectionProfile profile) noexcept {
    config_.policy.detection.profile = profile;
    config_.policy.detection.use_profile_defaults = true;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithStrayLightProfile(EosStrayLightProfile profile) noexcept {
    config_.policy.stray_light.profile = profile;
    config_.policy.stray_light.use_profile_defaults = true;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    config_.environment.scenario_config.model_type = model_type;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithEnvironmentPreset(EosEnvironmentPreset preset) noexcept {
    config_.environment.scenario_config.preset = preset;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithDetectionDetails(float minimum_snr_db,
                                                        float detection_sensitivity_w,
                                                        float visible_reference_irradiance_w_m2) noexcept {
    config_.policy.detection.use_profile_defaults = false;
    config_.policy.detection.minimum_snr_db = minimum_snr_db;
    config_.policy.detection.detection_sensitivity_w = detection_sensitivity_w;
    config_.policy.detection.visible_reference_irradiance_w_m2 =
        visible_reference_irradiance_w_m2;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithStrayLightDetails(
      bool enable_straylight_filter, float hood_inner_half_angle_deg,
      float hood_outer_half_angle_deg, float hood_min_suppression_ratio,
      float hood_max_suppression_ratio) noexcept {
    config_.policy.stray_light.use_profile_defaults = false;
    config_.policy.stray_light.enable_straylight_filter = enable_straylight_filter;
    config_.policy.stray_light.hood_inner_half_angle_deg = hood_inner_half_angle_deg;
    config_.policy.stray_light.hood_outer_half_angle_deg = hood_outer_half_angle_deg;
    config_.policy.stray_light.hood_min_suppression_ratio = hood_min_suppression_ratio;
    config_.policy.stray_light.hood_max_suppression_ratio = hood_max_suppression_ratio;
    return *this;
  }

  EosDetailedSessionConfigBuilder& WithEnvironmentDetails(
      foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model,
      float aerosol_density_factor, float turbulence_factor) noexcept {
    config_.environment.scenario_config.has_custom_overrides = true;
    config_.environment.scenario_config.custom_overrides.radiative_transfer_model =
      radiative_transfer_model;
    config_.environment.scenario_config.custom_overrides.aerosol_density_factor =
      aerosol_density_factor;
    config_.environment.scenario_config.custom_overrides.turbulence_factor =
      turbulence_factor;
    return *this;
  }

  session::EosSessionConfig Build() const noexcept { return config_; }

 private:
  session::EosSessionConfig config_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_DETAILED_SESSION_CONFIG_BUILDER_H_
