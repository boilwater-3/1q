/**
 * @file EosRuntimeConfigBuilder.h
 * @brief EOS 运行期补丁构造器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosRuntimeConfigBuilder 提供运行期补丁的链式构造。
 */
class ONEQ_API EosRuntimeConfigBuilder {
 public:
  explicit EosRuntimeConfigBuilder(
      const session::EosRuntimeConfigPatch& patch = {}) noexcept : patch_(patch) {}

  EosRuntimeConfigBuilder& WithRuntimeConfigPatch(
      const session::EosRuntimeConfigPatch& patch) noexcept {
    patch_ = patch;
    return *this;
  }

  EosRuntimeConfigBuilder& WithMission(const EosMissionConfig& mission) noexcept {
    patch_.has_mission = true;
    patch_.mission = mission;
    return *this;
  }

  EosRuntimeConfigBuilder& WithPolicy(const EosPolicyConfig& policy) noexcept {
    patch_.has_policy = true;
    patch_.policy = policy;
    return *this;
  }

  EosRuntimeConfigBuilder& WithEnvironment(
      const environment::EosEnvironmentRuntimeConfigPatch& environment_patch) noexcept {
    patch_.has_environment = true;
    patch_.environment = environment_patch;
    return *this;
  }

  EosRuntimeConfigBuilder& WithEnvironment(
      const EosEnvironmentConfig& environment) noexcept {
    patch_.has_environment = true;
    patch_.environment.has_model_type = true;
    patch_.environment.model_type = environment.scenario_config.model_type;
    if (environment.scenario_config.has_custom_overrides) {
      patch_.environment.has_radiative_transfer_model = true;
      patch_.environment.radiative_transfer_model =
          environment.scenario_config.custom_overrides.radiative_transfer_model;
      patch_.environment.has_aerosol_density_factor = true;
      patch_.environment.aerosol_density_factor =
          environment.scenario_config.custom_overrides.aerosol_density_factor;
      patch_.environment.has_turbulence_factor = true;
      patch_.environment.turbulence_factor =
          environment.scenario_config.custom_overrides.turbulence_factor;
      patch_.environment.has_enable_optical_countermeasure_extension = true;
      patch_.environment.enable_optical_countermeasure_extension =
          environment.scenario_config.custom_overrides
              .enable_optical_countermeasure_extension;
    }
    return *this;
  }

  EosRuntimeConfigBuilder& WithWorkMode(session::EosWorkMode mode) noexcept {
    patch_.has_work_mode = true;
    patch_.work_mode = mode;
    return *this;
  }

  EosRuntimeConfigBuilder& WithScanRateDegPerSec(float value) noexcept {
    patch_.has_scan_rate_deg_per_sec = true;
    patch_.scan_rate_deg_per_sec = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithFrameRateHz(float value) noexcept {
    patch_.has_frame_rate_hz = true;
    patch_.frame_rate_hz = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithDetectionProfile(EosDetectionProfile profile) noexcept {
    patch_.has_policy = true;
    patch_.policy.detection.profile = profile;
    patch_.policy.detection.use_profile_defaults = true;
    return *this;
  }

  EosRuntimeConfigBuilder& WithStrayLightProfile(EosStrayLightProfile profile) noexcept {
    patch_.has_policy = true;
    patch_.policy.stray_light.profile = profile;
    patch_.policy.stray_light.use_profile_defaults = true;
    return *this;
  }

  EosRuntimeConfigBuilder& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    patch_.has_environment = true;
    patch_.environment.has_model_type = true;
    patch_.environment.model_type = model_type;
    return *this;
  }

  EosRuntimeConfigBuilder& WithDetectionDetails(float minimum_snr_db,
                                                float detection_sensitivity_w,
                                                float visible_reference_irradiance_w_m2) noexcept {
    patch_.has_policy = true;
    patch_.policy.detection.use_profile_defaults = false;
    patch_.policy.detection.minimum_snr_db = minimum_snr_db;
    patch_.policy.detection.detection_sensitivity_w = detection_sensitivity_w;
    patch_.policy.detection.visible_reference_irradiance_w_m2 =
        visible_reference_irradiance_w_m2;
    return *this;
  }

  EosRuntimeConfigBuilder& WithStrayLightDetails(
      bool enable_straylight_filter, float hood_inner_half_angle_deg,
      float hood_outer_half_angle_deg, float hood_min_suppression_ratio,
      float hood_max_suppression_ratio) noexcept {
    patch_.has_policy = true;
    patch_.policy.stray_light.use_profile_defaults = false;
    patch_.policy.stray_light.enable_straylight_filter = enable_straylight_filter;
    patch_.policy.stray_light.hood_inner_half_angle_deg = hood_inner_half_angle_deg;
    patch_.policy.stray_light.hood_outer_half_angle_deg = hood_outer_half_angle_deg;
    patch_.policy.stray_light.hood_min_suppression_ratio = hood_min_suppression_ratio;
    patch_.policy.stray_light.hood_max_suppression_ratio = hood_max_suppression_ratio;
    return *this;
  }

  EosRuntimeConfigBuilder& WithEnvironmentDetails(
      foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model,
      float aerosol_density_factor, float turbulence_factor) noexcept {
    patch_.has_environment = true;
    patch_.environment.has_radiative_transfer_model = true;
    patch_.environment.radiative_transfer_model = radiative_transfer_model;
    patch_.environment.has_aerosol_density_factor = true;
    patch_.environment.aerosol_density_factor = aerosol_density_factor;
    patch_.environment.has_turbulence_factor = true;
    patch_.environment.turbulence_factor = turbulence_factor;
    return *this;
  }

  session::EosRuntimeConfigPatch Build() const noexcept { return patch_; }

 private:
  session::EosRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
