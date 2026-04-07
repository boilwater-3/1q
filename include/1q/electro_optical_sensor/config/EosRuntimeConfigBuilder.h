/**
 * @file EosRuntimeConfigBuilder.h
 * @brief EOS 对外配置入口：运行期补丁类型与 RuntimeConfig Builder。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosRuntimeConfigBuilder 提供运行期补丁的链式构造。
 */
class ONEQ_API EosRuntimeConfigBuilder {
 public:
  explicit EosRuntimeConfigBuilder(
      const core::session::EosRuntimeConfigPatch& patch = {}) noexcept : patch_(patch) {}

  EosRuntimeConfigBuilder& WithRuntimeConfigPatch(
      const core::session::EosRuntimeConfigPatch& patch) noexcept {
    patch_ = patch;
    return *this;
  }

  EosRuntimeConfigBuilder& WithWorkMode(core::session::EosWorkMode mode) noexcept {
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
  EosRuntimeConfigBuilder& WithMinimumSnrDb(float value) noexcept {
    patch_.has_minimum_snr_db = true;
    patch_.minimum_snr_db = value;
    return *this;
  }
  EosRuntimeConfigBuilder& EnableStraylightFilter(bool enable = true) noexcept {
    patch_.has_enable_straylight_filter = true;
    patch_.enable_straylight_filter = enable;
    return *this;
  }
  EosRuntimeConfigBuilder& WithVisibleReferenceIrradianceWm2(float value) noexcept {
    patch_.has_visible_reference_irradiance_w_m2 = true;
    patch_.visible_reference_irradiance_w_m2 = value;
    return *this;
  }
  EosRuntimeConfigBuilder& WithEnvironmentRuntimeConfigPatch(
      const environment::EosEnvironmentRuntimeConfigPatch& patch) noexcept {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config = patch;
    return *this;
  }
  EosRuntimeConfigBuilder& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_model_type = true;
    patch_.environment_runtime_config.model_type = model_type;
    return *this;
  }
  EosRuntimeConfigBuilder& WithRadiativeTransferModel(
      foundation::radiative_transfer::RadiativeTransferModel model) noexcept {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_radiative_transfer_model = true;
    patch_.environment_runtime_config.radiative_transfer_model = model;
    return *this;
  }
  EosRuntimeConfigBuilder& WithAerosolDensityFactor(float value) noexcept {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_aerosol_density_factor = true;
    patch_.environment_runtime_config.aerosol_density_factor = value;
    return *this;
  }
  EosRuntimeConfigBuilder& WithTurbulenceFactor(float value) noexcept {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_turbulence_factor = true;
    patch_.environment_runtime_config.turbulence_factor = value;
    return *this;
  }
  core::session::EosRuntimeConfigPatch Build() const noexcept { return patch_; }

 private:
  core::session::EosRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
