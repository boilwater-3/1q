/**
 * @file EosSessionConfigBuilder.h
 * @brief EOS 对外配置入口：初始化配置类型与 SessionConfig Builder。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/core/session/EosSession.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosSessionConfigBuilder 提供初始化配置的链式构造。
 */
class ONEQ_API EosSessionConfigBuilder {
 public:
  explicit EosSessionConfigBuilder(
      const core::session::EosSessionConfig& config = {}) noexcept : config_(config) {}

  EosSessionConfigBuilder& WithSessionConfig(
      const core::session::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }
  EosSessionConfigBuilder& WithWorkMode(core::session::EosWorkMode mode) noexcept {
    config_.work_mode = mode;
    return *this;
  }
  EosSessionConfigBuilder& WithScanRateDegPerSec(float value) noexcept {
    config_.scan_rate_deg_per_sec = value;
    return *this;
  }
  EosSessionConfigBuilder& WithFrameRateHz(float value) noexcept {
    config_.frame_rate_hz = value;
    return *this;
  }
  EosSessionConfigBuilder& WithMinimumSnrDb(float value) noexcept {
    config_.minimum_snr_db = value;
    return *this;
  }
  EosSessionConfigBuilder& EnableStraylightFilter(bool enable = true) noexcept {
    config_.enable_straylight_filter = enable;
    return *this;
  }
  EosSessionConfigBuilder& WithVisibleReferenceIrradianceWm2(float value) noexcept {
    config_.visible_reference_irradiance_w_m2 = value;
    return *this;
  }
  EosSessionConfigBuilder& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    config_.environment_default_config.model_type = model_type;
    return *this;
  }
  EosSessionConfigBuilder& WithRadiativeTransferModel(
      foundation::radiative_transfer::RadiativeTransferModel model) noexcept {
    config_.environment_default_config.radiative_transfer_model = model;
    return *this;
  }
  EosSessionConfigBuilder& WithAerosolDensityFactor(float value) noexcept {
    config_.environment_default_config.aerosol_density_factor = value;
    return *this;
  }
  EosSessionConfigBuilder& WithTurbulenceFactor(float value) noexcept {
    config_.environment_default_config.turbulence_factor = value;
    return *this;
  }
  core::session::EosSessionConfig Build() const noexcept { return config_; }

 private:
  core::session::EosSessionConfig config_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
