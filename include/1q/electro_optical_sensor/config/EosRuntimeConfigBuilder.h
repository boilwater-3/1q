/**
 * @file EosRuntimeConfigBuilder.h
 * @brief EOS 对外配置入口：运行期补丁类型与 RuntimeConfig Builder。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_

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
    patch_.has_detection_profile = true;
    patch_.detection_profile = profile;
    return *this;
  }

  EosRuntimeConfigBuilder& WithStrayLightProfile(EosStrayLightProfile profile) noexcept {
    patch_.has_stray_light_profile = true;
    patch_.stray_light_profile = profile;
    return *this;
  }

  EosRuntimeConfigBuilder& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    patch_.has_environment_model_type = true;
    patch_.environment_model_type = model_type;
    return *this;
  }

  EosRuntimeConfigBuilder& WithEnvironmentPreset(EosEnvironmentPreset preset) noexcept {
    patch_.has_environment_preset = true;
    patch_.environment_preset = preset;
    return *this;
  }

  session::EosRuntimeConfigPatch Build() const noexcept { return patch_; }

 private:
  session::EosRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
