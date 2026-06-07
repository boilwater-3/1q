/**
 * @file EosRuntimeConfigBuilder.h
 * @brief EOS 运行期补丁构造器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosRuntimeConfigBuilder 提供运行期补丁的链式构造。
 * @note 推荐路径：会话创建后的参数热更新统一通过本构造器生成补丁，
 *       避免直接手写 `config::EosRuntimeConfigPatch` 的 `has_*` 标志位。
 */
class ONEQ_API EosRuntimeConfigBuilder {
 public:
  explicit EosRuntimeConfigBuilder(
      const config::EosRuntimeConfigPatch& patch = {}) noexcept : patch_(patch) {}

  EosRuntimeConfigBuilder& WithRuntimeConfigPatch(
      const config::EosRuntimeConfigPatch& patch) noexcept {
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

  EosRuntimeConfigBuilder& WithEnvironmentScenarioConfig(
      const environment::EosEnvironmentScenarioConfig& config) noexcept {
    patch_.has_environment = true;
    patch_.environment.has_scenario_config = true;
    patch_.environment.scenario_config = config;
    return *this;
  }

  EosRuntimeConfigBuilder& WithWorkMode(config::EosWorkMode mode) noexcept {
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

  EosRuntimeConfigBuilder& WithSensorEnabled(bool enable) noexcept {
    patch_.has_sensor_enabled = true;
    patch_.sensor_enabled = enable;
    return *this;
  }

  EosRuntimeConfigBuilder& WithMinimumSnrDb(float value) noexcept {
    patch_.has_policy = true;
    patch_.policy.detection.minimum_snr_db = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithDetectionSensitivityW(float value) noexcept {
    patch_.has_policy = true;
    patch_.policy.detection.detection_sensitivity_w = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithVisibleReferenceIrradianceWM2(float value) noexcept {
    patch_.has_policy = true;
    patch_.policy.detection.visible_reference_irradiance_w_m2 = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithEnableStraylightFilter(bool value) noexcept {
    patch_.has_policy = true;
    patch_.policy.stray_light.enable_straylight_filter = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithHoodInnerHalfAngleDeg(float value) noexcept {
    patch_.has_policy = true;
    patch_.policy.stray_light.hood_inner_half_angle_deg = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithHoodOuterHalfAngleDeg(float value) noexcept {
    patch_.has_policy = true;
    patch_.policy.stray_light.hood_outer_half_angle_deg = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithHoodMinSuppressionRatio(float value) noexcept {
    patch_.has_policy = true;
    patch_.policy.stray_light.hood_min_suppression_ratio = value;
    return *this;
  }

  EosRuntimeConfigBuilder& WithHoodMaxSuppressionRatio(float value) noexcept {
    patch_.has_policy = true;
    patch_.policy.stray_light.hood_max_suppression_ratio = value;
    return *this;
  }


  config::EosRuntimeConfigPatch Build() const noexcept { return patch_; }

 private:
  config::EosRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_BUILDER_H_
