/**
 * @file EsrRuntimeConfigBuilder.h
 * @brief ESR 对外配置入口：运行期补丁类型与 RuntimeConfig Builder。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrRuntimeConfigBuilder 提供运行期补丁链式构造。
 * @note 推荐路径：会话创建后的参数热更新统一通过本构造器生成补丁，
 *       避免直接手写 `config::EsrRuntimeConfigPatch` 的 `has_*` 标志位。
 */
class ONEQ_API EsrRuntimeConfigBuilder {
 public:
  explicit EsrRuntimeConfigBuilder(const config::EsrRuntimeConfigPatch& patch = {})
      : patch_(patch) {}

  EsrRuntimeConfigBuilder& WithRuntimeConfigPatch(const config::EsrRuntimeConfigPatch& patch) {
    patch_ = patch;
    return *this;
  }

  EsrRuntimeConfigBuilder& WithMission(const EsrMissionConfig& mission) {
    patch_.has_mission = true;
    patch_.mission = mission;
    return *this;
  }

  EsrRuntimeConfigBuilder& WithPolicy(const EsrPolicyConfig& policy) {
    patch_.has_policy = true;
    patch_.policy = policy;
    return *this;
  }

  EsrRuntimeConfigBuilder& WithEnvironmentRuntimeConfig(
      const environment::EsrEnvironmentRuntimeConfigPatch& env_patch) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config = env_patch;
    return *this;
  }

  EsrRuntimeConfigBuilder& WithSensorEnabled(bool enable) {
    patch_.has_sensor_enabled = true;
    patch_.sensor_enabled = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithWorkMode(config::EsrWorkMode mode) {
    patch_.has_work_mode = true;
    patch_.work_mode = mode;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithScanRateHz(float value) {
    patch_.has_scan_rate_hz = true;
    patch_.scan_rate_hz = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithScanStartPosition(config::EsrScanStartPosition position) {
    patch_.has_scan_start_position = true;
    patch_.scan_start_position = position;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithScanSequence(config::EsrScanSequence sequence) {
    patch_.has_scan_sequence = true;
    patch_.scan_sequence = sequence;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithScanCenterAzDeg(float value) {
    patch_.has_scan_center_az_deg = true;
    patch_.scan_center_az_deg = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithScanCenterElDeg(float value) {
    patch_.has_scan_center_el_deg = true;
    patch_.scan_center_el_deg = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& SetUseExplicitScanBounds(bool enable) {
    patch_.has_use_explicit_scan_bounds = true;
    patch_.use_explicit_scan_bounds = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithExplicitScanBoundsDeg(float start_az, float end_az, float start_el,
                                                     float end_el) {
    patch_.has_use_explicit_scan_bounds = true;
    patch_.use_explicit_scan_bounds = true;
    patch_.has_scan_start_az_deg = true;
    patch_.has_scan_end_az_deg = true;
    patch_.has_scan_start_el_deg = true;
    patch_.has_scan_end_el_deg = true;
    patch_.scan_start_az_deg = start_az;
    patch_.scan_end_az_deg = end_az;
    patch_.scan_start_el_deg = start_el;
    patch_.scan_end_el_deg = end_el;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithAtmosphericPhysicsConfig(
      const environment::EsrAtmosphericPhysicsConfig& atmospheric_physics) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_atmospheric_physics = true;
    patch_.environment_runtime_config.atmospheric_physics = atmospheric_physics;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithAtmosphericContext(
      const environment::EsrAtmosphericDerivedContext& atmospheric_context) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_atmospheric_context = true;
    patch_.environment_runtime_config.atmospheric_context = atmospheric_context;
    return *this;
  }
  config::EsrRuntimeConfigPatch Build() const { return patch_; }

 private:
  config::EsrRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_BUILDER_H_
