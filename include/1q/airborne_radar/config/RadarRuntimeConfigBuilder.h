/**
 * @file RadarRuntimeConfigBuilder.h
 * @brief 提供运行期可变配置补丁的链式构造器。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief RadarRuntimeConfigBuilder 提供运行期补丁的链式构造。
 * @note 推荐路径：会话创建后的参数热更新统一通过本构造器生成补丁，
 *       避免直接手写 `RadarRuntimeConfigPatch` 的 `has_*` 标志位。
 */
class ONEQ_API RadarRuntimeConfigBuilder {
 public:
  /** @brief 整块覆盖任务域。 */
  RadarRuntimeConfigBuilder& WithMission(const config::RadarMissionConfig& mission) {
    patch_.has_mission = true;
    patch_.mission = mission;
    return *this;
  }

  /** @brief 整块覆盖策略域。 */
  RadarRuntimeConfigBuilder& WithPolicy(const config::RadarPolicyConfig& policy) {
    patch_.has_policy = true;
    patch_.policy = policy;
    return *this;
  }

  /** @brief 覆盖整套环境场景输入。 */
  RadarRuntimeConfigBuilder& WithEnvironmentScenarioConfig(
      const environment::EnvironmentScenarioConfig& config) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_scenario_config = true;
    patch_.environment_runtime_config.scenario_config = config;
    return *this;
  }

  /** @brief 应用环境运行期补丁。 */
  RadarRuntimeConfigBuilder& WithEnvironmentRuntimeConfig(
      const environment::EnvironmentRuntimeConfigPatch& patch) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config = patch;
    return *this;
  }

  /** @brief 更新干扰判定灵敏度语义档位。 */
  RadarRuntimeConfigBuilder& WithJammingSensitivityProfile(
      environment::JammingSensitivityProfile profile) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_jamming_sensitivity_profile = true;
    patch_.environment_runtime_config.jamming_sensitivity_profile = profile;
    return *this;
  }

  /** @brief 更新雷达工作子模式。 */
  RadarRuntimeConfigBuilder& WithRadarWorkSubMode(RadarWorkSubMode work_sub_mode) {
    patch_.has_work_sub_mode = true;
    patch_.work_sub_mode = work_sub_mode;
    return *this;
  }

  /** @brief 更新扫描中心。 */
  RadarRuntimeConfigBuilder& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    patch_.has_scan_center_deg = true;
    patch_.scan_center_deg = scan_center_deg;
    return *this;
  }

  /** @brief 更新驻留中心。 */
  RadarRuntimeConfigBuilder& WithDwellCenterDeg(const AzimuthElevationDeg& dwell_center_deg) {
    patch_.has_dwell_center_deg = true;
    patch_.dwell_center_deg = dwell_center_deg;
    return *this;
  }

  /** @brief 更新指令态波束宽度。 */
  RadarRuntimeConfigBuilder& WithCommandedBeamwidthDeg(
      const CommandedBeamwidthDeg& commanded_beamwidth_deg) {
    patch_.has_commanded_beamwidth_deg = true;
    patch_.commanded_beamwidth_deg = commanded_beamwidth_deg;
    return *this;
  }

  /** @brief 更新指令态波束宽度使能开关。 */
  RadarRuntimeConfigBuilder& EnableCommandedBeamwidth(bool enable = true) {
    patch_.has_commanded_beamwidth_enabled = true;
    patch_.commanded_beamwidth_enabled = enable;
    return *this;
  }

  /** @brief 生成运行期配置补丁。 */
  RadarRuntimeConfigPatch Build() const { return patch_; }

 private:
  RadarRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_BUILDER_H_
