/**
 * @file ArRuntimeConfigBuilder.h
 * @brief 机载雷达运行期配置补丁链式构造器。
 *
 * 运行期补丁构造器的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_RUNTIME_CONFIG_BUILDER_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief ArRuntimeConfigBuilder 提供运行期补丁的链式构造。
 * @note 推荐路径：会话创建后的参数热更新统一通过本构造器生成补丁，
 *       避免直接手写 `ArRuntimeConfigPatch` 的 `has_*` 标志位。
 */
class ONEQ_API ArRuntimeConfigBuilder {
 public:
  /** @brief 整块覆盖运行期补丁（与 EOS/ESR 对齐的统一入口）。 */
  ArRuntimeConfigBuilder& WithRuntimeConfigPatch(
      const config::ArRuntimeConfigPatch& patch) noexcept {
    patch_ = patch;
    return *this;
  }

  /** @brief 整块覆盖任务域。 */
  ArRuntimeConfigBuilder& WithMission(const config::ArMissionConfig& mission) noexcept {
    patch_.has_mission = true;
    patch_.mission = mission;
    return *this;
  }

  /** @brief 整块覆盖策略域。 */
  ArRuntimeConfigBuilder& WithPolicy(const config::ArPolicyConfig& policy) noexcept {
    patch_.has_policy = true;
    patch_.policy = policy;
    return *this;
  }

  /** @brief 覆盖整套环境场景输入。 */
  ArRuntimeConfigBuilder& WithEnvironmentScenarioConfig(
      const config::EnvironmentScenarioConfig& config) noexcept {
    patch_.has_environment = true;
    patch_.environment.has_scenario_config = true;
    patch_.environment.scenario_config = config;
    return *this;
  }

  /** @brief 应用环境运行期补丁。 */
  ArRuntimeConfigBuilder& WithEnvironment(
      const config::EnvironmentRuntimeConfigPatch& patch) noexcept {
    patch_.has_environment = true;
    patch_.environment = patch;
    return *this;
  }

  /** @brief 更新雷达工作模式（命名对齐 EOS/ESR 的 WithWorkMode）。 */
  ArRuntimeConfigBuilder& WithWorkMode(ArWorkMode work_mode) noexcept {
    patch_.has_work_mode = true;
    patch_.work_mode = work_mode;
    return *this;
  }

  /** @brief 更新扫描中心。 */
  ArRuntimeConfigBuilder& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) noexcept {
    patch_.has_scan_center_deg = true;
    patch_.scan_center_deg = scan_center_deg;
    return *this;
  }

  /** @brief 更新驻留中心。 */
  ArRuntimeConfigBuilder& WithDwellCenterDeg(const AzimuthElevationDeg& dwell_center_deg) noexcept {
    patch_.has_dwell_center_deg = true;
    patch_.dwell_center_deg = dwell_center_deg;
    return *this;
  }

  /** @brief 更新指令态波束宽度。 */
  ArRuntimeConfigBuilder& WithCommandedBeamwidthDeg(
      const CommandedBeamwidthDeg& commanded_beamwidth_deg) noexcept {
    patch_.has_commanded_beamwidth_deg = true;
    patch_.commanded_beamwidth_deg = commanded_beamwidth_deg;
    return *this;
  }

  /** @brief 设置设备开关机状态。 */
  ArRuntimeConfigBuilder& WithSensorEnabled(bool enable) noexcept {
    patch_.has_sensor_enabled = true;
    patch_.sensor_enabled = enable;
    return *this;
  }

  /** @brief 更新指令态波束宽度使能开关（命名对齐 WithSensorEnabled）。 */
  ArRuntimeConfigBuilder& WithCommandedBeamwidthEnabled(bool enable = true) noexcept {
    patch_.has_commanded_beamwidth_enabled = true;
    patch_.commanded_beamwidth_enabled = enable;
    return *this;
  }

  /** @brief 生成运行期配置补丁。 */
  ArRuntimeConfigPatch Build() const noexcept { return patch_; }

 private:
  ArRuntimeConfigPatch patch_{};
};


}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_RUNTIME_CONFIG_BUILDER_H_
