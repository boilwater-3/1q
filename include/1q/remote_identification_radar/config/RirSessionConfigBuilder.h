/**
 * @file RirSessionConfigBuilder.h
 * @brief 远程识别雷达会话配置薄封装建造者。
 *
 * 仅提供整域赋值与 `Build()` 返回副本，不承担 leaf setter 或隐式 validation
 * （对齐五模块 ArSessionConfigBuilder 薄封装语义）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirSessionConfigBuilder 会话配置建造者（薄封装）。
 */
class ONEQ_API RirSessionConfigBuilder {
 public:
  RirSessionConfigBuilder() = default;

  /** @brief 整域赋值：硬件域。 */
  RirSessionConfigBuilder& WithHardware(const RirHardwareConfig& hardware) {
    config_.hardware = hardware;
    return *this;
  }

  /** @brief 整域赋值：任务域。 */
  RirSessionConfigBuilder& WithMission(const RirMissionConfig& mission) {
    config_.mission = mission;
    return *this;
  }

  /** @brief 整域赋值：策略域。 */
  RirSessionConfigBuilder& WithPolicy(const RirPolicyConfig& policy) {
    config_.policy = policy;
    return *this;
  }

  /** @brief 整域赋值：环境域。 */
  RirSessionConfigBuilder& WithEnvironment(const RirEnvironmentConfig& environment) {
    config_.environment = environment;
    return *this;
  }

  /** @brief 电源状态（COMMON-OQ-4 提升字段）。 */
  RirSessionConfigBuilder& WithSensorEnabled(bool sensor_enabled) {
    config_.sensor_enabled = sensor_enabled;
    return *this;
  }

  /** @brief 返回当前配置副本。 */
  RirSessionConfig Build() const { return config_; }

 private:
  RirSessionConfig config_{};
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_SESSION_CONFIG_BUILDER_H_
