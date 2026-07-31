/**
 * @file EsrSessionConfigBuilder.h
 * @brief 电子侦察雷达会话配置链式构造器（薄封装）。
 *
 * Builder 仅做整域赋值与拷贝，不执行任何翻译、合并或覆写逻辑；
 * `Build()` 返回 `config_` 的副本。语义档位已提取为
 * `EsrProfileConstants.h` 中的预定义结构体常量，用户直接赋值即可：
 *
 * @code
 * config::EsrPolicyConfig policy;
 * policy.detection = profiles::kHighSensitivityDetection;
 * auto config = EsrSessionConfigBuilder()
 *                   .WithMission(profiles::kThreatWarningMission)
 *                   .WithPolicy(policy)
 *                   .WithEnvironmentPreset(EsrEnvironmentPreset::kStandard)
 *                   .Build();
 * config.mission.scan.scan_rate_hz = 1.0f;  // 微调 = 直接赋值（不再被 Profile 覆写）
 * @endcode
 *
 * @note 由于不再有 Profile 翻译，配置中不存在"覆盖直接赋值"语义——
 *   对 config 的任何赋值即最终决定，构造顺序无关。
 *   `WithSessionConfig()` 不会重置任何已设置的值（无 dirty flag 概念）。
 *
 * @note 推荐路径：
 * - 会话初始化：本构造器整域赋值，或直接构造 `config::EsrSessionConfig` 并逐字段赋值；
 * - 运行期热更新统一使用 `EsrRuntimeConfigBuilder`。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrSessionConfigBuilder 提供会话配置链式构造（薄封装）。
 */
class ONEQ_API EsrSessionConfigBuilder {
 public:
  explicit EsrSessionConfigBuilder(const config::EsrSessionConfig& config = {}) : config_(config) {}

  EsrSessionConfigBuilder& WithSessionConfig(const config::EsrSessionConfig& config) {
    config_ = config;
    return *this;
  }
  /** @brief 整域替换任务配置。 */
  EsrSessionConfigBuilder& WithMission(const config::EsrMissionConfig& mission) {
    config_.mission = mission;
    return *this;
  }
  /** @brief 整域替换硬件配置。 */
  EsrSessionConfigBuilder& WithHardware(const config::EsrHardwareConfig& hardware) {
    config_.hardware = hardware;
    return *this;
  }
  /** @brief 整域替换策略配置。 */
  EsrSessionConfigBuilder& WithPolicy(const config::EsrPolicyConfig& policy) {
    config_.policy = policy;
    return *this;
  }
  /** @brief 整域替换环境配置。 */
  EsrSessionConfigBuilder& WithEnvironment(const config::EsrEnvironmentConfig& environment) {
    config_.environment = environment;
    return *this;
  }
  /** @brief 设置环境预设档位（遗留 convenience，直接写单字段；其余配置走整域赋值）。 */
  EsrSessionConfigBuilder& WithEnvironmentPreset(config::EsrEnvironmentPreset preset) {
    config_.environment.scenario_config.preset = preset;
    return *this;
  }

  /** @brief 返回当前配置副本。 */
  config::EsrSessionConfig Build() const;

 private:
  config::EsrSessionConfig config_{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
