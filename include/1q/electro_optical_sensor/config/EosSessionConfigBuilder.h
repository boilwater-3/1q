/**
 * @file EosSessionConfigBuilder.h
 * @brief 光电传感器会话配置链式构造器（薄封装）。
 *
 * Builder 仅做整域赋值与拷贝，不执行任何翻译、合并或覆写逻辑；
 * `Build()` 返回 `config_` 的副本。语义档位已提取为
 * `EosProfileConstants.h` 中的预定义结构体常量，用户直接赋值即可：
 *
 * @code
 * auto config = EosSessionConfigBuilder()
 *                   .WithMission(profiles::kWideAreaSearchMission)
 *                   .WithHardware(profiles::kLongRangeLargeApertureHardware)
 *                   .WithEnvironmentPreset(EosEnvironmentPreset::kStandard)
 *                   .Build();
 * config.policy.detection.minimum_snr_db = 4.5f;  // 微调 = 直接赋值（不再被 Profile 覆写）
 * @endcode
 *
 * @note 由于不再有 Profile 翻译，配置中不存在"覆盖直接赋值"语义——
 *   对 config 的任何赋值即最终决定，构造顺序无关。
 *
 * @note 推荐路径：
 * - 会话初始化：本构造器整域赋值，或直接构造 `config::EosSessionConfig` 并逐字段赋值；
 * - 运行期热更新统一使用 `EosRuntimeConfigBuilder`。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosSessionConfigBuilder 提供会话配置链式构造（薄封装）。
 */
class ONEQ_API EosSessionConfigBuilder {
 public:
  explicit EosSessionConfigBuilder(const config::EosSessionConfig& config = {}) noexcept
      : config_(config) {}

  EosSessionConfigBuilder& WithSessionConfig(const config::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }
  /** @brief 整域替换任务配置。 */
  EosSessionConfigBuilder& WithMission(const config::EosMissionConfig& mission) noexcept {
    config_.mission = mission;
    return *this;
  }
  /** @brief 整域替换硬件配置。 */
  EosSessionConfigBuilder& WithHardware(const config::EosHardwareConfig& hardware) noexcept {
    config_.hardware = hardware;
    return *this;
  }
  /** @brief 整域替换策略配置。 */
  EosSessionConfigBuilder& WithPolicy(const config::EosPolicyConfig& policy) noexcept {
    config_.policy = policy;
    return *this;
  }
  /** @brief 整域替换环境配置。 */
  EosSessionConfigBuilder& WithEnvironment(const config::EosEnvironmentConfig& environment) noexcept {
    config_.environment = environment;
    return *this;
  }
  /** @brief 设置环境预设档位（遗留 convenience，直接写单字段；其余配置走整域赋值）。 */
  EosSessionConfigBuilder& WithEnvironmentPreset(
      config::EosEnvironmentPreset preset) noexcept {
    config_.environment.scenario_config.preset = preset;
    return *this;
  }

  /** @brief 返回当前配置副本。 */
  config::EosSessionConfig Build() const noexcept;

 private:
  config::EosSessionConfig config_{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
