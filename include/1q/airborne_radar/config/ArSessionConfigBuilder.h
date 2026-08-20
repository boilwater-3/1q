/**
 * @file ArSessionConfigBuilder.h
 * @brief 机载雷达会话配置链式构造器。
 *
 * 会话初始化配置链式构造器的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief ArSessionConfig 配置链式构造器（薄封装）。
 *
 * Builder 仅做整域赋值与拷贝，不执行任何翻译、合并或覆写逻辑；
 * `Build()` 返回 `config_` 的副本。语义档位已提取为
 * `ArProfileConstants.h` 中的预定义结构体常量，用户直接赋值即可：
 *
 * @code
 * config::ArPolicyConfig policy;
 * policy.detection = profiles::kDetectionPriorityDetection;
 * auto config = ArSessionConfigBuilder()
 *                   .WithHardware(profiles::kLongRangeHighPowerHardware)
 *                   .WithPolicy(policy)
 *                   .Build();
 * @endcode
 *
 * @note 由于不再有 Profile 翻译，配置中不存在"覆盖直接赋值"语义——
 *   对 config 的任何赋值即最终决定，构造顺序无关。
 *
 * @note 推荐路径：
 * - 会话初始化：本构造器整域赋值，或直接构造 `config::ArSessionConfig` 并逐字段赋值；
 * - 运行期热更新统一使用 `ArRuntimeConfigBuilder`。
 */
class ONEQ_API ArSessionConfigBuilder {
 public:
  /** @brief 使用结构体默认值（语义默认）初始化 Builder。 */
  ArSessionConfigBuilder();

  /**
   * @brief 使用现有会话配置初始化 Builder。
   *
   * @param config 作为编辑基线的会话配置。
   */
  explicit ArSessionConfigBuilder(const config::ArSessionConfig& config);

  /** @brief 整域替换硬件配置。 */
  ArSessionConfigBuilder& WithHardware(const config::ArHardwareConfig& hardware) {
    config_.hardware = hardware;
    return *this;
  }
  /** @brief 整域替换任务配置。 */
  ArSessionConfigBuilder& WithMission(const config::ArMissionConfig& mission) {
    config_.mission = mission;
    return *this;
  }
  /** @brief 整域替换策略配置。 */
  ArSessionConfigBuilder& WithPolicy(const config::ArPolicyConfig& policy) {
    config_.policy = policy;
    return *this;
  }
  /** @brief 整域替换环境配置。 */
  ArSessionConfigBuilder& WithEnvironment(const config::ArEnvironmentConfig& environment) {
    config_.environment = environment;
    return *this;
  }
  /** @brief 整段替换会话配置。 */
  ArSessionConfigBuilder& WithSessionConfig(const config::ArSessionConfig& config) {
    config_ = config;
    return *this;
  }

  /** @brief 返回当前配置副本。 */
  config::ArSessionConfig Build() const;

 private:
  config::ArSessionConfig config_{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_BUILDER_H_
