/**
 * @file SarSessionConfigBuilder.h
 * @brief 合成孔径雷达会话配置链式构造器（薄封装）。
 *
 * Builder 仅做整域赋值与拷贝，不执行任何翻译、合并或覆写逻辑；
 * `Build()` 返回 `config_` 的副本。语义档位已提取为
 * `SarProfileConstants.h` 中的预定义结构体常量，用户直接赋值即可：
 *
 * @code
 * auto config = SarSessionConfigBuilder()
 *                   .WithMission(profiles::kHighResolutionImagingMission)
 *                   .WithProcessingPolicy(profiles::kL3BackprojectionProcessing)
 *                   .Build();
 * config.mission.l3_waypoints = {...};  // 场景数据 = 档位之后的直接赋值
 * @endcode
 *
 * @note 由于不再有 Profile 翻译，配置中不存在"覆盖直接赋值"语义——
 *   对 config 的任何赋值即最终决定，构造顺序无关。
 *   `WithSessionConfig()` 不会重置任何已设置的值。
 *
 * @warning Mission 档位常量是完整 `SarMissionConfig`，整域赋值会重置
 *   `scene_center_*`、`l2_*`、`l3_waypoints` 等场景特定字段。
 *   正确用法是"先赋档位、再设场景数据"。
 *
 * @note 推荐路径：
 * - 会话初始化：本构造器整域赋值，或直接构造 `config::SarSessionConfig` 并逐字段赋值；
 * - 运行期热更新统一使用 `SarRuntimeConfigBuilder`。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/sar/config/SarSessionConfig.h"

namespace sar {
namespace config {

/**
 * @brief SarSessionConfigBuilder 提供会话配置链式构造（薄封装）。
 */
class ONEQ_API SarSessionConfigBuilder {
 public:
  explicit SarSessionConfigBuilder(const config::SarSessionConfig& config = {})
      : config_(config) {}

  SarSessionConfigBuilder& WithSessionConfig(const config::SarSessionConfig& config) {
    config_ = config;
    return *this;
  }
  /** @brief 整域替换任务配置。 */
  SarSessionConfigBuilder& WithMission(const config::SarMissionConfig& mission) {
    config_.mission = mission;
    return *this;
  }
  /** @brief 整域替换处理策略配置。 */
  SarSessionConfigBuilder& WithProcessingPolicy(const config::SarPolicyConfig& policy) {
    config_.policy = policy;
    return *this;
  }
  /** @brief 整域替换硬件配置。 */
  SarSessionConfigBuilder& WithHardware(const config::SarHardwareConfig& hardware) {
    config_.hardware = hardware;
    return *this;
  }
  /** @brief 整域替换环境配置。 */
  SarSessionConfigBuilder& WithEnvironment(const config::SarEnvironmentConfig& environment) {
    config_.environment = environment;
    return *this;
  }

  /** @brief 返回当前配置副本。 */
  config::SarSessionConfig Build() const noexcept;

 private:
  config::SarSessionConfig config_{};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_
