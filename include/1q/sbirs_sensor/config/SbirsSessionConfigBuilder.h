/**
 * @file SbirsSessionConfigBuilder.h
 * @brief 定义 SBIRS-inspired 会话配置 builder。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_BUILDER_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace sbirs_sensor {
namespace config {

class ONEQ_API SbirsSessionConfigBuilder {
 public:
  /**
   * @brief 设置硬件配置域。
   * @param[in] hardware 传感器硬件参数
   * @return 自身引用，支持链式调用
   */
  SbirsSessionConfigBuilder& WithHardware(const SbirsHardwareConfig& hardware);
  /**
   * @brief 设置任务配置域。
   * @param[in] mission 任务与视场参数
   * @return 自身引用，支持链式调用
   */
  SbirsSessionConfigBuilder& WithMission(const SbirsMissionConfig& mission);
  /**
   * @brief 设置策略配置域。
   * @param[in] policy 检测/误差/调度策略
   * @return 自身引用，支持链式调用
   */
  SbirsSessionConfigBuilder& WithPolicy(const SbirsPolicyConfig& policy);
  /**
   * @brief 设置环境配置域。
   * @param[in] environment 环境与气象衰减参数
   * @return 自身引用，支持链式调用
   */
  SbirsSessionConfigBuilder& WithEnvironment(const SbirsEnvironmentConfig& environment);
  /**
   * @brief 构造并返回累积的会话配置。
   * @return 已设置四域的 `SbirsSessionConfig`
   */
  SbirsSessionConfig Build() const;

 private:
  SbirsSessionConfig config_{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_BUILDER_H_
