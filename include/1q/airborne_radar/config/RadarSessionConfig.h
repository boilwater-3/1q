/**
 * @file RadarSessionConfig.h
 * @brief 定义 RadarSession 的初始化配置结构。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/SignalBeamControlConfig.h"
#include "1q/airborne_radar/config/SignalDetectionConfig.h"
#include "1q/airborne_radar/config/SignalLifecycleConfig.h"
#include "1q/airborne_radar/config/SignalTrackingConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSessionConfig 描述 RadarSession 的默认装配配置。
 */
struct ONEQ_API RadarSessionConfig {
  /** @brief 信号探测基线配置（初始化固定，运行期可通过 `ApplyRuntimeConfig` 覆盖部分字段）。 */
  config::SignalDetectionConfig detection{};
  /**
   * @brief 波束控制基线配置。
   * @note `radar_orientation.mount_angles_deg` 语义为 Body -> Radar。
   *       平台姿态属于运行期外部输入，不在 session baseline config 中静态持有。
   */
  config::SignalBeamControlConfig beam_control{};
  /** @brief 跟踪基线配置。 */
  config::SignalTrackingConfig tracking{};
  /** @brief 生命周期基线配置。 */
  config::SignalLifecycleConfig lifecycle{};
  /** @brief 环境模型基线配置（初始化固定，运行期可通过 `ApplyRuntimeConfig` 覆盖）。 */
  environment::EnvironmentDefaultConfig environment_default_config{};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
