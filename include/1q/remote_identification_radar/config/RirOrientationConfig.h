/**
 * @file RirOrientationConfig.h
 * @brief 远程识别雷达安装指向静态配置（会话第五域）。
 *
 * 承载阵面相对可扫描体积等静态硬件几何；运行期转台朝向见
 * `RirMissionConfig::scan_center_deg`。本域为初始化静态配置，不进入
 * `RirRuntimeConfigPatch`（与 contract.md「条件五域」规则一致）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ORIENTATION_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ORIENTATION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirOrientationConfig 表示远程识别雷达静态安装指向配置（会话第五域）。
 * @note steerable_volume_deg 中 az 为相对阵面法线/转台基准的可扫描域，
 *       el 为绝对俯仰域（ENU，deg）。
 */
struct ONEQ_API RirOrientationConfig {
  RirAzimuthElevationLimitsDeg steerable_volume_deg{}; /**< 阵面相对可扫描体积（默认 ±60/±30）。 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ORIENTATION_CONFIG_H_
