/**
 * @file ArMissionConfig.h
 * @brief 机载雷达任务域主配置类型。
 *
 * 任务域配置（工作模式、运行期扫描/驻留指向、指令波束宽）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_MISSION_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_MISSION_CONFIG_H_

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief ArMissionConfig 雷达任务域配置。
 *
 * 任务域承载工作子模式与波束指向运行态。电源状态由
 * `ArSessionConfig::sensor_enabled` 顶层字段唯一承载（COMMON-OQ-4 收敛）。
 * 静态安装/限位/稳定归 `ArSessionConfig::orientation`，不在本结构。
 */
struct ONEQ_API ArMissionConfig {
  ArWorkMode work_mode{ArWorkMode::kTws}; /**< [可外部调整] 当前工作模式 */
  AzimuthElevationDeg scan_center_deg; /**< [可外部调整] 基准指向方向；不是扫描体积中心 */
  bool commanded_beamwidth_enabled{false};       /**< 指令态波束宽度覆盖使能 */
  CommandedBeamwidthDeg commanded_beamwidth_deg; /**< 当前指令态瞬时波束宽度 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_MISSION_CONFIG_H_
