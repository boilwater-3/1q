/**
 * @file RadarRuntimeConfigPatch.h
 * @brief 定义运行期可变配置的补丁模型。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_PATCH_H_

#include "1q/airborne_radar/config/RadarMissionConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using model::AzimuthElevationDeg;
using model::CommandedBeamwidthDeg;
using model::RadarWorkMode;

/**
 * @brief RadarRuntimeConfigPatch 描述运行期可变参数补丁。
 *
 * @note "可外部调整"定义：调用方可在不重建 `RadarSession` 的前提下，
 * 通过 `RadarSession::ApplyRuntimeConfig(...)` 暂存修改，并在下一次成功周期
 * 提交前统一生效。
 *
 * 支持两类运行期更新：
 * 1) 整域覆盖：`mission`、`policy`、`environment`；
 * 2) 叶子覆盖：传感器开关、工作子模式、扫描/驻留指向、指令态波束宽度等。
 * 当整域与叶子同时出现时，先应用整域再应用叶子，叶子具有最终优先级。
 */
struct ONEQ_API RadarRuntimeConfigPatch {
  bool has_mission{false};
  config::RadarMissionConfig mission{};

  bool has_policy{false};
  config::RadarPolicyConfig policy{};

  bool has_environment{false};
  environment::EnvironmentRuntimeConfigPatch environment{};

  bool has_work_mode{false};
  RadarWorkMode work_mode{RadarWorkMode::kTws};

  bool has_scan_center_deg{false};
  AzimuthElevationDeg scan_center_deg{};

  bool has_dwell_center_deg{false};
  AzimuthElevationDeg dwell_center_deg{};

  bool has_commanded_beamwidth_deg{false};
  CommandedBeamwidthDeg commanded_beamwidth_deg{};

  bool has_commanded_beamwidth_enabled{false};
  bool commanded_beamwidth_enabled{false};

  bool has_sensor_enabled{false};
  bool sensor_enabled{true};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_RUNTIME_CONFIG_PATCH_H_
