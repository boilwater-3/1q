/**
 * @file RirRuntimeConfigPatch.h
 * @brief 远程识别雷达运行期配置补丁类型集合。
 *
 * 运行期可变参数补丁的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirRuntimeConfigPatch 描述运行期可变参数补丁。
 *
 * @note "可外部调整"定义：调用方可在不重建 `RirSession` 的前提下，
 * 通过 `RirSession::TryApplyRuntimeConfig(...)` 暂存修改，并在下一次成功周期
 * 提交前统一生效。
 *
 * 支持两类运行期更新：
 * 1) 整域覆盖：`policy`（识别策略整域，无叶子级 recognition patch 字段）；
 * 2) 叶子覆盖：工作模式、传感器开关。
 * 电源状态仅由叶子 `has_sensor_enabled` 控制（COMMON-OQ-4 收敛）。
 */
struct ONEQ_API RirRuntimeConfigPatch {
  bool has_work_mode{false};
  RirWorkMode work_mode{RirWorkMode::kStby};

  bool has_policy{false};
  RirPolicyConfig policy{};

  bool has_sensor_enabled{false};
  bool sensor_enabled{true};
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_
