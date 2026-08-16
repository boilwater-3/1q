/**
 * @file ArRuntimeConfigPatch.h
 * @brief 机载雷达运行期配置补丁类型集合。
 *
 * 运行期可变参数补丁的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_RUNTIME_CONFIG_PATCH_H_

#include <cstdint>

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/config/ArMissionConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using config::AzimuthElevationDeg;
using config::CommandedBeamwidthDeg;
using config::ArWorkMode;

/**
 * @brief EnvironmentRuntimeConfigPatch 描述运行期可变环境参数补丁。
 *
 * @par 类型合约
 * - 仅包含运行期可变更的环境参数项。
 * - 每个字段配备对应的 has_* 布尔标志，未设置的项不参与更新。
 * - 支持的补丁项：scenario_config。
 * - 解析时遵循原子语义：整个补丁要么全部生效，要么全部拒绝。
 */
struct ONEQ_API EnvironmentRuntimeConfigPatch {
  bool has_scenario_config{false};                /**< 是否更新环境场景输入 */
  EnvironmentScenarioConfig scenario_config{};    /**< 运行期环境场景输入 */
};

/**
 * @brief ArRuntimeConfigPatch 描述运行期可变参数补丁。
 *
 * @note "可外部调整"定义：调用方可在不重建 `ArSession` 的前提下，
 * 通过 `ArSession::TryApplyRuntimeConfig(...)` 暂存修改，并在下一次成功周期
 * 提交前统一生效。
 *
 * 支持两类运行期更新：
 * 1) 整域覆盖：`mission`、`policy`、`environment`；
 * 2) 叶子覆盖：工作子模式、扫描/驻留指向、指令态波束宽度、传感器开关等。
 * 整域与叶子同时出现时，先应用整域再应用叶子，叶子具有最终优先级。
 * 电源状态仅由叶子 `has_sensor_enabled` 控制（COMMON-OQ-4 收敛）：
 * `has_mission` 不改变电源，mission 域在类型层面已无电源字段（字段提升）。
 */
struct ONEQ_API ArRuntimeConfigPatch {
  bool has_mission{false};
  config::ArMissionConfig mission{};

  bool has_policy{false};
  config::ArPolicyConfig policy{};

  bool has_environment{false};
  EnvironmentRuntimeConfigPatch environment{};

  bool has_work_mode{false};
  ArWorkMode work_mode{ArWorkMode::kTws};

  bool has_scan_center_deg{false};
  AzimuthElevationDeg scan_center_deg{};

  bool has_dwell_center_deg{false};
  AzimuthElevationDeg dwell_center_deg{};

  /**
   * @brief 指定跟踪目标（STT 航迹跟随指向的目标句柄）。
   * @note STT 指向来源优先级（冻结语义，不得改序）：
   *   1) 显式 dwell_center_deg 非零：最终指向 = scan_center + dwell（现状语义，最高优先）；
   *   2) 指定目标存在且其航迹 confirmed：最终指向 = 航迹位置换算的 az/el（雷达局部系），
   *      dwell 视为零偏移（雷达自动跟随自身航迹，外部无需提供目标角度）；
   *   3) 其余（未指定/航迹未确认/丢失/不存在）：最终指向 = scan_center（现状 STT 行为）。
   * @note 指定仅在 work_mode == kStt 时被消费；TWS/TAS 下忽略。
   * @note 指定航迹非 confirmed 时，STT 自动回退 TWS（生效模式为每周期派生，见
   *       ArCycleResult::effective_work_mode / designation_reverted_to_tws）。
   * @note designated_external_target_id == 0 表示清除指定。
   */
  bool has_designated_target_id{false};
  std::uint64_t designated_external_target_id{0U};

  bool has_commanded_beamwidth_deg{false};
  CommandedBeamwidthDeg commanded_beamwidth_deg{};

  bool has_commanded_beamwidth_enabled{false};
  bool commanded_beamwidth_enabled{false};

  bool has_sensor_enabled{false};
  bool sensor_enabled{true};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_RUNTIME_CONFIG_PATCH_H_
