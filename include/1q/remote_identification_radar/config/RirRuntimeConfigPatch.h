/**
 * @file RirRuntimeConfigPatch.h
 * @brief 远程识别雷达运行期配置补丁类型集合。
 *
 * 运行期可变参数补丁的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"
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
 * 1) 整域覆盖：`mission`、`policy`、`environment`（识别策略整域，无叶子级 recognition
 *    patch 字段）；
 * 2) 叶子覆盖：工作模式、传感器开关。
 * 整域与叶子同时出现时，先应用整域再应用叶子，叶子具有最终优先级。
 * 电源状态仅由叶子 `has_sensor_enabled` 控制（COMMON-OQ-4 收敛）：
 * `has_mission` 不改变电源，mission 域在类型层面已无电源字段。
 */
struct ONEQ_API RirRuntimeConfigPatch {
  bool has_mission{false};
  RirMissionConfig mission{};

  bool has_work_mode{false};
  RirWorkMode work_mode{RirWorkMode::kStby};

  bool has_policy{false};
  RirPolicyConfig policy{};

  bool has_environment{false};
  RirEnvironmentConfig environment{};

  bool has_sensor_enabled{false};
  bool sensor_enabled{true};

  /**
   * @brief 指定识别目标（限时识别任务的目标句柄，镜像 AR designation 语义）。
   * @note 任务语义（见 boundaries.md「指定识别任务（限时锁定）」）：
   *   - 指定仅在 work_mode == kIdentify 时被消费；kStby 下忽略；
   *   - 任务窗口内驻留对准指定目标（目标在场景时），直到识别达成
   *     （RirRecognitionState 达 kCategoryConfirmed/kModelConfirmed）——
   *     识别达成即任务完成，回到扫描策略；
   *   - 窗口耗尽仍未识别 → 任务作废（回到扫描），作废沿经
   *     `RirCycleResult::designation_revert_reason = kAcquisitionTimeout` 暴露；
   *   - `designation_duration_cycles == 0` 表示无限期（旧行为），直到外部清除；
   *   - `designated_external_target_id == 0` 表示清除指定。
   */
  bool has_designated_target_id{false};
  std::uint64_t designated_external_target_id{0U};

  /** @brief 限时识别窗口（周期数；0 = 无限期，旧行为；见上注）。 */
  bool has_designation_duration_cycles{false};
  std::uint32_t designation_duration_cycles{0U};
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_RUNTIME_CONFIG_PATCH_H_
