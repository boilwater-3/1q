/**
 * @file RirMissionConfig.h
 * @brief 远程识别雷达任务域主配置类型。
 *
 * 任务态、工作模式与识别任务作用范围/驻留参数的主头文件。
 * 识别任务作用距离与驻留参数原由策略域 `RirPolicyConfig::recognition` 平移
 * 承载，现已四域归位至 mission 域。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirWorkMode 表示远程识别雷达工作模式。
 *
 * 对应原 AR `ArWorkMode::kLrr` 的"是否处于识别驻留"语义：
 * `kIdentify` 下对航迹执行驻留观测与积累；`kStby` 下不积累，
 * 已有结论按 `result_hold_sec` 保持后过期。
 */
enum class ONEQ_API RirWorkMode {
  kStby = 0,     /**< 待机：不执行识别积累，结论保持至过期。 */
  kIdentify = 1  /**< 识别驻留：对重点航迹执行识别观测与积累。 */
};

/**
 * @brief RirMissionConfig 远程识别雷达任务域配置。
 * @note 电源状态由 `RirSessionConfig::sensor_enabled` 顶层承载，mission 域不含电源字段。
 */
struct ONEQ_API RirMissionConfig {
  RirWorkMode work_mode{RirWorkMode::kStby}; /**< [可外部调整] 当前工作模式。 */
  float max_range_m{300000.0f};              /**< 识别任务最大作用距离（m），>0。 */
  float recognition_dwell_sec{0.05f};        /**< 单次识别驻留时间（s），>0。 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_
