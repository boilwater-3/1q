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
#include "1q/foundation/scan_schedule_types.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

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
 * @brief RirScanConfig 扫描策略配置（与 AR 同一 common 扫描内核口径）。
 *
 * 空闲/任务间隙时驻留波束中心按扫描策略逐周期推进：common 内核在
 * `orientation.steerable_volume_deg` 相对限位上建波位，再经
 * `mission.scan_center_deg` 平移并方位归一化输出绝对指向；指定识别
 * 任务窗口内对准指定目标（限位执行见 boundaries.md）。波束步长 =
 * 生效波束宽度 × `step_scale`（TAS 密度等价于 step_scale=0.5）。
 */
struct ONEQ_API RirScanConfig {
  oneq::foundation::ScanStartPosition scan_start_position{
      oneq::foundation::ScanStartPosition::kLeftTop}; /**< 扫描起始象限。 */
  oneq::foundation::ScanSequence scan_sequence{
      oneq::foundation::ScanSequence::kAzimuthFirst}; /**< 二维扫描推进顺序。 */
  float step_scale{1.0f}; /**< 波位步长系数（波束宽度 × 系数），>0。 */
};

/**
 * @brief RirMissionConfig 远程识别雷达任务域配置。
 * @note 电源状态由 `RirSessionConfig::sensor_enabled` 顶层承载，mission 域不含电源字段。
 */
struct ONEQ_API RirMissionConfig {
  RirWorkMode work_mode{RirWorkMode::kStby}; /**< [可外部调整] 当前工作模式。 */
  RirAzimuthElevationDeg scan_center_deg{}; /**< [可外部调整] 转台当前朝向（ENU az/el）；默认 (0,0)=东水平。 */
  float max_range_m{300000.0f};              /**< 识别任务最大作用距离（m），>0。 */
  float recognition_dwell_sec{0.05f};        /**< 单次识别驻留时间（s），>0。 */
  RirScanConfig scan{};                      /**< 扫描策略（库内驻留调度器消费）。 */
  /**
   * @brief [可外部调整] 任务扫描子窗：用户指定的作战搜索扇区（az 相对 scan_center、
   *        el 绝对，deg）。默认无界 [-180,180]×[-90,90] = 不额外收窄。
   *
   * 与 `orientation.steerable_volume_deg`（硬件最大可扫描体积）分层：实际搜索扇区 =
   * scan_window ∩ steerable_volume——扫描波位在此扇区内推进，搜索态检测候选按此裁剪。
   * 落在子窗外、仍在 steerable_volume 内的目标仅在被「指定识别」时豁免子窗裁剪。
   */
  RirAzimuthElevationLimitsDeg scan_window_deg{-180.0f, 180.0f, -90.0f, 90.0f};
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_MISSION_CONFIG_H_
