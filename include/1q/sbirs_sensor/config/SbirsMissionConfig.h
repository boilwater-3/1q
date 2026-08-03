/**
 * @file SbirsMissionConfig.h
 * @brief 定义 SBIRS-inspired 任务与视场参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

/** @brief 传感器工作模式：待机、仅 WFOV 宽视场搜索、搜索+凝视协同。 */
enum class ONEQ_API SbirsWorkMode { kStandby = 0, kWideSearch, kSearchAndStare };

/** @brief WFOV 环形扫描方向；扫描速率始终为非负标量。 */
enum class ONEQ_API SbirsScanDirection { kIncreasingAzimuth = 0, kDecreasingAzimuth };

/**
 * @brief SBIRS-inspired 任务与视场参数。
 * @note 纯数据类型 (POD)。WFOV/NFOV 视场、扫描范围与速率、距离门控和帧率共同定义
 *       pipeline 几何门控与扫描相位推进；凝视相关参数仅在 NFOV 通道生效。
 *       电源状态由 `SbirsSessionConfig::sensor_enabled` 顶层字段唯一承载
 *       （COMMON-OQ-4 收敛）。
 */
struct ONEQ_API SbirsMissionConfig {
  SbirsWorkMode work_mode{SbirsWorkMode::kSearchAndStare}; /**< 传感器工作模式 */
  float wide_field_fov_az_deg{20.0f};                      /**< WFOV 方位视场（FOV），单位 deg */
  float wide_field_fov_el_deg{20.0f};                      /**< WFOV 俯仰视场（FOV），单位 deg */
  float narrow_field_fov_az_deg{2.0f};                     /**< NFOV 方位视场（IFOV），单位 deg */
  float narrow_field_fov_el_deg{2.0f};                     /**< NFOV 俯仰视场（IFOV），单位 deg */
  float scan_start_az_deg{-60.0f};                         /**< WFOV 扫描方位起点，单位 deg */
  float scan_span_deg{120.0f}; /**< WFOV 有向扫描跨度，范围 (0, 360] deg */
  SbirsScanDirection scan_direction{SbirsScanDirection::kIncreasingAzimuth}; /**< WFOV 扫描方向 */
  float scan_center_el_deg{0.0f};                          /**< WFOV 扫描中心俯仰角，单位 deg */
  float scan_rate_deg_per_sec{10.0f};                      /**< WFOV 扫描速率，单位 deg/s */
  float min_range_m{1.0e3f};                               /**< 距离门控下限 Dmin，单位 m */
  float max_range_m{5.0e7f};                               /**< 距离门控上限 Dmax，单位 m */
  float frame_rate_hz{10.0f};                              /**< 帧率，单位 Hz */
  float narrow_cue_latency_s{0.0f};                        /**< WFOV→NFOV cue 延迟，单位 s */
  float narrow_pointing_settle_error_deg{0.0f};            /**< NFOV 指向稳定误差，单位 deg */
  float narrow_pointing_max_slew_rate_deg_per_sec{30.0f};  /**< NFOV 光轴最大转速，单位 deg/s */
  float narrow_pointing_settle_tolerance_deg{0.01f};       /**< NFOV 光轴稳定容差，单位 deg */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_
