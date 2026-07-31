/**
 * @file SarMissionConfig.h
 * @brief 定义 SAR 任务几何与成像网格配置。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_MISSION_CONFIG_H_
#define ONEQ_SAR_CONFIG_SAR_MISSION_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace sar {
namespace config {

/**
 * @brief L3 BP 逐脉冲聚焦所需的多航点（waypoint）配置项。
 */
struct ONEQ_API SarWaypointConfig {
  double time_from_session_start_s{0.0}; /**< 相对会话起始的时刻（s） */
  double latitude_deg{0.0};              /**< 纬度（deg） */
  double longitude_deg{0.0};             /**< 经度（deg） */
  double altitude_m{0.0};                /**< 海拔高度（m） */
};

using SarWaypointConfigList = std::vector<SarWaypointConfig>;

/**
 * @brief SAR 条带成像任务配置。
 */
struct ONEQ_API SarMissionConfig {
  double scene_center_latitude_deg{0.0};
  double scene_center_longitude_deg{0.0};
  double scene_center_altitude_m{0.0};
  double nominal_slant_range_m{15000.0};
  double platform_speed_mps{180.0};
  std::uint32_t range_sample_count{4096U};
  std::uint32_t azimuth_pulse_count{1024U};
  double desired_ground_range_resolution_m{1.5};
  double desired_azimuth_resolution_m{1.5};
  double l2_velocity_error_stddev_x_mps{0.0};
  double l2_velocity_error_stddev_y_mps{0.0};
  double l2_velocity_error_stddev_z_mps{0.0};
  std::uint32_t l2_random_seed{0U};
  SarWaypointConfigList l3_waypoints{};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_MISSION_CONFIG_H_
