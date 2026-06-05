/**
 * @file SarMissionConfig.h
 * @brief 定义 SAR 任务几何与成像网格配置。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_MISSION_CONFIG_H_
#define ONEQ_SAR_CONFIG_SAR_MISSION_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"

namespace sar {
namespace config {

/**
 * @brief SAR 条带成像任务配置。
 */
struct ONEQ_API SarMissionConfig {
  double scene_center_latitude_deg{0.0};
  double scene_center_longitude_deg{0.0};
  double scene_center_altitude_m{0.0};
  double nominal_slant_range_m{15000.0};
  double synthetic_aperture_time_s{2.0};
  double platform_speed_mps{180.0};
  std::uint32_t range_sample_count{2048U};
  std::uint32_t azimuth_pulse_count{1024U};
  double desired_ground_range_resolution_m{1.5};
  double desired_azimuth_resolution_m{1.5};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_MISSION_CONFIG_H_
