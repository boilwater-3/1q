/**
 * @file SbirsMissionConfig.h
 * @brief 定义 SBIRS-inspired 任务与视场参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

enum class ONEQ_API SbirsWorkMode { kStandby = 0, kWideSearch, kSearchAndStare };

struct ONEQ_API SbirsMissionConfig {
  SbirsWorkMode work_mode{SbirsWorkMode::kSearchAndStare};
  bool sensor_enabled{true};
  float wide_field_fov_az_deg{20.0f};
  float wide_field_fov_el_deg{20.0f};
  float narrow_field_fov_az_deg{2.0f};
  float narrow_field_fov_el_deg{2.0f};
  float scan_start_az_deg{-60.0f};
  float scan_end_az_deg{60.0f};
  float scan_center_el_deg{0.0f};
  float scan_rate_deg_per_sec{10.0f};
  float min_range_m{1.0e3f};
  float max_range_m{5.0e7f};
  float frame_rate_hz{10.0f};
  float narrow_cue_latency_s{0.0f};
  float narrow_pointing_settle_error_deg{0.0f};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_MISSION_CONFIG_H_
