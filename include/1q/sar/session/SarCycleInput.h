/**
 * @file SarCycleInput.h
 * @brief 定义 SAR 会话单周期输入载荷。
 */

#ifndef ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_H_
#define ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace sar {
namespace session {

/**
 * @brief SAR 平台状态输入。
 */
struct ONEQ_API SarPlatformState {
  double time_s{0.0};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  double velocity_north_mps{0.0};
  double velocity_east_mps{0.0};
  double velocity_down_mps{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
};

/**
 * @brief SAR 点目标输入。
 */
struct ONEQ_API SarPointTarget {
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  double radar_cross_section_dbsm{0.0};
};

using SarPointTargetList = std::vector<SarPointTarget>;

/**
 * @brief SAR 单周期输入。
 */
struct ONEQ_API SarCycleInput {
  std::uint32_t cycle_index{0U};
  double dt_sec{1.0};
  SarPlatformState platform{};
  SarPointTargetList point_targets{};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_H_
