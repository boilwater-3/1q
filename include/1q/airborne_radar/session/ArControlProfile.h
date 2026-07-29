/**
 * @file ArControlProfile.h
 * @brief 机载雷达控制 profile 状态类型。
 *
 * 下一周期生效的雷达控制状态的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CONTROL_PROFILE_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CONTROL_PROFILE_H_

#include <cstdint>

#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief ArControlProfile 表示下一周期生效的雷达控制状态。
 */
struct ONEQ_API ArControlProfile {
  std::uint64_t version{0};             /**< 配置版本号，每次 reducer 生成新 profile 时递增 */
  bool enable_lpi_power_control{false}; /**< 是否启用 LPI 功率控制 */
  float lpi_power_scale{1.0f};          /**< LPI 功率比例 */
  bool enable_lpi_beamforming{false};   /**< 是否启用 LPI 波束形成 */
  float lpi_dwell_scale{1.0f};          /**< LPI 驻留比例 */
  bool enable_agility_frequency{false}; /**< 是否启用频率捷变 */
  std::uint8_t agility_frequency_hop_phase{0}; /**< 频率捷变相位（0/1），用于解耦跳频方向与版本号 */
  bool enable_sidelobe_canceller{false};       /**< 是否启用旁瓣对消 */
  bool enable_adaptive_beamforming{false};     /**< 是否启用自适应波束形成 */
  bool enable_eccm_rejitter{false};            /**< 是否启用 ECCM 重频抖动 */
  float eccm_burnthrough_gain{1.0f};           /**< ECCM 烧穿增益倍率 */
  bool enable_anti_rgpo_leading_edge{false};   /**< 是否启用前沿跟踪对抗 RGPO */
  bool enable_anti_vgpo_acceleration_bound{false}; /**< 是否启用加速度限幅对抗 VGPO */
  bool enable_anti_false_target_discrimination{false}; /**< 是否启用假目标鉴别 */
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CONTROL_PROFILE_H_
