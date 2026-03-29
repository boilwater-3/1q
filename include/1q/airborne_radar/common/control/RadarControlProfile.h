/**
 * @file RadarControlProfile.h
 * @brief 定义信号层在下一周期读取的控制真值。
 */

#ifndef AIRBORNE_RADAR_COMMON_RADAR_CONTROL_PROFILE_H_
#define AIRBORNE_RADAR_COMMON_RADAR_CONTROL_PROFILE_H_

#include <cstdint>

namespace airborne_radar {
namespace common {
namespace control {

/**
 * @brief RadarControlProfile 表示下一周期生效的雷达控制状态。
 */
struct RadarControlProfile {
  std::uint64_t version{0};                   /**< 配置版本号，每次 reducer 生成新 profile 时递增 */
  bool enable_lpi_power_control{false};       /**< 是否启用 LPI 功率控制 */
  float lpi_power_scale{1.0f};                /**< LPI 功率比例 */
  bool enable_lpi_beamforming{false};         /**< 是否启用 LPI 波束形成 */
  float lpi_dwell_scale{1.0f};                /**< LPI 驻留比例 */
  bool enable_agility_frequency{false};       /**< 是否启用频率捷变 */
  bool enable_sidelobe_canceller{false};      /**< 是否启用旁瓣对消 */
  bool enable_adaptive_beamforming{false};    /**< 是否启用自适应波束形成 */
  bool enable_eccm_rejitter{false};           /**< 是否启用 ECCM 重频抖动 */
  float eccm_burnthrough_gain{1.0f};          /**< ECCM 烧穿增益倍率 */
};

}  // namespace control
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_RADAR_CONTROL_PROFILE_H_
