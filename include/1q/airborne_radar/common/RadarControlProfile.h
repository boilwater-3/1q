// Copyright 2026. All Rights Reserved.
//
// @file RadarControlProfile.h
// @brief 定义信号层在下一周期读取的控制真值。

#ifndef AIRBORNE_RADAR_COMMON_RADAR_CONTROL_PROFILE_H_
#define AIRBORNE_RADAR_COMMON_RADAR_CONTROL_PROFILE_H_

#include <cstdint>

namespace airborne_radar {
namespace common {

/// @brief RadarControlProfile 表示下一周期生效的雷达控制状态。
struct RadarControlProfile {
  /// @brief 配置版本号，每次 reducer 生成新 profile 时递增。
  std::uint64_t version{0};

  /// @brief 是否启用 LPI 功率控制。
  bool enable_lpi_power_control{false};

  /// @brief LPI 功率比例。
  float lpi_power_scale{1.0f};

  /// @brief 是否启用 LPI 波束形成。
  bool enable_lpi_beamforming{false};

  /// @brief LPI 驻留比例。
  float lpi_dwell_scale{1.0f};

  /// @brief 是否启用频率捷变。
  bool enable_agility_frequency{false};

  /// @brief 是否启用旁瓣对消。
  bool enable_sidelobe_canceller{false};

  /// @brief 是否启用自适应波束形成。
  bool enable_adaptive_beamforming{false};

  /// @brief 是否启用 ECCM 重频抖动。
  bool enable_eccm_rejitter{false};

  /// @brief ECCM 烧穿增益倍率。
  float eccm_burnthrough_gain{1.0f};

  /// @brief LPI 域剩余保持周期，仅供 reducer 运行态使用。
  std::uint32_t lpi_hold_cycles_remaining{0};

  /// @brief ECCM 域剩余保持周期，仅供 reducer 运行态使用。
  std::uint32_t eccm_hold_cycles_remaining{0};

  /// @brief LPI 域释放后的冷却周期，仅供 reducer 运行态使用。
  std::uint32_t lpi_cooldown_cycles_remaining{0};

  /// @brief ECCM 域释放后的冷却周期，仅供 reducer 运行态使用。
  std::uint32_t eccm_cooldown_cycles_remaining{0};
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_RADAR_CONTROL_PROFILE_H_
