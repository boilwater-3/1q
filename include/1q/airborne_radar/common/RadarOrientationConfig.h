// Copyright 2026. All Rights Reserved.
//
// Description: 定义机载雷达方向、扫描窗口与波束指向相关配置结构。

#ifndef AIRBORNE_RADAR_COMMON_RADAR_ORIENTATION_CONFIG_H_
#define AIRBORNE_RADAR_COMMON_RADAR_ORIENTATION_CONFIG_H_

namespace airborne_radar {
namespace common {

/// @brief EulerAnglesDeg 表示欧拉角定义（单位：度）。
/// 约定:
/// - yaw   表示偏航 / 方位角
/// - pitch 表示俯仰角
/// - roll  表示滚转角
struct EulerAnglesDeg {
  /// @brief 偏航 / 方位角（单位：度）。
  float yaw_deg{0.0f};

  /// @brief 俯仰角（单位：度）。
  float pitch_deg{0.0f};

  /// @brief 滚转角（单位：度）。
  float roll_deg{0.0f};
};

/// @brief AzimuthElevationDeg 表示方位-俯仰二维角度（单位：度）。
struct AzimuthElevationDeg {
  /// @brief 方位角（单位：度）。
  float az_deg{0.0f};

  /// @brief 俯仰角（单位：度）。
  float el_deg{0.0f};
};

/// @brief AzimuthElevationLimitsDeg 表示方位-俯仰扫描限位（单位：度）。
struct AzimuthElevationLimitsDeg {
  /// @brief 方位最小扫描角（单位：度）。
  float az_min_deg{-60.0f};

  /// @brief 方位最大扫描角（单位：度）。
  float az_max_deg{60.0f};

  /// @brief 俯仰最小扫描角（单位：度）。
  float el_min_deg{-30.0f};

  /// @brief 俯仰最大扫描角（单位：度）。
  float el_max_deg{30.0f};
};

/// @brief CommandedBeamwidthDeg 表示指令态波束宽度配置（单位：度）。
/// @note 该配置表示战术控制/ECCM/LPI 等逻辑下发的瞬时波束宽度，
///       不等同于雷达体制的名义波束宽度。
struct CommandedBeamwidthDeg {
  /// @brief 指令态方位波束宽度（单位：度）。
  float commanded_az_beamwidth_deg{4.0f};

  /// @brief 指令态俯仰波束宽度（单位：度）。
  float commanded_el_beamwidth_deg{4.0f};
};

/// @brief StabilizationMode 表示雷达波束稳定方式。
enum class StabilizationMode {
  /// @brief 随机体稳定，波束方向随平台姿态变化。
  kBodyStabilized = 0,

  /// @brief 对惯性空间稳定，尽量保持相对惯性坐标系方向不变。
  kInertialStabilized = 1,

  /// @brief 对地稳定，适用于对地搜索或地形跟随场景。
  kGroundStabilized = 2
};

/// @brief RadarOrientationConfig 表示机载雷达方向与扫描相关配置。
/// 建议组合关系如下:
/// actual_beam_pointing =
///     platform_attitude + mount_angles_deg + scan_center_deg + dwell_center_deg
struct RadarOrientationConfig {
  /// @brief 雷达相对机体坐标系的安装偏置角。
  EulerAnglesDeg mount_angles_deg;

  /// @brief 搜索窗口中心方向。
  AzimuthElevationDeg scan_center_deg;

  /// @brief 机械扫描限位。
  AzimuthElevationLimitsDeg mechanical_scan_limits_deg;

  /// @brief 电子扫描限位。
  AzimuthElevationLimitsDeg electronic_scan_limits_deg;

  /// @brief 当前波束驻留中心。
  AzimuthElevationDeg dwell_center_deg;

  /// @brief 当前指令态瞬时波束宽度。
  CommandedBeamwidthDeg commanded_beamwidth_deg;

  /// @brief 是否启用指令态波束宽度覆盖。
  /// @note false 时，探测/测量等链路应回退到雷达名义波束宽度。
  bool commanded_beamwidth_enabled{false};

  /// @brief 波束稳定方式。
  StabilizationMode stabilization_mode{
      StabilizationMode::kBodyStabilized};
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_RADAR_ORIENTATION_CONFIG_H_
