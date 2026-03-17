// Copyright 2026. All Rights Reserved.
//
// Description: 定义决策控制模块消费的来源信息结构体。

#ifndef AIRBORNE_RADAR_COMMON_DECISION_SOURCE_INFO_H_
#define AIRBORNE_RADAR_COMMON_DECISION_SOURCE_INFO_H_

namespace airborne_radar {
namespace common {

/// @brief LpiSourceInfo 表示供 LPI 模块消费的来源信息。
/// TODO 当前仅保留一个布尔量，后续根据需求扩展为更细粒度威胁来源描述。
struct LpiSourceInfo {
  /// @brief 当前周期目标中是否存在电子侦察飞行器或侦察类设备。
  bool has_recon_platform{false};
};

/// @brief EccmSourceInfo 表示供 ECCM 模块消费的干扰来源信息。
/// TODO 当前仅保留一个布尔量，后续可扩展为干扰类型、强度和来源方向等信息。
struct EccmSourceInfo {
  /// @brief 当前周期是否检测到干扰信号。
  bool has_jamming_signal{false};

  /// @brief 干扰功率估计（单位：dB）。
  float jammer_power_db{0.0f};

  /// @brief 干扰与当前工作频率的重叠度，范围 [0, 1]。
  float frequency_overlap_ratio{0.0f};

  /// @brief 干扰对当前 PRF 锁定的风险度，范围 [0, 1]。
  float prf_lock_risk{0.0f};

  /// @brief 干扰是否主要经由旁瓣进入。
  bool jammer_in_sidelobe{false};

  /// @brief 默认构造函数。
  EccmSourceInfo() = default;

  /// @brief 带参数构造函数。
  explicit EccmSourceInfo(bool has_jamming)
      : has_jamming_signal(has_jamming) {}
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_DECISION_SOURCE_INFO_H_
