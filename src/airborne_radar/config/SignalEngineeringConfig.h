/**
 * @file SignalEngineeringConfig.h
 * @brief 定义 Signal 子系统内部工程参数类型（非公开 API）。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_ENGINEERING_SIGNAL_ENGINEERING_CONFIG_H_
#define AIRBORNE_RADAR_SRC_CONFIG_ENGINEERING_SIGNAL_ENGINEERING_CONFIG_H_

#include <cstdint>

#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "1q/airborne_radar/config/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {
namespace engineering {

using AntennaPatternModelType = detection::AntennaPatternModelType;
using KalmanUpdateBackend = tracking::KalmanUpdateBackend;

// Detection domain types — 1:1 aliases to public detection:: types (extends P7 pattern).
using TransmitterConfig = detection::TransmitterConfig;
using AntennaPatternConfig = detection::AntennaPatternConfig;
using AntennaConfig = detection::AntennaConfig;
using ReceiverConfig = detection::ReceiverConfig;
using DetectionPolicy = detection::DetectionPolicyConfig;
using RcsPhysicsConfig = detection::RcsPhysicsConfig;
using DetectionConfig = detection::DetectionConfig;

// Tracking domain type — 1:1 alias to public tracking:: type (extends P9+P10 pattern).
using TrackingConfig = tracking::TrackingConfig;

struct LifecycleConfig {
  std::uint32_t confirm_hits{3};
  std::uint32_t max_miss_before_lost{2};
  std::uint32_t max_lost_cycles{5};
};

struct LifecycleRuntimeConfig {
  LifecycleConfig lifecycle_config{};
  bool enable_imm_lifecycle{false};
};

}  // namespace engineering
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_ENGINEERING_SIGNAL_ENGINEERING_CONFIG_H_