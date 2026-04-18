/**
 * @file RadarHardwareConfig.h
 * @brief 定义雷达硬件域公开配置。
 *
 * 硬件域承载探测链路固有能力参数。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_HARDWARE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_HARDWARE_CONFIG_H_

#include "1q/airborne_radar/config/expert/detection/DetectionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {
namespace expert {

using detection::AntennaConfig;
using detection::AntennaPatternConfig;
using detection::AntennaPatternModelType;
using detection::DetectionConfig;
using detection::DetectionPolicyConfig;
using detection::RcsPhysicsConfig;
using detection::ReceiverConfig;
using detection::TransmitterConfig;

}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

namespace airborne_radar {
namespace config {

using expert::AntennaConfig;
using expert::AntennaPatternConfig;
using expert::AntennaPatternModelType;
using expert::DetectionConfig;
using expert::DetectionPolicyConfig;
using expert::RcsPhysicsConfig;
using expert::ReceiverConfig;
using expert::TransmitterConfig;

/**
 * @brief 雷达硬件域配置。
 *
 * 当前阶段硬件域承载探测链路固有能力参数。
 */
struct ONEQ_API RadarHardwareConfig {
  DetectionConfig detection{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_HARDWARE_CONFIG_H_
