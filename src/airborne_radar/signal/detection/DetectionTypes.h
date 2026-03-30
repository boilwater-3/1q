/**
 * @file DetectionTypes.h
 * @brief 检测域内部兼容类型桥接（迁移至 common::config 后保留）。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_DETECTION_TYPES_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_DETECTION_TYPES_H_

#include "1q/airborne_radar/config/SignalDetectionConfig.h"

namespace airborne_radar {
namespace signal {
namespace detection {

using TransmitterConfig = common::config::TransmitterConfig;
using AntennaConfig = common::config::AntennaConfig;
using ReceiverConfig = common::config::ReceiverConfig;
using DetectionPolicy = common::config::DetectionPolicy;
using RadarSystemConfig = common::config::RadarSystemConfig;
using SwerlingModel = common::config::SwerlingModel;

using common::config::kSwerling0;
using common::config::kSwerling1;
using common::config::kSwerling2;
using common::config::kSwerling3;
using common::config::kSwerling4;

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_DETECTION_TYPES_H_
