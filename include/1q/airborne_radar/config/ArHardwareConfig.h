/**
 * @file ArHardwareConfig.h
 * @brief AR module primary aliases for hardware configuration.
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_

#include "1q/airborne_radar/config/RadarHardwareConfig.h"

namespace airborne_radar {
namespace config {

namespace profiles {
using ArHardwareProfile = RadarHardwareProfile;
}  // namespace profiles

using ArHardwareConfig = RadarHardwareConfig;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_
