/**
 * @file SbirsRadiometry.h
 * @brief SBIRS-inspired 红外辐射与 SNR 标量链路。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_RADIOMETRY_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_RADIOMETRY_H_

#include "1q/sbirs_sensor/config/SbirsHardwareConfig.h"

namespace sbirs_sensor {
namespace foundation {

double ComputePlanckRadiance(double wavelength_um, double temperature_k);
double ComputeBandRadiance(double wavelength_lower_um, double wavelength_upper_um,
                           double temperature_k);
double ComputeReceivedPowerW(double band_radiance, double projected_area_m2, double range_m,
                             double aperture_m, double optical_transmission,
                             double path_transmittance, double detector_quantum_efficiency);
double ComputeInfraredSnrLinear(double received_power_w,
                                const config::SbirsHardwareConfig& hardware);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_RADIOMETRY_H_
