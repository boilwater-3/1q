#include "sbirs_sensor/foundation/SbirsRadiometry.h"

#include <algorithm>
#include <cmath>

namespace sbirs_sensor {
namespace foundation {
namespace {

const double kPi = 3.14159265358979323846;

}  // namespace

double ComputeReceivedPowerW(double radiant_intensity_w_per_sr, double range_m,
                             double aperture_m, double optical_transmission,
                             double path_transmittance, double detector_quantum_efficiency) {
  if (radiant_intensity_w_per_sr < 0.0 || range_m <= 0.0 || aperture_m <= 0.0) {
    return 0.0;
  }
  const double aperture_area = kPi * aperture_m * aperture_m * 0.25;
  const double signal = radiant_intensity_w_per_sr * aperture_area;
  return signal * std::max(0.0, optical_transmission) * std::max(0.0, path_transmittance) *
         std::max(0.0, detector_quantum_efficiency) / (range_m * range_m);
}

double ComputeInfraredSnrLinear(double received_power_w,
                                const config::SbirsHardwareConfig& hardware) {
  const double signal_energy =
      std::max(0.0, received_power_w) * std::max(0.0f, hardware.integration_time_sec);
  const double noise = std::max(1.0e-18f, hardware.noise_equivalent_power_w);
  return signal_energy / noise;
}

}  // namespace foundation
}  // namespace sbirs_sensor
