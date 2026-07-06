#include "sbirs_sensor/foundation/SbirsRadiometry.h"

#include <algorithm>
#include <cmath>

namespace sbirs_sensor {
namespace foundation {
namespace {

const double kPlanck = 6.62607015e-34;
const double kLightSpeed = 299792458.0;
const double kBoltzmann = 1.380649e-23;
const double kPi = 3.14159265358979323846;

}  // namespace

double ComputePlanckRadiance(double wavelength_um, double temperature_k) {
  if (wavelength_um <= 0.0 || temperature_k <= 0.0) {
    return 0.0;
  }
  const double wavelength_m = wavelength_um * 1.0e-6;
  const double numerator = 2.0 * kPlanck * kLightSpeed * kLightSpeed;
  const double exponent = kPlanck * kLightSpeed / (wavelength_m * kBoltzmann * temperature_k);
  const double denominator = std::pow(wavelength_m, 5.0) * (std::exp(exponent) - 1.0);
  if (denominator <= 0.0) {
    return 0.0;
  }
  return numerator / denominator;
}

double ComputeBandRadiance(double wavelength_lower_um, double wavelength_upper_um,
                           double temperature_k) {
  const double lower = std::min(wavelength_lower_um, wavelength_upper_um);
  const double upper = std::max(wavelength_lower_um, wavelength_upper_um);
  const double center = 0.5 * (lower + upper);
  const double width_m = std::max(0.0, upper - lower) * 1.0e-6;
  return ComputePlanckRadiance(center, temperature_k) * width_m;
}

double ComputeReceivedPowerW(double band_radiance, double projected_area_m2, double range_m,
                             double aperture_m, double optical_transmission,
                             double path_transmittance, double detector_quantum_efficiency) {
  if (range_m <= 0.0 || aperture_m <= 0.0) {
    return 0.0;
  }
  const double aperture_area = kPi * aperture_m * aperture_m * 0.25;
  const double signal = band_radiance * std::max(0.0, projected_area_m2) * aperture_area;
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
