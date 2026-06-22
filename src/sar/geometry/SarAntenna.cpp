#include "sar/geometry/SarAntenna.h"
#include "sar/geometry/SarGeometry.h"

#include <cmath>

namespace sar {
namespace geometry {

namespace {

constexpr double kFourPi = 12.566370614359172953850573533118011537;

}  // namespace

double AntennaGain(const AntennaParams& antenna, double wavelength_m) {
  if (!std::isfinite(wavelength_m) || wavelength_m <= 0.0 ||
      !std::isfinite(antenna.length_m) || antenna.length_m <= 0.0 ||
      !std::isfinite(antenna.width_m) || antenna.width_m <= 0.0) {
    return 0.0;
  }
  // G = 4π · L · W / λ² (理想口径)
  return kFourPi * antenna.length_m * antenna.width_m / (wavelength_m * wavelength_m);
}

double AzimuthPattern(const AntennaParams& antenna, double wavelength_m,
                      double off_boresight_rad) {
  if (!std::isfinite(wavelength_m) || wavelength_m <= 0.0 ||
      !std::isfinite(antenna.length_m) || antenna.length_m <= 0.0) {
    return 0.0;
  }
  const double arg = antenna.length_m * std::sin(off_boresight_rad) / wavelength_m;
  const double v = Sinc(arg);
  return v * v;  // sinc²
}

double SincPattern(double length_m, double wavelength_m, double off_boresight_rad) {
  if (!std::isfinite(wavelength_m) || wavelength_m <= 0.0 ||
      !std::isfinite(length_m) || length_m <= 0.0) {
    return 0.0;
  }
  const double arg = length_m * std::sin(off_boresight_rad) / wavelength_m;
  const double v = Sinc(arg);
  return v * v;
}

double SyntheticApertureTime(const AntennaParams& antenna, double slant_range_m,
                             double platform_velocity_mps) {
  if (!std::isfinite(slant_range_m) || slant_range_m <= 0.0 ||
      !std::isfinite(platform_velocity_mps) || platform_velocity_mps <= 0.0 ||
      !std::isfinite(antenna.length_m) || antenna.length_m <= 0.0) {
    return 0.0;
  }
  // 波长从波束宽度反推: θ_bw = λ / L, 若已提供 beam_width_azimuth_rad 优先使用
  // 否则用 0.03 m(X-band) 作为默认, 由外部传入(此处依赖 beam_width 或默认波长)
  // 简化: 假设 λ 已隐含在 beam_width 中, 故直接使用 beam_width_azimuth_rad
  const double beam_width_rad = antenna.beam_width_azimuth_rad > 0.0
                                    ? antenna.beam_width_azimuth_rad
                                    : 0.03 / antenna.length_m;  // 默认 X-band λ≈0.03 m
  return slant_range_m * beam_width_rad / platform_velocity_mps;
}

double AntennaResolution(const AntennaParams& antenna, double slant_range_m,
                         double wavelength_m, bool synthetic_aperture) {
  if (synthetic_aperture) {
    // 合成孔径: 理论分辨率 = 天线长度 / 2
    return antenna.length_m > 0.0 ? antenna.length_m * 0.5 : 0.0;
  }
  // 实孔径: ρ_az = R · λ / L
  if (!std::isfinite(wavelength_m) || wavelength_m <= 0.0 ||
      !std::isfinite(antenna.length_m) || antenna.length_m <= 0.0 ||
      !std::isfinite(slant_range_m) || slant_range_m <= 0.0) {
    return 0.0;
  }
  return slant_range_m * wavelength_m / antenna.length_m;
}

}  // namespace geometry
}  // namespace sar
