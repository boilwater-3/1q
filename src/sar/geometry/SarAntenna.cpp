#include "sar/geometry/SarAntenna.h"
#include "sar/geometry/SarGeometry.h"

#include <cmath>

namespace sar {
namespace geometry {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = kPi * 0.5;
constexpr double kFourPi = kPi * 4.0;

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

double AntennaPattern(const AntennaParams& antenna, double wavelength_m,
                      double off_boresight_az_rad, double off_boresight_el_rad) {
  if (!std::isfinite(wavelength_m) || wavelength_m <= 0.0) {
    return 0.0;
  }

  // 后瓣判定(任一轴离轴超过 90°)。
  const bool inside_back_lobe =
      std::fabs(off_boresight_az_rad) > kHalfPi || std::fabs(off_boresight_el_rad) > kHalfPi;
  if (inside_back_lobe) {
    return std::pow(10.0, antenna.backlobe_level_db / 20.0);
  }

  // 主瓣判定: 离轴角在半功率波束半宽内。
  const double half_bw_az = antenna.beam_width_azimuth_rad * 0.5;
  const double half_bw_el = antenna.beam_width_range_rad * 0.5;
  const bool inside_main_lobe =
      (half_bw_az <= 0.0 || std::fabs(off_boresight_az_rad) <= half_bw_az) &&
      (half_bw_el <= 0.0 || std::fabs(off_boresight_el_rad) <= half_bw_el);

  if (!inside_main_lobe) {
    return std::pow(10.0, antenna.max_sidelobe_level_db / 20.0);
  }

  // 主瓣方向图模型。
  switch (antenna.pattern_model) {
    case AntennaPatternModel::kGaussianMainLobe: {
      const double kLn2 = 0.6931471805599453;
      double az_factor = 1.0;
      double el_factor = 1.0;
      if (half_bw_az > 0.0) {
        const double norm_az = off_boresight_az_rad / half_bw_az;
        az_factor = std::exp(-kLn2 * norm_az * norm_az);
      }
      if (half_bw_el > 0.0) {
        const double norm_el = off_boresight_el_rad / half_bw_el;
        el_factor = std::exp(-kLn2 * norm_el * norm_el);
      }
      return az_factor * el_factor;
    }

    case AntennaPatternModel::kParabolicMainLobe: {
      double attenuation_db = 0.0;
      if (half_bw_az > 0.0) {
        const double norm_az = off_boresight_az_rad / half_bw_az;
        attenuation_db += 3.0 * norm_az * norm_az;
      }
      if (half_bw_el > 0.0) {
        const double norm_el = off_boresight_el_rad / half_bw_el;
        attenuation_db += 3.0 * norm_el * norm_el;
      }
      return std::pow(10.0, -attenuation_db / 20.0);
    }

    case AntennaPatternModel::kCosinePower: {
      const double kLnHalfSqrt2 = -0.34657359027997264;  // ln(1/√2)
      double az_factor = 1.0;
      double el_factor = 1.0;
      if (half_bw_az > 0.0) {
        const double cos_half = std::cos(half_bw_az);
        if (cos_half > 1e-12 && cos_half < 1.0) {
          const double k = kLnHalfSqrt2 / std::log(cos_half);
          const double cos_theta = std::cos(std::fabs(off_boresight_az_rad));
          az_factor = (cos_theta > 1e-12) ? std::pow(cos_theta, k) : 0.0;
        }
      }
      if (half_bw_el > 0.0) {
        const double cos_half = std::cos(half_bw_el);
        if (cos_half > 1e-12 && cos_half < 1.0) {
          const double k = kLnHalfSqrt2 / std::log(cos_half);
          const double cos_theta = std::cos(std::fabs(off_boresight_el_rad));
          el_factor = (cos_theta > 1e-12) ? std::pow(cos_theta, k) : 0.0;
        }
      }
      return az_factor * el_factor;
    }

    case AntennaPatternModel::kSincPattern:
    default: {
      double az_factor = 1.0;
      double el_factor = 1.0;
      if (std::isfinite(antenna.length_m) && antenna.length_m > 0.0) {
        const double arg = antenna.length_m * std::sin(off_boresight_az_rad) / wavelength_m;
        const double v = Sinc(arg);
        az_factor = v * v;  // sinc²
      }
      if (std::isfinite(antenna.width_m) && antenna.width_m > 0.0) {
        const double arg = antenna.width_m * std::sin(off_boresight_el_rad) / wavelength_m;
        const double v = Sinc(arg);
        el_factor = v * v;  // sinc²
      }
      return az_factor * el_factor;
    }
  }
}

double AzimuthPattern(const AntennaParams& antenna, double wavelength_m,
                      double off_boresight_rad) {
  return AntennaPattern(antenna, wavelength_m, off_boresight_rad, 0.0);
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
