#include "sbirs_sensor/foundation/SbirsNoiseModel.h"

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace sbirs_sensor {
namespace foundation {
namespace {

using oneq::common::numerics::kBoltzmann;
using oneq::common::numerics::kPi;

}  // namespace

SbirsNoiseStatistics ComputeBackgroundNoiseStatistics(const config::SbirsHardwareConfig& hardware) {
  SbirsNoiseStatistics stats;

  // 光子（散粒）噪声：背景辐射亮度 × 孔径面积 × 积分时间，取平方根（泊松统计）。
  // 背景辐射为 0 时该项为 0。
  if (hardware.background_radiance_w_sr_m2 > 0.0f && hardware.optical_aperture_m > 0.0f) {
    const double aperture_area = kPi * hardware.optical_aperture_m * hardware.optical_aperture_m * 0.25;
    // 假设 1 sr 等效视场立体角（标量链路简化）。
    const double background_power =
        static_cast<double>(hardware.background_radiance_w_sr_m2) * aperture_area;
    const double photons_equivalent =
        background_power * std::max(0.0f, hardware.integration_time_sec);
    stats.photon_noise_w = std::sqrt(std::max(0.0, photons_equivalent));
  }

  // 热噪声（Johnson 噪声等效功率）：与探测器温度、带宽相关。
  // NEP_thermal ~ sqrt(4 k_B T Δf)（简化量纲，单位 W）。Δf 由积分时间倒数近似。
  if (hardware.detector_temperature_k > 0.0f && hardware.integration_time_sec > 0.0f) {
    const double bandwidth = 1.0 / static_cast<double>(hardware.integration_time_sec);
    stats.thermal_noise_w = std::sqrt(4.0 * kBoltzmann *
                                      static_cast<double>(hardware.detector_temperature_k) *
                                      bandwidth);
  }

  // 读出噪声：直接取配置。
  stats.readout_noise_w = std::max(0.0f, hardware.readout_noise_rms_w);

  // 三项 RMS 合成。
  const double sum_sq = stats.photon_noise_w * stats.photon_noise_w +
                        stats.thermal_noise_w * stats.thermal_noise_w +
                        stats.readout_noise_w * stats.readout_noise_w;
  stats.total_noise_w = std::sqrt(std::max(0.0, sum_sq));
  return stats;
}

double ResolveEffectiveNoiseW(const config::SbirsHardwareConfig& hardware,
                              const SbirsNoiseStatistics& statistics) {
  // 三项分解噪声为 0 时回退到 NEP 标量，保持向后兼容。
  if (statistics.total_noise_w <= 0.0) {
    return std::max(1.0e-18f, hardware.noise_equivalent_power_w);
  }
  return statistics.total_noise_w;
}

}  // namespace foundation
}  // namespace sbirs_sensor
