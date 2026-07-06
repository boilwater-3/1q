#include "sbirs_sensor/foundation/SbirsErrorModel.h"

#include <cmath>

namespace sbirs_sensor {
namespace foundation {
namespace {

const double kPi = 3.14159265358979323846;
const double kDegToRad = kPi / 180.0;

}  // namespace

double SbirsRandomSource::NextUniform() {
  // xorshift32（Marsaglia）。state 永远非零（构造/Restore 时夹紧）。
  unsigned int x = state_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  state_ = x;
  // 映射到 [0,1)，避免返回正好 1.0 破坏 Box-Muller 的 log。
  return (static_cast<double>(state_) / 4294967296.0);
}

double SbirsRandomSource::NextStandardNormal() {
  // Box-Muller 变换。u2 保证 > 0，避免 log(0)。
  const double u1 = NextUniform();
  double u2 = NextUniform();
  if (u2 <= 0.0) {
    u2 = 1.0e-12;
  }
  const double radius = std::sqrt(-2.0 * std::log(u2));
  const double angle = 2.0 * kPi * u1;
  // 仅消费一个正态样本，丢弃另一个以保持确定性推进。
  return radius * std::cos(angle);
}

double RefractionErrorDeg(double range_m, float elevation_deg) {
  if (range_m <= 0.0) {
    return 0.0;
  }
  const double cos_beta = std::cos(static_cast<double>(elevation_deg) * kDegToRad);
  // 仰角接近 0° 时 cosβ→1；俯仰角在天基 LOS 下通常较大，cos 不会发散。
  // 1.5e-6 为红外波段折射角系数（design 2.10）。
  const double denominator = range_m * std::fabs(cos_beta);
  if (denominator <= 0.0) {
    return 0.0;
  }
  return 1.5e-6 / denominator;
}

double DynamicLagErrorDeg(float target_angular_rate_deg_per_sec, float detector_bandwidth_hz) {
  if (detector_bandwidth_hz <= 0.0f) {
    return 0.0;
  }
  return static_cast<double>(target_angular_rate_deg_per_sec) / (2.0 * kPi * detector_bandwidth_hz);
}

SbirsErrorBearing ApplyAngularErrorModel(const config::SbirsErrorModelConfig& model,
                                         SbirsRandomSource* random, float true_azimuth_deg,
                                         float true_elevation_deg, double true_range_m,
                                         float target_angular_rate_deg_per_sec) {
  SbirsErrorBearing bearing;
  // 5 类误差：轨道、姿态、视场为高斯随机；折射、滞后为确定性公式。
  // 各 sigma 合成按 RSS（独立零均值高斯方差可加）。
  const double orbit =
      model.orbit_sigma_deg > 0.0f && random != nullptr
          ? model.orbit_sigma_deg * random->NextStandardNormal()
          : 0.0;
  const double attitude =
      model.attitude_sigma_deg > 0.0f && random != nullptr
          ? model.attitude_sigma_deg * random->NextStandardNormal()
          : 0.0;
  const double fov =
      model.fov_sigma_deg > 0.0f && random != nullptr
          ? model.fov_sigma_deg * random->NextStandardNormal()
          : 0.0;
  // 兼容字段：当 orbit/attitude/fov sigma 都为 0 时，回退到合并 angular_sigma_deg，
  // 保持现有测试与配置默认行为不变。
  const double legacy =
      (model.orbit_sigma_deg == 0.0f && model.fov_sigma_deg == 0.0f)
          ? model.angular_sigma_deg
          : 0.0;

  const double refraction = RefractionErrorDeg(true_range_m, true_elevation_deg);
  const double lag =
      DynamicLagErrorDeg(target_angular_rate_deg_per_sec, model.detector_bandwidth_hz);

  // 加法合成（design 2.10 式 1111-1113）。
  bearing.azimuth_deg =
      static_cast<float>(static_cast<double>(true_azimuth_deg) + orbit + attitude + fov + legacy +
                         refraction + lag);
  bearing.elevation_deg =
      static_cast<float>(static_cast<double>(true_elevation_deg) + orbit + attitude + fov +
                         legacy + refraction + lag);

  // 乘法合成（距离误差）：d_meas = d_true · (1 + Δd_rand)。
  double range_factor = 0.0;
  if (model.range_fraction_sigma > 0.0f && random != nullptr) {
    range_factor = model.range_fraction_sigma * random->NextStandardNormal();
  }
  bearing.range_m = true_range_m * (1.0 + range_factor);
  return bearing;
}

}  // namespace foundation
}  // namespace sbirs_sensor
