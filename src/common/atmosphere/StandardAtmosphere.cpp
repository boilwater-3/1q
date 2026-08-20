#include "common/atmosphere/StandardAtmosphere.h"

#include <algorithm>
#include <cmath>

namespace oneq {
namespace common {
namespace atmosphere {

namespace {

// ISA 1976 物理常数
constexpr double kG0 = 9.80665;           // m/s^2
constexpr double kM = 0.0289644;          // kg/mol
constexpr double kR = 8.31447;            // J/(mol*K)
constexpr double kRSpecific = kR / kM;    // 287.05287 J/(kg*K)
constexpr double kGamma = 1.4;            // 空气比热比
constexpr double kREarth = 6356766.0;     // ISA 标准地球半径 (m)
constexpr double kMaxAltitudeM = 86000.0; // ISA 1976 上限

// ISA 1976 温度断点表（7 个层的 8 个边界）
// 每层定义：base_geopotential_m, base_temperature_K, lapse_rate_K_per_m
struct IsaLayer {
  double base_h;       // 层底位势高度 (m)
  double base_t;       // 层底温度 (K)
  double lapse_rate;   // 温度递减率 (K/m)，0 表示等温层
};

// 7 个层（对流层到中间层顶）
const IsaLayer kLayers[] = {
    {0.0, 288.15, -0.0065},       // 0–11 km: 对流层
    {11000.0, 216.65, 0.0},       // 11–20 km: 对流层顶（等温）
    {20000.0, 216.65, 0.001},     // 20–32 km: 平流层下层
    {32000.0, 228.65, 0.0028},    // 32–47 km: 平流层上层
    {47000.0, 270.65, 0.0},       // 47–51 km: 平流层顶（等温）
    {51000.0, 270.65, -0.0028},   // 51–71 km: 中间层下层
    {71000.0, 214.65, -0.002},    // 71–86 km: 中间层上层
};
const int kLayerCount = 7;

// 各层底的标准气压 (Pa)
const double kBasePressure[] = {
    101325.0,      // 0 km
    22632.1,       // 11 km
    5474.89,       // 20 km
    868.019,       // 32 km
    110.906,       // 47 km
    66.9389,       // 51 km
    3.95642,       // 71 km
};

// g0*M/(R) 常用组合
constexpr double kGmOverR = kG0 * kM / kR;  // ~0.03416

double GeometricToGeopotential(double h_m) {
  return kREarth * h_m / (kREarth + h_m);
}

double GeopotentialToGeometric(double h_m) {
  return kREarth * h_m / (kREarth - h_m);
}

int FindLayer(double geopotential_h) {
  for (int i = kLayerCount - 1; i >= 0; --i) {
    if (geopotential_h >= kLayers[i].base_h) {
      return i;
    }
  }
  return 0;
}

double ComputeTemperature(int layer, double delta_h) {
  return kLayers[layer].base_t + kLayers[layer].lapse_rate * delta_h;
}

double ComputePressure(int layer, double delta_h) {
  const double base_t = kLayers[layer].base_t;
  const double lr = kLayers[layer].lapse_rate;
  const double base_p = kBasePressure[layer];

  if (std::fabs(lr) < 1.0e-10) {
    // 等温层: P = P_b * exp(-g0*M*dh / (R*T_b))
    return base_p * std::exp(-kGmOverR * delta_h / base_t);
  } else {
    // 梯度层: P = P_b * (T_b / T)^(g0*M/(R*L))
    const double t = ComputeTemperature(layer, delta_h);
    const double exponent = kGmOverR / lr;
    return base_p * std::pow(base_t / t, exponent);
  }
}

}  // namespace

environment::AtmosphericState StandardAtmosphere::GetState(float altitude_m) const {
  const double safe_alt = std::max(0.0, static_cast<double>(altitude_m));

  // 几何高度 → 位势高度
  const double h_geo = GeometricToGeopotential(safe_alt);

  // 钳位到 ISA 1976 范围 [0, 86000]
  const double clamped_h = std::min(h_geo, kMaxAltitudeM);

  const int layer = FindLayer(clamped_h);
  const double delta_h = clamped_h - kLayers[layer].base_h;

  const double t = ComputeTemperature(layer, delta_h);
  const double p = ComputePressure(layer, delta_h);
  const double rho = p / (kRSpecific * t);
  const double a = std::sqrt(kGamma * kRSpecific * t);

  environment::AtmosphericState state;
  state.altitude_m = altitude_m;
  state.temperature_k = static_cast<float>(t);
  state.pressure_pa = static_cast<float>(p);
  state.density_kg_m3 = static_cast<float>(rho);
  state.speed_of_sound_mps = static_cast<float>(a);
  state.pressure_hpa = static_cast<float>(p / 100.0);
  return state;
}

environment::AtmosphericState StandardAtmosphere::GetSeaLevelState() const {
  return GetState(0.0f);
}

}  // namespace atmosphere
}  // namespace common
}  // namespace oneq
