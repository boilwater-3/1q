/**
 * @file RirRadarEquations.cpp
 * @brief 远程识别雷达内部链路预算实现（副本改写自 RadarEquations.cpp）。
 */

#include "remote_identification_radar/internal/RirRadarEquations.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/Constants.h"
#include "common/numerics/NumericGuard.h"

namespace remote_identification_radar {
namespace internal {

namespace {

/** @brief IEEE 标准参考温度 (K)。 */
const float kRefTemperature = 290.0f;
/** @brief 参考脉宽（单位：s），用于把峰值功率映射到单脉冲能量尺度。 */
const float kReferencePulseWidthS = 13.0e-6f;
using oneq::common::numerics::kLog10Floor;

/** @brief 将线性值转为 dB；≤ 下限时钳位避免 log10(0)。 */
float LinearToDb(float linear) {
  if (linear <= kLog10Floor) {
    return 10.0f * std::log10(kLog10Floor);
  }
  return 10.0f * std::log10(linear);
}

/** @brief 计算相对参考脉宽的单脉冲能量缩放（最小钳位到机器精度量级）。 */
float ComputePulseEnergyScale(const config::hardware::RirTransmitterConfig& tx) {
  constexpr float kMinEnergyScale = 1e-12f;
  if (!std::isfinite(tx.pulse_width_s) || tx.pulse_width_s <= 0.0f) {
    return kMinEnergyScale;
  }
  return std::max(tx.pulse_width_s / kReferencePulseWidthS, kMinEnergyScale);
}

/** @brief 将 dB 转为线性值。 */
float DbToLinear(float db) { return std::pow(10.0f, db / 10.0f); }

}  // namespace

float RirRadarEquations::ComputeEchoPowerWithGain_dBW(
    const config::hardware::RirTransmitterConfig& tx, float one_way_gain_db, float rcs_m2,
    float range_m, float propagation_loss_db) {
  if (range_m <= 0.0f || rcs_m2 <= 0.0f || tx.frequency_hz <= 0.0f) {
    return -300.0f;
  }

  const float wavelength_m =
      static_cast<float>(oneq::common::numerics::kLightSpeed) / tx.frequency_hz;
  const float pt_db = LinearToDb(tx.peak_power_w);
  const float pulse_energy_scale_db = LinearToDb(ComputePulseEnergyScale(tx));
  const float lambda_db = LinearToDb(wavelength_m);
  const float r_db = LinearToDb(range_m);
  const float rcs_db = LinearToDb(rcs_m2);
  const float total_loss_db = tx.transmit_loss_db + propagation_loss_db;

  const float pr_dbw = pt_db + pulse_energy_scale_db + one_way_gain_db + one_way_gain_db +
                       2.0f * lambda_db + rcs_db -
                       30.0f * std::log10(4.0f * static_cast<float>(oneq::common::numerics::kPi)) -
                       4.0f * r_db - total_loss_db;

  return pr_dbw;
}

float RirRadarEquations::ComputeEchoPower_dBW(
    const config::hardware::RirTransmitterConfig& tx, const config::hardware::RirAntennaConfig& ant,
    float rcs_m2, float range_m, float propagation_loss_db) {
  return ComputeEchoPowerWithGain_dBW(tx, ant.main_beam_gain_db, rcs_m2, range_m,
                                      propagation_loss_db);
}

float RirRadarEquations::ComputeThermalNoisePower_W(
    const config::hardware::RirTransmitterConfig& tx,
    const config::hardware::RirReceiverConfig& rx) {
  const float noise_figure_linear = DbToLinear(rx.noise_figure_db);
  return static_cast<float>(oneq::common::numerics::kBoltzmann) * kRefTemperature *
         tx.bandwidth_hz * noise_figure_linear;
}

}  // namespace internal
}  // namespace remote_identification_radar
