/**
 * @file RadarEquations.cpp
 * @brief 雷达物理方程纯函数库实现。
 * @details 核心公式提取自参考算法 RadarProcess.cpp，
 *          并与 Skolnik《Introduction to Radar Systems》及
 *          Richards《Exact and Approximate Detection Probability Formulas》交叉验证。
 */

#include "1q/airborne_radar/signal/detection/RadarEquations.h"

#include <cmath>
#include <algorithm>

// Boost.Math 不完全 Gamma 函数（header-only）
#include <boost/math/special_functions/gamma.hpp>

namespace airborne_radar {
namespace signal {
namespace detection {

namespace {

/// @brief 光速 (m/s)。
const float kLightSpeed = 3.0e8f;

/// @brief 玻尔兹曼常数 (J/K)。
const float kBoltzmann = 1.38064852e-23f;

/// @brief IEEE 标准参考温度 (K)。
const float kRefTemperature = 290.0f;

/// @brief π 常数。
const float kPi = 3.14159265358979323846f;

/// @brief 防止 log10(0) 的极小值保护。
const float kEpsilon = 1e-30f;

/// @brief 将线性值转为 dB。
float LinearToDb(float linear) {
  if (linear <= kEpsilon) {
    return 10.0f * std::log10(kEpsilon);
  }
  return 10.0f * std::log10(linear);
}

/// @brief 将 dB 转为线性值。
float DbToLinear(float db) {
  return std::pow(10.0f, db / 10.0f);
}

/// @brief 钳位到 [0, 1]。
float ClampPd(float pd) {
  if (pd < 0.0f) return 0.0f;
  if (pd > 1.0f) return 1.0f;
  return pd;
}

}  // namespace

// ===========================================================================
// 回波功率、噪声、积累、测量精度（不变）
// ===========================================================================

/// @brief 计算带指定单程天线增益的回波功率（dBW）。
/// @param tx 发射机配置。
/// @param one_way_gain_db 单程天线增益（dB）。
/// @param rcs_m2 目标 RCS（m^2）。
/// @param range_m 目标斜距（m）。
/// @param propagation_loss_db 传播损耗（dB）。
/// @return 接收回波功率（dBW）。
float RadarEquations::ComputeEchoPowerWithGain_dBW(
    const TransmitterConfig& tx,
    float one_way_gain_db,
    float rcs_m2,
    float range_m,
    float propagation_loss_db) {
  if (range_m <= 0.0f || rcs_m2 <= 0.0f) {
    return -300.0f;
  }

  const float wavelength_m = kLightSpeed / tx.frequency_hz;
  const float pt_db = LinearToDb(tx.peak_power_w);
  const float lambda_db = LinearToDb(wavelength_m);
  const float r_db = LinearToDb(range_m);
  const float rcs_db = LinearToDb(rcs_m2);
  const float total_loss_db = tx.transmit_loss_db + propagation_loss_db;

  const float pr_dbw = pt_db
      + one_way_gain_db
      + one_way_gain_db
      + 2.0f * lambda_db
      + rcs_db
      - 30.0f * std::log10(4.0f * kPi)
      - 4.0f * r_db
      - total_loss_db;

  return pr_dbw;
}

/// @brief 使用主瓣增益计算回波功率（dBW）。
/// @param tx 发射机配置。
/// @param ant 天线配置。
/// @param rcs_m2 目标 RCS（m^2）。
/// @param range_m 目标斜距（m）。
/// @param propagation_loss_db 传播损耗（dB）。
/// @return 接收回波功率（dBW）。
float RadarEquations::ComputeEchoPower_dBW(
    const TransmitterConfig& tx,
    const AntennaConfig& ant,
    float rcs_m2,
    float range_m,
    float propagation_loss_db) {
  return ComputeEchoPowerWithGain_dBW(
      tx, ant.main_beam_gain_db, rcs_m2, range_m, propagation_loss_db);
}

/// @brief 计算接收机热噪声功率（W）。
/// @param tx 发射机配置（带宽使用）。
/// @param rx 接收机配置（噪声系数使用）。
/// @return 热噪声功率（W）。
float RadarEquations::ComputeThermalNoisePower_W(
    const TransmitterConfig& tx,
    const ReceiverConfig& rx) {
  const float noise_figure_linear = DbToLinear(rx.noise_figure_db);
  return kBoltzmann * kRefTemperature * tx.bandwidth_hz
         * noise_figure_linear;
}

/// @brief 计算脉冲积累增益（线性值）。
/// @param pulse_count 脉冲数量。
/// @param coherent_integration 是否相参积累。
/// @return 积累增益线性值。
float RadarEquations::ComputeIntegrationGain(
    int pulse_count,
    bool coherent_integration) {
  if (pulse_count <= 0) {
    return 1.0f;
  }
  const float n = static_cast<float>(pulse_count);
  if (coherent_integration) {
    return n;
  }
  return std::sqrt(n);
}

/// @brief 估计距离误差标准差（m）。
/// @param snr_db 信噪比（dB）。
/// @param bandwidth_hz 信号带宽（Hz）。
/// @return 距离误差标准差（m）。
float RadarEquations::ComputeRangeErrorStdDev(
    float snr_db,
    float bandwidth_hz) {
  const float range_resolution = 0.5f * kLightSpeed / bandwidth_hz;
  const float kMinSnrDb = -10.0f;
  if (snr_db < kMinSnrDb) {
    return range_resolution * 1.5777f;
  }
  const float snr_linear = DbToLinear(snr_db);
  const float std_dev = 0.5f * range_resolution / std::sqrt(snr_linear);
  const float kRangeBias_m = 20.0f;
  return std_dev + kRangeBias_m;
}

/// @brief 估计角度误差标准差（rad）。
/// @param snr_db 信噪比（dB）。
/// @param beamwidth_rad 波束宽度（rad）。
/// @return 角度误差标准差（rad）。
float RadarEquations::ComputeAngleErrorStdDev(
    float snr_db,
    float beamwidth_rad) {
  const float kMinSnrDb = -10.0f;
  if (snr_db < kMinSnrDb) {
    return beamwidth_rad;
  }
  const float snr_linear = DbToLinear(snr_db);
  const float std_dev = 0.317f * beamwidth_rad / std::sqrt(snr_linear);
  const float angle_bias = beamwidth_rad / 30.0f;
  return std_dev + angle_bias;
}

// ===========================================================================
// 检测门限 T（方波检测器，N 脉冲非相参积累）
// ===========================================================================

/// @brief 计算给定虚警率与脉冲数对应的检测门限。
/// @param pfa 虚警概率。
/// @param num_pulses 脉冲数。
/// @return 方波检测门限。
double RadarEquations::ComputeThreshold(double pfa, int num_pulses) {
  if (pfa <= 0.0 || pfa >= 1.0) {
    pfa = 1e-6;
  }
  if (num_pulses <= 1) {
    // N=1: P_fa = exp(-T) → T = -ln(P_fa)
    return -std::log(pfa);
  }
  // N>1: 求解 Q(N, T) = P_fa
  // boost::math::gamma_q_inv(a, q) 返回 x 使得 Q(a,x) = q
  return boost::math::gamma_q_inv(
      static_cast<double>(num_pulses), pfa);
}

// ===========================================================================
// 广义 Marcum Q 函数（Poisson 加权 gamma_q 级数）
// ===========================================================================

/// @brief 计算广义 Marcum Q 函数。
/// @param order 阶数 M。
/// @param a 非中心参数。
/// @param b 检测门限相关参数。
/// @return Q_M(a,b) 的数值结果。
double RadarEquations::MarcumQ(int order, double a, double b) {
  // Q_M(a, b) = Σ_{k=0}^∞ [e^{-λ} · λ^k / k!] · Q(M+k, b²/2)
  // 其中 λ = a²/2
  //
  // 数值稳定性：从 Poisson 分布峰值 k₀ ≈ ⌊λ⌋ 处开始，
  // 向两侧累加。避免从 k=0 开始导致 exp(-λ) 下溢。

  if (b <= 0.0) return 1.0;
  if (a < 0.0) a = 0.0;

  // 极大信噪比情况（极远大于检测门限），由于 Poisson 分布的方差导致两翼截断误差会显现出数值不稳定
  // 根据渐进性，当 a >> b 时，检测概率必然趋于 1.0
  if (a > b + 20.0) return 1.0;

  const double lambda = a * a / 2.0;
  const double x = b * b / 2.0;

  const int kMaxIter = 500;
  const double kConvergence = 1e-12;

  // 从 Poisson 峰值处开始
  const int k0 = static_cast<int>(lambda);

  // 计算 k0 处的 log(Poisson(k0, λ))，然后以此为基准
  // log P(k, λ) = -λ + k·ln(λ) - ln(k!)
  // 使用 lgamma 计算 ln(k!) = lgamma(k+1)
  auto log_poisson = [lambda](int k) -> double {
    if (k == 0) return -lambda;
    return -lambda + k * std::log(lambda) - std::lgamma(k + 1);
  };

  double sum = 0.0;

  // 向右累加：k = k0, k0+1, k0+2, ...
  {
    double log_pk = log_poisson(k0);
    for (int k = k0; k < k0 + kMaxIter; ++k) {
      if (k > k0) {
        log_pk += std::log(lambda / static_cast<double>(k));
      }
      const double pk = std::exp(log_pk);
      const double gq = boost::math::gamma_q(
          static_cast<double>(order + k), x);
      const double term = pk * gq;
      sum += term;
      if (term < kConvergence && k > k0 + 2) break;
    }
  }

  // 向左累加：k = k0-1, k0-2, ..., 0
  {
    double log_pk = log_poisson(k0 > 0 ? k0 - 1 : 0);
    for (int k = (k0 > 0 ? k0 - 1 : -1); k >= 0; --k) {
      if (k < k0 - 1) {
        log_pk += std::log(static_cast<double>(k + 1) / lambda);
      }
      const double pk = std::exp(log_pk);
      const double gq = boost::math::gamma_q(
          static_cast<double>(order + k), x);
      const double term = pk * gq;
      sum += term;
      if (term < kConvergence && k < k0 - 2) break;
    }
  }

  // 钳位
  if (sum < 0.0) return 0.0;
  if (sum > 1.0) return 1.0;
  return sum;
}

// ===========================================================================
// Swerling 0~4 全模型检测概率（Richards 精确公式）
// ===========================================================================

/// @brief 计算 Swerling 模型下检测概率。
/// @param snr_db 信噪比（dB）。
/// @param pfa 虚警概率。
/// @param model Swerling 起伏模型。
/// @param num_pulses 脉冲积累数。
/// @return 检测概率（[0,1]）。
float RadarEquations::ComputeDetectionProbability(
    float snr_db,
    float pfa,
    SwerlingModel model,
    int num_pulses) {
  if (pfa <= 0.0f || pfa >= 1.0f) {
    pfa = 1e-6f;
  }
  if (num_pulses < 1) {
    num_pulses = 1;
  }

  const double chi = static_cast<double>(DbToLinear(snr_db));
  const int N = num_pulses;
  const double T = ComputeThreshold(static_cast<double>(pfa), N);
  double pd = 0.0;

  switch (model) {
    // -----------------------------------------------------------------
    // Swerling 0（非起伏 / 确定性 RCS）
    // Pd = Q_N(√(2Nχ), √(2T))  — Marcum Q
    // -----------------------------------------------------------------
    case kSwerling0: {
      const double a = std::sqrt(2.0 * N * chi);
      const double b = std::sqrt(2.0 * T);
      pd = MarcumQ(N, a, b);
      break;
    }

    // -----------------------------------------------------------------
    // Swerling 1（扫描间慢起伏，Rayleigh RCS）
    // N=1 精确: Pd = exp(-T/(1+χ))    [Richards/M&M eq.3-52]
    // N≥2 精确: Pd = Q(N-1, V) + C^{N-1} · [Q(N-1, V/C) - Q(N-1, V)]
    //          V = T/(1+Nχ),  C = Nχ/(1+Nχ)
    //          Q(a,x) = gamma_q(a, x)    [Richards Table 2]
    // -----------------------------------------------------------------
    case kSwerling1: {
      const double total_snr = N * chi;  // N 个脉冲的总 SNR
      if (N == 1) {
        pd = std::exp(-T / (1.0 + chi));
      } else {
        const double V = T / (1.0 + total_snr);
        const double C = total_snr / (1.0 + total_snr);
        const double a_shape = static_cast<double>(N - 1);

        const double q1 = boost::math::gamma_q(a_shape, V);

        // V/C = T/(Nχ)，当 χ→0 时趋于无穷
        if (C < 1e-15) {
          pd = q1;
        } else {
          const double q2 = boost::math::gamma_q(a_shape, V / C);
          pd = q1 + std::pow(C, a_shape) * (q2 - q1);
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    // Swerling 2（脉冲间快起伏，Rayleigh RCS）
    // 精确: Pd = Q(N, T/(1+χ)) = gamma_q(N, T/(1+χ))
    // [Richards Table 2, DiFranco & Rubin]
    // -----------------------------------------------------------------
    case kSwerling2: {
      const double cT = T / (1.0 + chi);
      pd = boost::math::gamma_q(static_cast<double>(N), cT);
      break;
    }

    // -----------------------------------------------------------------
    // Swerling 3（扫描间慢起伏，卡方 k=4 RCS）
    // N=1 精确: Pd = (1 + 2T/(2+χ)) · exp(-2T/(2+χ))  [M&M eq.3-55]
    // N≥2: 使用与 Swerling 1 类似的结构，但有效自由度翻倍
    //       V = 2T/(2+Nχ),  C = Nχ/(2+Nχ)
    //       Pd = Q(2(N-1), V) + C^{2(N-1)} · [Q(2(N-1), V/C) - Q(2(N-1),V)]
    //            + (2(N-1)) · C^{2(N-1)-1} · (1-C)
    //            · [Q(2(N-1)-1, V/C) - Q(2(N-1)-1, V)]
    // -----------------------------------------------------------------
    case kSwerling3: {
      const double total_snr = N * chi;  // N 个脉冲的总 SNR
      if (N == 1) {
        const double u = 2.0 * T / (2.0 + chi);
        pd = (1.0 + u) * std::exp(-u);
      } else {
        const double V = 2.0 * T / (2.0 + total_snr);
        const double C = total_snr / (2.0 + total_snr);
        const double M = 2.0 * (N - 1);

        if (C < 1e-15) {
          pd = boost::math::gamma_q(M, V);
        } else {
          const double q_M_V = boost::math::gamma_q(M, V);
          const double q_M_VC = boost::math::gamma_q(M, V / C);
          const double c_pow_M = std::pow(C, M);
          pd = q_M_V + c_pow_M * (q_M_VC - q_M_V);

          // 修正项（卡方 k=4 的额外自由度贡献）
          if (M >= 2.0) {
            const double q_M1_V = boost::math::gamma_q(M - 1.0, V);
            const double q_M1_VC = boost::math::gamma_q(M - 1.0, V / C);
            pd += M * std::pow(C, M - 1.0) * (1.0 - C)
                  * (q_M1_VC - q_M1_V);
          }
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    // Swerling 4（脉冲间快起伏，卡方 k=4 RCS）
    // 精确: Pd = gamma_q(2N, 2T/(2+χ))
    //          + (2N)·C·(1-C)^{2N-1} · [partial correction]
    // 简化为等效双自由度形式:
    //   Pd = gamma_q(2N, 2T/(2+χ))
    //      · (1 + correction_term)
    // [Richards Table 2, M&M Appendix A]
    // -----------------------------------------------------------------
    case kSwerling4: {
      // Swerling 4 精确公式（脉冲间快起伏，χ²(k=4) RCS）:
      // 每脉冲独立的 χ²(k=4) RCS → 等效 2 自由度/脉冲
      // Pd = gamma_q(2N, 2T/(2+χ))
      // [DiFranco & Rubin, Robertson 1967]
      //
      // N=1 时退化为 (1+u)·exp(-u) = Swerling 3 单脉冲结果 ✓
      const double u = 2.0 * T / (2.0 + chi);
      pd = boost::math::gamma_q(2.0 * N, u);
      break;
    }

    default: {
      // 未知模型，回退到 Swerling 0 逻辑
      const double a = std::sqrt(2.0 * N * chi);
      const double b = std::sqrt(2.0 * T);
      pd = MarcumQ(N, a, b);
      break;
    }
  }

  return ClampPd(static_cast<float>(pd));
}

// ===========================================================================
// 蒙特卡洛判决
// ===========================================================================

/// @brief 根据检测概率执行蒙特卡洛门限判决。
/// @param detection_prob 检测概率。
/// @param rng 随机数引擎。
/// @return 命中判决结果。
bool RadarEquations::ThresholdDecision(
    float detection_prob,
    std::mt19937& rng) {
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);
  const float r = dist(rng);
  return r <= detection_prob;
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
