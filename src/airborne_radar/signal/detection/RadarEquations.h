// Copyright 2026. All Rights Reserved.
//
// Description: 雷达物理方程纯函数库，提供回波功率、噪声底、
// 测量精度和检测概率等无状态计算。
// 所有公式均与 Skolnik《Introduction to Radar Systems》交叉验证。

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_RADAR_EQUATIONS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_RADAR_EQUATIONS_H_

#include <random>
#include <cmath>

namespace airborne_radar {
namespace signal {
namespace detection {

// ---------------------------------------------------------------------------
// 子配置结构体（仿真工程标准术语）
// ---------------------------------------------------------------------------

/// @brief 发射机配置参数。
struct TransmitterConfig {
  float peak_power_w{1e6f};     ///< 峰值发射功率 (W)
  float frequency_hz{3e9f};     ///< 工作载频 (Hz)
  float bandwidth_hz{4.5e6f};   ///< 信号带宽 (Hz)
  float pulse_width_s{13e-6f};  ///< 脉冲宽度 (s)
  float prf_hz{300.0f};         ///< 脉冲重复频率 (Hz)
  float transmit_loss_db{3.5f}; ///< 馈线/发射系统损耗 (dB)
};

/// @brief 天线配置参数。
struct AntennaConfig {
  float main_beam_gain_db{35.0f}; ///< 主瓣增益 (dB)
  float az_beamwidth_deg{4.0f};   ///< 方位波束宽度 (°)
  float el_beamwidth_deg{4.0f};   ///< 俯仰波束宽度 (°)
};

/// @brief 接收机配置参数。
struct ReceiverConfig {
  float noise_figure_db{4.0f}; ///< 噪声系数 (dB)
  float receive_loss_db{2.0f}; ///< 接收系统损耗 (dB)
};

/// @brief 检测策略参数。
struct DetectionPolicy {
  float cfar_pfa{1e-6f};     ///< 恒虚警概率
  float min_snr_db{-10.0f};  ///< SNR 硬截断下限 (dB)
};

/// @brief 完整雷达系统配置（组合上述子配置）。
struct RadarSystemConfig {
  TransmitterConfig transmitter; ///< 发射机
  AntennaConfig antenna;         ///< 天线
  ReceiverConfig receiver;       ///< 接收机
  DetectionPolicy detection;     ///< 检测策略
};

// ---------------------------------------------------------------------------
// RCS 起伏模型枚举 (Swerling Cases)
// ---------------------------------------------------------------------------

/// @brief Swerling RCS 起伏模型类型。
/// - 0/1/3: 扫描间慢起伏（一个驻留期内 RCS 恒定）
/// - 2/4:   脉冲间快起伏（每个脉冲独立 RCS）
/// - 1/2:   Rayleigh 分布（多个等强散射体）, PDF: (1/σ̄)·exp(-σ/σ̄)
/// - 3/4:   卡方 k=4 分布（一个主散射体+多个小散射体）, PDF: (4σ/σ̄²)·exp(-2σ/σ̄)
enum SwerlingModel {
  kSwerling0 = 0,  ///< 无起伏（确定性 RCS / Swerling V）
  kSwerling1 = 1,  ///< 扫描间慢起伏，Rayleigh
  kSwerling2 = 2,  ///< 脉冲间快起伏，Rayleigh
  kSwerling3 = 3,  ///< 扫描间慢起伏，卡方 k=4
  kSwerling4 = 4   ///< 脉冲间快起伏，卡方 k=4
};

// ---------------------------------------------------------------------------
// 雷达物理方程（纯函数集合）
// ---------------------------------------------------------------------------

/// @brief 无状态的雷达物理计算函数集合。
/// 所有方法为 static，不持有任何内部状态。
struct RadarEquations {
  /// @brief 单站雷达方程（对数域），计算接收回波功率。
  /// 公式: Pr_dBW = Pt_dB + 2*Gt_dB + 2*λ_dB + σ_dB
  ///                - 30*log10(4π) - 4*R_dB - L_sys
  /// Skolnik eq.1.6 对数展开形式。
  /// @param tx       发射机参数
  /// @param ant      天线参数
  /// @param rcs_m2   目标雷达散射截面 (m²)
  /// @param range_m  目标斜距 (m)
  /// @param propagation_loss_db 大气传播往返损耗 (dB)
  /// @return 接收回波功率 (dBW)
  static float ComputeEchoPower_dBW(
      const TransmitterConfig& tx,
      const AntennaConfig& ant,
      float rcs_m2,
      float range_m,
      float propagation_loss_db);

  /// @brief 接收机热噪声功率底: N₀ = k·T₀·B·F。
  /// T₀ = 290K (IEEE 标准参考温度)。
  /// @param tx 发射机参数（取带宽）
  /// @param rx 接收机参数（取噪声系数）
  /// @return 热噪声功率 (W)
  static float ComputeThermalNoisePower_W(
      const TransmitterConfig& tx,
      const ReceiverConfig& rx);

  /// @brief 脉冲积累增益因子。
  /// 相参积累: G = N。
  /// 非相参积累: G = √N。
  /// 过渡区（pulse_count < min_coherent_pulses 时）: G = 0.5·(√N + N)。
  /// Skolnik Ch.2 积累理论。
  /// @param pulse_count 积累脉冲数
  /// @param coherent_integration 是否为相参积累
  /// @return 积累增益因子（线性值）
  static float ComputeIntegrationGain(int pulse_count,
                                      bool coherent_integration);

  /// @brief 测距标准差 σ_R (m)。
  /// 工程近似: σ_R ≈ 0.5·δ_R / √(SNR_linear) + bias，
  /// 其中 δ_R = c/(2B) 为距离分辨力。
  /// 近似 Skolnik eq.11.2 Cramér-Rao 下界（省略 √2 系数）。
  /// @param snr_db    信噪比 (dB)
  /// @param bandwidth_hz 信号带宽 (Hz)
  /// @return 距离测量标准差 (m)
  static float ComputeRangeErrorStdDev(float snr_db,
                                       float bandwidth_hz);

  /// @brief 测角标准差 σ_θ (rad)。
  /// 工程近似: σ_θ ≈ 0.317·θ_bw / √(SNR_linear) + θ_bw/30。
  /// 近似 Skolnik eq.11.27（k≈1.6 经验常数）。
  /// @param snr_db         信噪比 (dB)
  /// @param beamwidth_rad  波束宽度 (rad)
  /// @return 角度测量标准差 (rad)
  static float ComputeAngleErrorStdDev(float snr_db,
                                       float beamwidth_rad);

  /// @brief 支持 Swerling 0~4 全模型、多脉冲非相参积累的检测概率。
  /// 精确公式来源: Richards《Exact and Approximate Detection Probability
  /// Formulas in FRSP》(2018), DiFranco & Rubin, Skolnik Ch.2。
  /// 内部使用 boost::math::gamma_q / gamma_q_inv 计算正则化不完全 Gamma 函数。
  /// @param snr_db      每脉冲信噪比 (dB)
  /// @param pfa         虚警概率
  /// @param model       Swerling 起伏模型
  /// @param num_pulses  非相参积累脉冲数 (N ≥ 1)
  /// @return 检测概率 Pd ∈ [0, 1]
  static float ComputeDetectionProbability(float snr_db, float pfa,
                                           SwerlingModel model,
                                           int num_pulses);

  /// @brief 计算方波检测器的检测门限 T。
  /// N=1: T = -ln(P_fa)。
  /// N>1: 求解 Q(N, T) = P_fa，即 gamma_q_inv(N, P_fa)。
  /// @param pfa         虚警概率
  /// @param num_pulses  积累脉冲数 (N ≥ 1)
  /// @return 检测门限 T
  static double ComputeThreshold(double pfa, int num_pulses);

  /// @brief 广义 Marcum Q 函数 Q_M(a, b)。
  /// 用于 Swerling 0 精确检测概率计算。
  /// 采用 Poisson 加权的 gamma_q 级数展开实现。
  /// @param order  阶数 M (= 积累脉冲数 N)
  /// @param a      √(2·N·SNR_linear)
  /// @param b      √(2·T)
  /// @return Q_M(a, b) ∈ [0, 1]
  static double MarcumQ(int order, double a, double b);

  /// @brief 蒙特卡洛探测判决。
  /// 生成 [0,1) 均匀随机数，与 Pd 比较。
  /// @param detection_prob 检测概率 Pd
  /// @param rng            随机数引擎（外部注入保证确定性）
  /// @return 是否探测成功
  static bool ThresholdDecision(float detection_prob,
                                std::mt19937& rng);
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_RADAR_EQUATIONS_H_
