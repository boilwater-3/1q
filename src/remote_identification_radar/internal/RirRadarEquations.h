/**
 * @file RirRadarEquations.h
 * @brief 远程识别雷达内部链路预算纯函数。
 *
 * 副本来源：`src/airborne_radar/signal/detection/RadarEquations.*`（审计基线
 * 96de367c）中识别链路消费的最小函数集（回波功率 + 热噪声底），类型换用
 * `RirHardwareConfig` 的 hardware 域结构；阶段 3 评估 common 化。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_RADAR_EQUATIONS_H_
#define REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_RADAR_EQUATIONS_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"

namespace remote_identification_radar {
namespace internal {

/**
 * @brief 无状态的雷达物理计算函数集合（识别链路子集）。
 */
struct RirRadarEquations {
  /**
   * @brief 单站雷达方程（对数域），按指定单程天线增益计算接收回波功率。
   * 公式: Pr_dBW = Pt_dB + 2*G_one_way_dB + 2*λ_dB + σ_dB
   *                - 30*log10(4π) - 4*R_dB - L_sys
   * @param tx 发射机参数
   * @param one_way_gain_db 当前波束指向下的单程天线增益
   * @param rcs_m2 目标雷达散射截面 (m²)
   * @param range_m 目标斜距 (m)
   * @param propagation_loss_db 大气传播往返损耗 (dB)
   * @return 接收回波功率 (dBW)
   */
  static float ComputeEchoPowerWithGain_dBW(const config::hardware::RirTransmitterConfig& tx,
                                            float one_way_gain_db, float rcs_m2, float range_m,
                                            float propagation_loss_db);

  /**
   * @brief 单站雷达方程（对数域），以天线主瓣峰值增益计算接收回波功率。
   * @param tx 发射机参数
   * @param ant 天线参数
   * @param rcs_m2 目标雷达散射截面 (m²)
   * @param range_m 目标斜距 (m)
   * @param propagation_loss_db 大气传播往返损耗 (dB)
   * @return 接收回波功率 (dBW)
   */
  static float ComputeEchoPower_dBW(const config::hardware::RirTransmitterConfig& tx,
                                    const config::hardware::RirAntennaConfig& ant, float rcs_m2,
                                    float range_m, float propagation_loss_db);

  /**
   * @brief 接收机热噪声功率底: N₀ = k·T₀·B·F。
   * T₀ = 290K (IEEE 标准参考温度)。
   * @param tx 发射机参数（取带宽）
   * @param rx 接收机参数（取噪声系数）
   * @return 热噪声功率 (W)
   */
  static float ComputeThermalNoisePower_W(const config::hardware::RirTransmitterConfig& tx,
                                          const config::hardware::RirReceiverConfig& rx);
};

}  // namespace internal
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_RADAR_EQUATIONS_H_
