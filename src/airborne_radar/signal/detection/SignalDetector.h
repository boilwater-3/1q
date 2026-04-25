/**
 * @file SignalDetector.h
 * @brief 定义封装物理化回波评估与探测判决的信号检测器。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_

#include <limits>
#include <random>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/detection/RadarEquations.h"

namespace airborne_radar {
namespace signal {
namespace detection {
/**
 * @brief 单目标探测结果。
 */
struct DetectionResult {
  float echo_power_dbw{-300.0f}; /**< 接收回波功率 (dBW) */
  float snr_db{-100.0f};         /**< 信噪比 (dB) */
  float detection_prob{0.0f};    /**< 检测概率 Pd */
  bool detected{false};          /**< 是否达到门限 */
};
/**
 * @brief 目标回波特征上下文。
 */
struct TargetReturn {
  float rcs_m2{0.0f};  /**< 目标 RCS (m²) */
  float range_m{0.0f}; /**< 目标到雷达斜距 (m) */
  config::profiles::SwerlingModel swerling_type{
      config::profiles::SwerlingModel::kSwerling0}; /**< 目标的 Swerling 起伏模型 */
};
/**
 * @brief 环境噪声上下文。
 */
struct EnvironmentState {
  float propagation_loss_db{0.0f}; /**< 大气传播往返损耗 (dB) */
  float clutter_noise_w{0.0f};     /**< 杂波噪声功率 (W) */
  float jam_noise_w{0.0f};         /**< 干扰噪声功率 (W) */
};
/**
 * @brief SignalDetector 封装物理化的回波评估与探测判决。
 *
 * 它组合 RadarEquations 纯函数完成一条完整的物理检测链路：
 *   回波功率预算 → SNR 计算 → 检测概率 → 蒙特卡洛判决
 *
 * 通过构造函数注入内部工程探测配置雷达参数，
 * 热噪声功率底在构造时一次性预计算。
 */
class SignalDetector {
 public:
  /**
   * @brief 使用探测配置构造检测器。
   * @param config 探测配置
   */
  explicit SignalDetector(config::engineering::DetectionConfig config);
  /**
   * @brief 更新探测配置并重算热噪声底。
   * @param config 探测配置
   */
  void UpdateConfig(config::engineering::DetectionConfig config);
  /**
   * @brief 对单个目标执行完整检测链。
   * @param target               目标回波特征上下文
   * @param env                  环境噪声上下文
   * @param one_way_antenna_gain_db 单程天线增益；若为 NaN 则回退到配置中的主瓣峰值增益
   * @param pulse_count          检测脉冲数 N
   * @return 探测结果
   */
  DetectionResult Detect(const TargetReturn& target, const EnvironmentState& env,
                         float one_way_antenna_gain_db = std::numeric_limits<float>::quiet_NaN(),
                         int pulse_count = 1);
  /**
   * @brief 设置随机种子（用于确定性回归测试）。
   * @param seed 随机数种子
   */
  void SetRandomSeed(unsigned int seed);

 private:
  config::engineering::DetectionConfig config_; /**< 探测配置 */
  float thermal_noise_w_;                       /**< 预计算的接收机热噪声底 (W) */
  std::mt19937 rng_;                            /**< 确定性随机数引擎 */
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_
