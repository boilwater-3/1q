/**
 * @file RirSignalDetector.h
 * @brief 定义 RIR 物理化回波评估与统计级 CFAR 探测判决器。
 *
 * 副本来源：`src/airborne_radar/signal/detection/SignalDetector.*`（审计基线
 * 96de367c，阶段 2-M M4），类型换用 `Rir*`、物理函数走 `RirRadarEquations`。
 * 判决链与 AR 逐行一致：回波功率预算 → SNR → Swerling Pd（Pfa 门限，
 * 统计级 CFAR 口径，非 CA-CFAR，《能力边界界定》§3.1）→ 蒙特卡洛判决，
 * 附 `min_snr_db` 硬截断与 `min_detection_margin_db` 裕量门。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API；检测配置的 policy 域
 *       公开化随阶段 2-S 接线评估。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_SIGNAL_DETECTOR_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_SIGNAL_DETECTOR_H_

#include <limits>
#include <random>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"

namespace remote_identification_radar {
namespace dwell {

/** @brief 探测策略参数（副本：AR DetectionPolicy）。 */
struct RirDetectionPolicyConfig {
  float cfar_pfa{1e-6f};   /**< 恒虚警概率（统计级门限反解输入）。 */
  float min_snr_db{-10.0f}; /**< SNR 硬截断下限，低于该值直接判未探测。 */
};

/** @brief 检测器工程配置（副本：AR DetectionConfig 的识别消费子集）。 */
struct RirDetectorConfig {
  config::hardware::RirTransmitterConfig transmitter{};
  config::hardware::RirAntennaConfig antenna{};
  config::hardware::RirReceiverConfig receiver{};
  RirDetectionPolicyConfig detection_policy{};
  float min_detection_margin_db{-2.0f}; /**< 可靠性裕量门（低于该 SNR 的检测不可靠）。 */
  int pulse_count{10};                   /**< 默认检测脉冲数 N。 */
};

/** @brief 单目标探测结果。 */
struct RirDetectionResult {
  float echo_power_dbw{-300.0f}; /**< 接收回波功率 (dBW) */
  float snr_db{-100.0f};         /**< 信噪比 (dB) */
  float detection_prob{0.0f};    /**< 检测概率 Pd */
  bool detected{false};          /**< 是否达到门限 */
};

/** @brief 目标回波特征上下文。 */
struct RirTargetReturn {
  float rcs_m2{0.0f};  /**< 目标 RCS (m²) */
  float range_m{0.0f}; /**< 目标到雷达斜距 (m) */
  internal::RirSwerlingModel swerling_type{
      internal::RirSwerlingModel::kSwerling0}; /**< 目标的 Swerling 起伏模型 */
};

/** @brief 环境噪声上下文（v1 效能级路径；RF v2 分解走 DetectResolvedCell）。 */
struct RirEnvironmentNoise {
  float propagation_loss_db{0.0f}; /**< 大气传播往返损耗 (dB) */
  float clutter_noise_w{0.0f};     /**< 杂波噪声功率 (W) */
  float jam_noise_w{0.0f};         /**< 干扰噪声功率 (W) */
};

/**
 * @brief RirSignalDetector 封装物理化的回波评估与统计级 CFAR 探测判决。
 *
 * 组合 `RirRadarEquations` 纯函数完成一条完整的物理检测链路：
 *   回波功率预算 → SNR 计算 → 检测概率 → 蒙特卡洛判决
 * 热噪声功率底在构造时一次性预计算；随机种子外部注入保证确定性
 * （replay 语义：同种子同输入同判决）。
 */
class RirSignalDetector {
 public:
  /** @brief 使用检测配置构造检测器。 */
  explicit RirSignalDetector(const RirDetectorConfig& config);

  /** @brief 更新检测配置并重算热噪声底。 */
  void UpdateConfig(const RirDetectorConfig& config);

  /**
   * @brief 对单个目标执行完整检测链（v1 效能级路径）。
   * @param target 目标回波特征上下文
   * @param env 环境噪声上下文
   * @param one_way_antenna_gain_db 单程天线增益；NaN 时回退配置主瓣峰值增益
   * @param pulse_count 检测脉冲数 N
   * @return 探测结果
   */
  RirDetectionResult Detect(const RirTargetReturn& target, const RirEnvironmentNoise& env,
                            float one_way_antenna_gain_db = std::numeric_limits<float>::quiet_NaN(),
                            int pulse_count = 1);

  /**
   * @brief 对已完成物理 detection-cell 求解的结果执行 Pd 与随机门限判决。
   * @note 本入口不重复计算回波、噪声、压缩或脉冲积累增益。
   */
  RirDetectionResult DetectResolvedCell(const RirTargetReturn& target,
                                        const RirDetectionCellResult& cell);

  /** @brief 设置随机种子（用于确定性回归测试与 replay）。 */
  void SetRandomSeed(unsigned int seed);

 private:
  RirDetectorConfig config_; /**< 检测配置 */
  float thermal_noise_w_;    /**< 预计算的接收机热噪声底 (W) */
  std::mt19937 rng_;         /**< 确定性随机数引擎 */
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_SIGNAL_DETECTOR_H_
