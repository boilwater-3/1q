/**
 * @file SignalDetector.h
 * @brief 定义封装物理化回波评估与探测判决的信号检测器。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_

#include <limits>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"
#include "airborne_radar/signal/detection/RadarEquations.h"
#include "common/radar/StatisticalCfarDetector.h"

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
 * 回波功率由模块 RadarEquations 适配计算；Pd/蒙特卡洛/裕量门转调
 * `oneq::common::radar::StatisticalCfarDetector`。
 */
class SignalDetector {
 public:
  explicit SignalDetector(config::engineering::DetectionConfig config);
  void UpdateConfig(config::engineering::DetectionConfig config);
  DetectionResult Detect(const TargetReturn& target, const EnvironmentState& env,
                         float one_way_antenna_gain_db = std::numeric_limits<float>::quiet_NaN(),
                         int pulse_count = 1);
  DetectionResult DetectResolvedCell(const TargetReturn& target,
                                     const ArDetectionCellResult& cell);
  void SetRandomSeed(unsigned int seed);

 private:
  config::engineering::DetectionConfig config_;
  oneq::common::radar::StatisticalCfarDetector cfar_;
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_
