/**
 * @file StatisticalCfarDetector.h
 * @brief 统计级 CFAR 判决编排（Pfa/Swerling/种子）；不含仿真脚手架门控。
 */

#ifndef COMMON_RADAR_STATISTICAL_CFAR_DETECTOR_H_
#define COMMON_RADAR_STATISTICAL_CFAR_DETECTOR_H_

#include <random>

#include "common/radar/RadarEquations.h"

namespace oneq {
namespace common {
namespace radar {

/** @brief 统计级 CFAR 策略标量。 */
struct StatisticalCfarPolicy {
  float cfar_pfa{1.0e-6f};
  float min_snr_db{-10.0f};
  float min_detection_margin_db{-2.0f};
};

/** @brief 单目标探测判决结果。 */
struct StatisticalCfarResult {
  float echo_power_dbw{-300.0f};
  float snr_db{-100.0f};
  float detection_prob{0.0f};
  bool detected{false};
};

/**
 * @brief 统计级 CFAR 编排器：SNR → Pd → 蒙特卡洛 → 裕量门。
 *
 * 热噪声底由调用方预计算注入；回波功率可由调用方用 RadarEquations 算出后传入
 * DetectFromEchoBudget，或直接用 DetectResolvedCell 消费 detection-cell 账本。
 */
class StatisticalCfarDetector {
 public:
  StatisticalCfarDetector(float thermal_noise_w, StatisticalCfarPolicy policy);

  void Update(float thermal_noise_w, StatisticalCfarPolicy policy);
  void SetRandomSeed(unsigned int seed);

  /**
   * @brief 效能级路径：已含接收损耗的回波 dBW + 杂波/干扰瓦 → 判决。
   * @param echo_power_dbw_after_receive_loss 扣除接收机系统损耗后的回波功率 (dBW)
   */
  StatisticalCfarResult DetectFromEchoBudget(float echo_power_dbw_after_receive_loss,
                                             float clutter_noise_w, float jam_noise_w,
                                             SwerlingModel swerling, int pulse_count);

  /**
   * @brief detection-cell 路径：不重复计算回波/噪声/脉压。
   */
  StatisticalCfarResult DetectResolvedCell(float echo_power_w, float processed_single_pulse_sinr_db,
                                           SwerlingModel swerling, int pulse_count);

 private:
  float thermal_noise_w_{0.0f};
  StatisticalCfarPolicy policy_{};
  std::mt19937 rng_;
};

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_STATISTICAL_CFAR_DETECTOR_H_
