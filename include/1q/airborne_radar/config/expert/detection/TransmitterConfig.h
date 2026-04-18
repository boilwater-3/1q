/**
 * @file TransmitterConfig.h
 * @brief 定义 expert 发射机配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_TRANSMITTER_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_TRANSMITTER_CONFIG_H_

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert 发射机工程参数。
 */
struct TransmitterConfig {
  float peak_power_w{1e6f}; /**< 峰值发射功率。 */
  float frequency_hz{3e9f}; /**< 工作频率。 */
  float bandwidth_hz{4.5e6f}; /**< 发射带宽。 */
  float pulse_width_s{13e-6f}; /**< 脉宽。 */
  float prf_hz{300.0f}; /**< 脉冲重复频率。 */
  float transmit_loss_db{3.5f}; /**< 发射链路损耗。 */
};

}  // namespace detection
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_TRANSMITTER_CONFIG_H_
