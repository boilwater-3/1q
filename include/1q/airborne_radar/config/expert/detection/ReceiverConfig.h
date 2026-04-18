/**
 * @file ReceiverConfig.h
 * @brief 定义 expert 接收机配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_RECEIVER_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_RECEIVER_CONFIG_H_

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert 接收机工程参数。
 */
struct ReceiverConfig {
  float noise_figure_db{4.0f}; /**< 接收机噪声系数。 */
  float receive_loss_db{2.0f}; /**< 接收链路损耗。 */
};

}  // namespace detection
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_RECEIVER_CONFIG_H_
