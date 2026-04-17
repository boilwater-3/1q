/**
 * @file SignalTrackingConfig.h
 * @brief 定义跟踪域关键配置（config 主入口）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_TRACKING_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_TRACKING_CONFIG_H_

namespace airborne_radar {
namespace config {

/**
 * @brief TrackingPolicyProfile 表示对外跟踪策略档位。
 */
enum class TrackingPolicyProfile {
  kBalanced = 0,         /**< 平衡策略 */
  kFastAssociation = 1,  /**< 快速关联策略 */
  kRobustAntiJamming = 2 /**< 抗干扰稳健策略 */
};

/**
 * @brief SignalTrackingConfig 描述对外语义化跟踪输入。
 */
struct SignalTrackingConfig {
  bool enable_tracking_filter{true}; /**< 是否启用跟踪滤波链路 */
  TrackingPolicyProfile policy_profile{
      TrackingPolicyProfile::kBalanced}; /**< 跟踪策略档位 */
};

namespace engineering {

enum class KalmanUpdateBackend {
  kStandardKfJoseph = 0,
  kUdKf,
  kSrif
};

struct TrackingConfig {
  bool enable_kalman_filter{true};
  float kalman_measurement_noise_std{10.0f};
  KalmanUpdateBackend kalman_update_backend{
      KalmanUpdateBackend::kStandardKfJoseph};
};

}  // namespace engineering

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_TRACKING_CONFIG_H_
