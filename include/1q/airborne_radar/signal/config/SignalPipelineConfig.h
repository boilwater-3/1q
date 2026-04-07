/**
 * @file SignalPipelineConfig.h
 * @brief 定义面向外部调用方的信号流水线聚合配置壳。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_PIPELINE_CONFIG_H_

#include "1q/airborne_radar/signal/config/SignalBeamControlConfig.h"
#include "1q/airborne_radar/signal/config/SignalDetectionConfig.h"
#include "1q/airborne_radar/signal/config/SignalLifecycleConfig.h"
#include "1q/airborne_radar/signal/config/SignalTrackingConfig.h"

namespace airborne_radar {
namespace signal {
namespace config {

/**
 * @brief SignalPipelineConfig 描述信号处理流水线对外聚合配置。
 */
struct SignalPipelineConfig {
  SignalDetectionConfig detection{};      /**< 探测配置 */
  SignalBeamControlConfig beam_control{}; /**< 波束控制配置 */
  SignalTrackingConfig tracking{};        /**< 跟踪配置 */
  SignalLifecycleConfig lifecycle{};      /**< 生命周期配置 */
};

}  // namespace config
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_PIPELINE_CONFIG_H_
