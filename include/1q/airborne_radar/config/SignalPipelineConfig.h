/**
 * @file SignalPipelineConfig.h
 * @brief 定义面向外部调用方的信号流水线聚合配置壳。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_PIPELINE_CONFIG_H_

#include <cstdint>

#include "1q/airborne_radar/config/SignalBeamControlConfig.h"
#include "1q/airborne_radar/config/SignalDetectionConfig.h"

namespace airborne_radar {
namespace common {
namespace config {

/**
 * @brief LifecycleConfig 定义轨迹状态机阈值配置。
 */
struct LifecycleConfig {
  std::uint32_t confirm_hits{3};         /**< 候选轨迹转已确认所需最小命中次数 */
  std::uint32_t max_miss_before_lost{2}; /**< 已确认轨迹转丢失前允许的最大连续失配次数 */
  std::uint32_t max_lost_cycles{5};      /**< 丢失轨迹可保留的最大周期数，超出则回收 */
};

/**
 * @brief SignalTrackingConfig 描述信号层对外可调的跟踪域配置。
 */
struct SignalTrackingConfig {
  bool enable_kalman_filter{true};           /**< 是否启用卡尔曼滤波器 */
  float kalman_measurement_noise_std{10.0f}; /**< 卡尔曼测量噪声标准差 */
};

/**
 * @brief SignalLifecycleConfig 描述生命周期域配置。
 */
struct SignalLifecycleConfig {
  bool enable_auto_lifecycle_manager{false}; /**< 是否启用自动生命周期管理 */
  LifecycleConfig lifecycle_config{};        /**< 生命周期配置 */
  bool enable_imm_lifecycle{false};          /**< 是否启用 IMM 生命周期管理 */
};

/**
 * @brief SignalPipelineConfig 描述信号处理流水线对外聚合配置。
 */
struct SignalPipelineConfig {
  SignalDetectionConfig detection{};       /**< 探测配置 */
  SignalBeamControlConfig beam_control{};  /**< 波束控制配置 */
  SignalTrackingConfig tracking{};         /**< 跟踪配置 */
  SignalLifecycleConfig lifecycle{};       /**< 生命周期配置 */
};

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_PIPELINE_CONFIG_H_
