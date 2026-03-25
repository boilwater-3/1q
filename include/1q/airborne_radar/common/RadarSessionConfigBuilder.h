/**
 * @file RadarSessionConfigBuilder.h
 * @brief 提供链式构造 RadarSessionConfig 的 Builder，简化深层嵌套配置的设置。
 */

#ifndef AIRBORNE_RADAR_COMMON_RADAR_SESSION_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_COMMON_RADAR_SESSION_CONFIG_BUILDER_H_

#include <cstdint>

#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace common {

/**
 * @brief RadarSession 配置链式构造器。
 *
 * 将 `RadarSessionConfig` 内部五层嵌套的硬件与任务参数展平为单层方法调用，
 * 适合外部项目在已有预设基础上按平台特性微调配置。
 *
 * 典型用法：
 * @code
 * // 以探测任务预设为基础，叠加平台雷达硬件参数
 * auto config =
 *     airborne_radar::common::RadarSessionConfigBuilder(
 *         airborne_radar::common::MakeDetectionMissionRadarSessionConfig())
 *         .EnablePhysicsDetection()
 *         .WithTransmitterPeakPowerW(5e6f)
 *         .WithTransmitterFrequencyHz(9.3e9f)
 *         .WithAntennaMainBeamGainDb(38.0f)
 *         .WithReceiverNoiseFigureDb(3.5f)
 *         .WithJammingDetectionThresholdDb(4.5f)
 *         .Build();
 * airborne_radar::core::session::RadarSession session(config);
 * @endcode
 *
 * @note
 * - 构造函数接受任意 `RadarSessionConfig`（包括预设函数返回值或默认构造），
 *   未调用的 setter 保留基础配置的原始值。
 * - Builder 不验证参数合法性，物理量取值合理性由调用方负责。
 * - 未暴露于 Builder 的高级选项（天线方向图、IMM 参数、对象池大小等）
 *   仍可通过直接访问 `RadarSessionConfig` 内部嵌套字段配置。
 */
class ONEQ_API RadarSessionConfigBuilder {
 public:
  /**
   * @brief 以给定配置为基础构造 Builder。
   * @param config 基础配置，默认为零值构造的 RadarSessionConfig。
   *        通常传入 `MakeDetectionMissionRadarSessionConfig()` 等预设函数返回值。
   */
  explicit RadarSessionConfigBuilder(const core::session::RadarSessionConfig& config = {})
      : config_(config) {}

  // -------------------------------------------------------------------------
  // 发射机参数
  // -------------------------------------------------------------------------

  /** @brief 设置峰值发射功率（单位：W）。 */
  RadarSessionConfigBuilder& WithTransmitterPeakPowerW(float w) {
    config_.signal_pipeline_config.detection.radar_system.transmitter.peak_power_w = w;
    return *this;
  }

  /** @brief 设置工作载频（单位：Hz）。 */
  RadarSessionConfigBuilder& WithTransmitterFrequencyHz(float hz) {
    config_.signal_pipeline_config.detection.radar_system.transmitter.frequency_hz = hz;
    return *this;
  }

  /** @brief 设置信号带宽（单位：Hz）。 */
  RadarSessionConfigBuilder& WithTransmitterBandwidthHz(float hz) {
    config_.signal_pipeline_config.detection.radar_system.transmitter.bandwidth_hz = hz;
    return *this;
  }

  /** @brief 设置脉冲宽度（单位：s）。 */
  RadarSessionConfigBuilder& WithTransmitterPulseWidthS(float s) {
    config_.signal_pipeline_config.detection.radar_system.transmitter.pulse_width_s = s;
    return *this;
  }

  /** @brief 设置脉冲重复频率（单位：Hz）。 */
  RadarSessionConfigBuilder& WithTransmitterPrfHz(float hz) {
    config_.signal_pipeline_config.detection.radar_system.transmitter.prf_hz = hz;
    return *this;
  }

  /** @brief 设置馈线/发射系统损耗（单位：dB）。 */
  RadarSessionConfigBuilder& WithTransmitterLossDb(float db) {
    config_.signal_pipeline_config.detection.radar_system.transmitter.transmit_loss_db = db;
    return *this;
  }

  // -------------------------------------------------------------------------
  // 天线参数
  // -------------------------------------------------------------------------

  /** @brief 设置波束中心名义峰值增益（单位：dB）。 */
  RadarSessionConfigBuilder& WithAntennaMainBeamGainDb(float db) {
    config_.signal_pipeline_config.detection.radar_system.antenna.main_beam_gain_db = db;
    return *this;
  }

  /**
   * @brief 设置名义波束宽度（单位：°）。
   * @param az_deg 方位波束宽度。
   * @param el_deg 俯仰波束宽度。
   */
  RadarSessionConfigBuilder& WithAntennaNominalBeamwidthDeg(float az_deg, float el_deg) {
    auto& antenna = config_.signal_pipeline_config.detection.radar_system.antenna;
    antenna.nominal_az_beamwidth_deg = az_deg;
    antenna.nominal_el_beamwidth_deg = el_deg;
    return *this;
  }

  // -------------------------------------------------------------------------
  // 接收机参数
  // -------------------------------------------------------------------------

  /** @brief 设置接收机噪声系数（单位：dB）。 */
  RadarSessionConfigBuilder& WithReceiverNoiseFigureDb(float db) {
    config_.signal_pipeline_config.detection.radar_system.receiver.noise_figure_db = db;
    return *this;
  }

  /** @brief 设置接收系统损耗（单位：dB）。 */
  RadarSessionConfigBuilder& WithReceiverLossDb(float db) {
    config_.signal_pipeline_config.detection.radar_system.receiver.receive_loss_db = db;
    return *this;
  }

  // -------------------------------------------------------------------------
  // 物理探测参数
  // -------------------------------------------------------------------------

  /**
   * @brief 启用或禁用物理层雷达方程检测。
   * @param enable 默认 `true`（启用）。启用后 `radar_system` 参数生效。
   */
  RadarSessionConfigBuilder& EnablePhysicsDetection(bool enable = true) {
    config_.signal_pipeline_config.detection.enable_physics_detection = enable;
    return *this;
  }

  /**
   * @brief 设置最小检测裕量（单位：dB）。
   * @note 此值越小，越容易通过检测门限；负值表示允许在预期 SNR 不足时仍报告检测结果。
   */
  RadarSessionConfigBuilder& WithMinDetectionMarginDb(float db) {
    config_.signal_pipeline_config.detection.min_detection_margin_db = db;
    return *this;
  }

  /** @brief 设置相干积累脉冲数。 */
  RadarSessionConfigBuilder& WithPulseCount(int count) {
    config_.signal_pipeline_config.detection.pulse_count = count;
    return *this;
  }

  /** @brief 设置是否启用相干积累。 */
  RadarSessionConfigBuilder& WithCoherentIntegration(bool enable) {
    config_.signal_pipeline_config.detection.coherent_integration = enable;
    return *this;
  }

  // -------------------------------------------------------------------------
  // 跟踪参数
  // -------------------------------------------------------------------------

  /** @brief 设置卡尔曼测量噪声标准差。值越大，滤波器对新量测的信任度越低。 */
  RadarSessionConfigBuilder& WithKalmanMeasurementNoiseStd(float std_dev) {
    config_.signal_pipeline_config.tracking.kalman_measurement_noise_std = std_dev;
    return *this;
  }

  // -------------------------------------------------------------------------
  // 生命周期参数
  // -------------------------------------------------------------------------

  /**
   * @brief 启用或禁用自动生命周期管理（Kalman 跟踪路径）。
   * @param enable 默认 `true`（启用）。生产配置通常应设为 `true`。
   */
  RadarSessionConfigBuilder& EnableAutoLifecycleManager(bool enable = true) {
    config_.signal_pipeline_config.lifecycle.enable_auto_lifecycle_manager = enable;
    return *this;
  }

  /** @brief 设置候选轨迹转已确认所需最小命中次数。 */
  RadarSessionConfigBuilder& WithConfirmHits(std::uint32_t hits) {
    config_.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits = hits;
    return *this;
  }

  /** @brief 设置已确认轨迹转丢失前允许的最大连续失配次数。 */
  RadarSessionConfigBuilder& WithMaxMissBeforeLost(std::uint32_t misses) {
    config_.signal_pipeline_config.lifecycle.lifecycle_config.max_miss_before_lost = misses;
    return *this;
  }

  /** @brief 设置丢失轨迹可保留的最大周期数，超出则回收。 */
  RadarSessionConfigBuilder& WithMaxLostCycles(std::uint32_t cycles) {
    config_.signal_pipeline_config.lifecycle.lifecycle_config.max_lost_cycles = cycles;
    return *this;
  }

  // -------------------------------------------------------------------------
  // 会话级参数
  // -------------------------------------------------------------------------

  /**
   * @brief 设置干扰判定阈值（单位：dB）。
   * @note 默认 6.0 dB；调低意味着更容易触发干扰标记。
   */
  RadarSessionConfigBuilder& WithJammingDetectionThresholdDb(float db) {
    config_.jamming_detection_threshold_db = db;
    return *this;
  }

  // -------------------------------------------------------------------------

  /** @brief 生成配置对象。 */
  core::session::RadarSessionConfig Build() const { return config_; }

 private:
  core::session::RadarSessionConfig config_;
};

}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_RADAR_SESSION_CONFIG_BUILDER_H_
