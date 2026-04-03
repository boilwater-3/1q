/**
 * @file EsrSession.h
 * @brief 定义面向外部调用方的电子侦察会话门面。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_SESSION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/common/scan_schedule_types.h"
#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {

/**
 * @brief EsrWorkMode 描述 ESR 工作模式。
 */
enum class ONEQ_API EsrWorkMode {
  kEsm = 0, /**< 常规电子支援侦察模式 */
  kHgesm,   /**< 高增益电子支援侦察模式 */
  kRwr      /**< 告警接收机模式 */
};

/** @brief ESR 兼容别名：扫描起始象限。 */
using EsrScanStartPosition = oneq::common::ScanStartPosition;

/** @brief ESR 兼容别名：二维扫描推进顺序。 */
using EsrScanSequence = oneq::common::ScanSequence;

/**
 * @brief EsrHardwareConfig 描述 ESR 装备固有参数。
 */
struct ONEQ_API EsrHardwareConfig {
  double receiver_band_lower_hz{0.23e9};  /**< 接收频段下限（单位：Hz） */
  double receiver_band_upper_hz{100.0e9}; /**< 接收频段上限（单位：Hz） */
  float receiver_sensitivity_w{1.0e-12f}; /**< 接收机灵敏度（单位：W） */
  float integrated_receive_loss_db{0.0f}; /**< 系统综合接收损耗（单位：dB） */
  float beam_az_width_deg{5.0f};          /**< 方位波束宽度（单位：deg） */
  float beam_el_width_deg{5.0f};          /**< 俯仰波束宽度（单位：deg） */
  float az_scan_range_deg{120.0f};        /**< 方位扫描范围（单位：deg） */
  float el_scan_range_deg{20.0f};         /**< 俯仰扫描范围（单位：deg） */
  float antenna_mount_az_deg{0.0f};       /**< 天线中心方位相对角（单位：deg） */
  float antenna_mount_el_deg{0.0f};       /**< 天线中心俯仰相对角（单位：deg） */
};

/**
 * @brief EsrMissionControlConfig 描述 ESR 任务运行态控制参数。
 */
struct ONEQ_API EsrMissionControlConfig {
  bool power_on{true};                      /**< 设备开关机状态 */
  EsrWorkMode work_mode{EsrWorkMode::kEsm}; /**< 当前工作模式 */
  float scan_center_az_deg{0.0f};           /**< 扫描中心方位（单位：deg） */
  float scan_center_el_deg{0.0f};           /**< 扫描中心俯仰（单位：deg） */
  float scan_rate_hz{1.0f};                 /**< 扫描数据率（单位：Hz） */
  EsrScanStartPosition scan_start_position{EsrScanStartPosition::kLeftTop}; /**< 扫描起始位置 */
  EsrScanSequence scan_sequence{EsrScanSequence::kAzimuthFirst};            /**< 扫描顺序 */
  bool use_explicit_scan_bounds{false}; /**< 是否使用显式扫描起止角 */
  float scan_start_az_deg{-60.0f};      /**< 扫描起始方位（单位：deg） */
  float scan_end_az_deg{60.0f};         /**< 扫描结束方位（单位：deg） */
  float scan_start_el_deg{-10.0f};      /**< 扫描起始俯仰（单位：deg） */
  float scan_end_el_deg{10.0f};         /**< 扫描结束俯仰（单位：deg） */
};

/**
 * @brief EsrLayeredConfig 描述 ESR 分层参数入口。
 */
struct ONEQ_API EsrLayeredConfig {
  EsrHardwareConfig hardware{};      /**< 装备固有参数 */
  EsrMissionControlConfig mission{}; /**< 任务控制参数 */
};

/**
 * @brief EsrSessionConfig 描述电子侦察会话默认装配配置。
 */
struct ONEQ_API EsrSessionConfig {
  bool enable_layered_config{false};                           /**< 是否启用分层参数覆盖 */
  EsrLayeredConfig layered_config{};                           /**< 分层参数入口 */
  pipeline::InterceptPipelineConfig pipeline_config{};         /**< 流水线配置 */
  environment::EsrEnvironmentModelConfig environment_config{}; /**< 环境模型配置 */
};

/**
 * @brief EsrRuntimeConfigPatch 描述运行期可变参数补丁。
 */
struct ONEQ_API EsrRuntimeConfigPatch {
  bool has_sensor_enabled{false};
  bool sensor_enabled{true};

  bool has_scan_rate_hz{false};
  float scan_rate_hz{1.0f};

  bool has_integrated_receive_loss_db{false};
  float integrated_receive_loss_db{0.0f};

  bool has_fixed_receiver_window_hz{false};
  double receiver_lower_hz{0.0};
  double receiver_upper_hz{0.0};

  bool has_use_fixed_receiver_window{false};
  bool use_fixed_receiver_window{true};

  bool has_enable_statistical_detection{false};
  bool enable_statistical_detection{true};

  bool has_enable_spectral_analysis{false};
  bool enable_spectral_analysis{true};

  bool has_detection_min_snr_db{false};
  float detection_min_snr_db{6.0f};

  bool has_jamming_detection_threshold_w{false};
  float jamming_detection_threshold_w{0.0f};
};

/**
 * @brief EsrSessionConfigBuilder 提供初始化配置链式构造。
 */
class ONEQ_API EsrSessionConfigBuilder {
 public:
  explicit EsrSessionConfigBuilder(const EsrSessionConfig& config = {}) : config_(config) {}

  EsrSessionConfigBuilder& EnableLayeredConfig(bool enable = true) {
    config_.enable_layered_config = enable;
    return *this;
  }
  EsrSessionConfigBuilder& WithSessionConfig(const EsrSessionConfig& config) {
    config_ = config;
    return *this;
  }
  EsrSessionConfigBuilder& WithWorkMode(EsrWorkMode mode) {
    config_.layered_config.mission.work_mode = mode;
    return *this;
  }
  EsrSessionConfigBuilder& WithScanRateHz(float value) {
    config_.layered_config.mission.scan_rate_hz = value;
    return *this;
  }
  EsrSessionConfigBuilder& EnableSpectralAnalysis(bool enable = true) {
    config_.pipeline_config.spectral_analysis.enable = enable;
    return *this;
  }
  EsrSessionConfigBuilder& WithDetectionMinSnrDb(float value) {
    config_.pipeline_config.detection.min_detect_snr_db = value;
    return *this;
  }
  EsrSessionConfigBuilder& WithJammingDetectionThresholdW(float value) {
    config_.environment_config.jamming_detection_threshold_w = value;
    return *this;
  }
  EsrSessionConfig Build() const { return config_; }

 private:
  EsrSessionConfig config_{};
};

/**
 * @brief EsrRuntimeConfigBuilder 提供运行期补丁链式构造。
 */
class ONEQ_API EsrRuntimeConfigBuilder {
 public:
  explicit EsrRuntimeConfigBuilder(const EsrRuntimeConfigPatch& patch = {}) : patch_(patch) {}

  EsrRuntimeConfigBuilder& WithRuntimeConfigPatch(const EsrRuntimeConfigPatch& patch) {
    patch_ = patch;
    return *this;
  }

  EsrRuntimeConfigBuilder& WithSensorEnabled(bool enable) {
    patch_.has_sensor_enabled = true;
    patch_.sensor_enabled = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithScanRateHz(float value) {
    patch_.has_scan_rate_hz = true;
    patch_.scan_rate_hz = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithIntegratedReceiveLossDb(float value) {
    patch_.has_integrated_receive_loss_db = true;
    patch_.integrated_receive_loss_db = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithFixedReceiverWindowHz(double lower_hz, double upper_hz) {
    patch_.has_fixed_receiver_window_hz = true;
    patch_.has_use_fixed_receiver_window = true;
    patch_.use_fixed_receiver_window = true;
    patch_.receiver_lower_hz = lower_hz;
    patch_.receiver_upper_hz = upper_hz;
    return *this;
  }
  EsrRuntimeConfigBuilder& SetFixedReceiverWindowEnabled(bool enable) {
    patch_.has_use_fixed_receiver_window = true;
    patch_.use_fixed_receiver_window = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& DisableFixedReceiverWindow(bool disable = true) {
    patch_.has_use_fixed_receiver_window = true;
    patch_.use_fixed_receiver_window = !disable;
    return *this;
  }
  EsrRuntimeConfigBuilder& EnableStatisticalDetection(bool enable = true) {
    patch_.has_enable_statistical_detection = true;
    patch_.enable_statistical_detection = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& EnableSpectralAnalysis(bool enable = true) {
    patch_.has_enable_spectral_analysis = true;
    patch_.enable_spectral_analysis = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithDetectionMinSnrDb(float value) {
    patch_.has_detection_min_snr_db = true;
    patch_.detection_min_snr_db = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithJammingDetectionThresholdW(float value) {
    patch_.has_jamming_detection_threshold_w = true;
    patch_.jamming_detection_threshold_w = value;
    return *this;
  }
  EsrRuntimeConfigPatch Build() const { return patch_; }

 private:
  EsrRuntimeConfigPatch patch_{};
};

/**
 * @brief EsrSession 提供单周期外部接入门面。
 */
class ONEQ_API EsrSession {
 public:
  /**
   * @brief 使用默认装配配置构造会话。
   * @param[in] config 会话配置。
   */
  explicit EsrSession(EsrSessionConfig config = {});
  ~EsrSession();

  EsrSession(const EsrSession&) = delete;
  EsrSession& operator=(const EsrSession&) = delete;

  /**
   * @brief 执行单周期并返回输出帧。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  common::EsrOutputFrame Step(const context::EsrCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult StepWithResult(const context::EsrCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期补丁。
   */
  void ApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_SESSION_H_
