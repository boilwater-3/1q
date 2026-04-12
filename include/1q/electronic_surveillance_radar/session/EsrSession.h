/**
 * @file EsrSession.h
 * @brief 定义面向外部调用方的电子侦察会话门面。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/common/scan_schedule_types.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/output/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"

namespace electronic_surveillance_radar {
namespace extension {
class EsrController;
}

namespace extension {
class IInterceptPipeline;
}
namespace environment {
class IEsrEnvironmentService;
}
}  // namespace electronic_surveillance_radar

namespace electronic_surveillance_radar {
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
  bool enable_layered_config{false};                    /**< 是否启用分层参数覆盖 */
  EsrLayeredConfig layered_config{};                    /**< 分层参数入口 */
  extension::InterceptPipelineConfig pipeline_config{}; /**< 流水线配置 */
  environment::EsrEnvironmentDefaultConfig environment_default_config{}; /**< 默认环境配置 */
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

  bool has_environment_runtime_config{false};
  environment::EsrEnvironmentRuntimeConfigPatch environment_runtime_config{};

  bool has_observation_jam_mark_threshold_w{false};
  float observation_jam_mark_threshold_w{0.0f};
};

/**
 * @brief EsrSession 提供单周期外部接入门面。
 * @note 线程模型：会话对象持有可变状态，默认非线程安全；并发访问需外部同步。
 */
class ONEQ_API EsrSession {
 public:
  /**
   * @brief 使用默认装配配置构造会话。
   * @param[in] config 会话配置。
   */
  explicit EsrSession(EsrSessionConfig config = {});
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, extension::IInterceptPipeline& pipeline);
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, environment::IEsrEnvironmentService& environment_service);
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, extension::EsrController& controller);
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, extension::IInterceptPipeline& pipeline,
             environment::IEsrEnvironmentService& environment_service,
             extension::EsrController& controller);
  ~EsrSession();

  EsrSession(const EsrSession&) = delete;
  EsrSession& operator=(const EsrSession&) = delete;

  /**
   * @brief 移动构造会话。
   */
  EsrSession(EsrSession&&) noexcept;
  /**
   * @brief 移动赋值会话。
   */
  EsrSession& operator=(EsrSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  output::EsrOutputFrame Step(const session::EsrCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult StepWithResult(const session::EsrCycleInput& input);

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

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_
