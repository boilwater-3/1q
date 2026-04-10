/**
 * @file EosSession.h
 * @brief 定义光学传感器对外会话门面。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"

namespace electro_optical_sensor {
namespace extension {
class IEosPipeline;
class IEosEnvironmentService;
class EosController;
}  // namespace extension
namespace session {

class EosSessionFactory;

/**
 * @brief EosWorkMode 表示传感器工作模式。
 */
enum class ONEQ_API EosWorkMode {
  kInfraredOnly = 0, /**< 红外探测 */
  kVisibleOnly,      /**< 可见光探测 */
  kFused             /**< 红外/可见光融合探测 */
};

/**
 * @brief EosSessionConfig 描述会话初始化参数。
 */
struct ONEQ_API EosSessionConfig {
  float wavelength_lower_um{3.0f};               /**< 工作波长下限（单位：um） */
  float wavelength_upper_um{5.0f};               /**< 工作波长上限（单位：um） */
  float optical_aperture_m{0.2f};                /**< 光学口径（单位：m） */
  float focal_length_m{0.8f};                    /**< 焦距（单位：m） */
  EosWorkMode work_mode{EosWorkMode::kFused};    /**< 工作模式 */
  float horizontal_fov_deg{6.0f};                /**< 水平视场角（单位：deg） */
  float vertical_fov_deg{4.0f};                  /**< 垂直视场角（单位：deg） */
  float scan_rate_deg_per_sec{20.0f};            /**< 扫描速率（单位：deg/s） */
  float frame_rate_hz{30.0f};                    /**< 帧频（单位：Hz） */
  float minimum_snr_db{6.0f};                    /**< 最低信噪比阈值（单位：dB） */
  float detection_sensitivity_w{1.0e-12f};       /**< 探测灵敏度（单位：W） */
  float scan_start_az_deg{-60.0f};               /**< 扫描起始方位角（单位：deg） */
  float scan_end_az_deg{60.0f};                  /**< 扫描结束方位角（单位：deg） */
  float scan_center_el_deg{0.0f};                /**< 扫描中心俯仰角（单位：deg） */
  float boresight_depression_deg{45.0f};         /**< 光轴下视角（单位：deg） */
  float min_detection_depression_deg{1.0f};      /**< 最小有效下视角（单位：deg） */
  float max_detection_depression_deg{89.0f};     /**< 最大有效下视角（单位：deg） */
  float visible_reference_irradiance_w_m2{800.0f}; /**< 可见光辐照度归一化参考值（单位：W/m^2） */
  bool enable_straylight_filter{false};          /**< 是否启用遮光罩杂散光抑制 */
  float hood_inner_half_angle_deg{12.0f};        /**< 遮光罩内半角（单位：deg） */
  float hood_outer_half_angle_deg{75.0f};        /**< 遮光罩外半角（单位：deg） */
  float hood_min_suppression_ratio{0.20f};       /**< 最低抑制比例，范围 [0, 1] */
  float hood_max_suppression_ratio{0.85f};       /**< 最高抑制比例，范围 [0, 1] */
  environment::EosEnvironmentDefaultConfig
      environment_default_config{}; /**< 默认环境配置 */
};

/**
 * @brief EosRuntimeConfigPatch 描述运行期可变参数补丁。
 */
struct ONEQ_API EosRuntimeConfigPatch {
  bool has_work_mode{false};
  EosWorkMode work_mode{EosWorkMode::kFused};

  bool has_scan_rate_deg_per_sec{false};
  float scan_rate_deg_per_sec{20.0f};

  bool has_frame_rate_hz{false};
  float frame_rate_hz{30.0f};

  bool has_minimum_snr_db{false};
  float minimum_snr_db{6.0f};

  bool has_enable_straylight_filter{false};
  bool enable_straylight_filter{false};

  bool has_visible_reference_irradiance_w_m2{false};
  float visible_reference_irradiance_w_m2{800.0f};

  bool has_environment_runtime_config{false};
  environment::EosEnvironmentRuntimeConfigPatch environment_runtime_config{};
};

/**
 * @brief EosSession 提供单周期步进执行入口。
 * @note 通过 `EosSessionFactory` 创建，避免外部直接拼装不一致依赖图。
 * @note 线程模型：会话内部维护可变运行态，非线程安全；并发调用需外部串行化或加锁。
 */
class ONEQ_API EosSession {
 public:
  ~EosSession() noexcept;

  EosSession(const EosSession&) = delete;
  EosSession& operator=(const EosSession&) = delete;
  EosSession(EosSession&&) noexcept;
  EosSession& operator=(EosSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧（输出便捷入口）。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   * @note 该接口仅返回输出帧，不携带 `executed_this_cycle` /
   *       `reused_previous_output` 等状态语义；若调用方需要区分
   *       "本周期实际执行" 与 "复用上一有效输出"，请使用 `StepWithResult()`。
   * @note 非线程安全：会读写会话内部状态；并发调用需外部同步。
   */
  common::EosOutputFrame Step(const EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note 结果中的 `executed_this_cycle` / `reused_previous_output`
   *       提供结构化周期状态语义。
   * @note 非线程安全：会读写会话内部状态；并发调用需外部同步。
   */
  EosCycleResult StepWithResult(const EosCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期补丁。
   * @note 非线程安全：会更新运行期配置并可能重置扫描相位；并发调用需外部同步。
   */
  void ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch);

 private:
  friend class EosSessionFactory;

  struct Impl;
  explicit EosSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

/**
 * @brief EosSessionFactory 负责 EOS 会话装配与创建。
 */
class ONEQ_API EosSessionFactory {
 public:
  static EosSession Create(const EosSessionConfig& config = {});

  static EosSession CreateWithPipeline(const EosSessionConfig& config,
                                       extension::IEosPipeline& pipeline);

  static EosSession CreateWithEnvironmentService(
      const EosSessionConfig& config,
      extension::IEosEnvironmentService& environment_service);

  static EosSession CreateWithController(const EosSessionConfig& config,
                                         extension::EosController& controller);
};

}  // namespace session

namespace core {
namespace session {
using ::electro_optical_sensor::session::EosWorkMode;
using ::electro_optical_sensor::session::EosSessionConfig;
using ::electro_optical_sensor::session::EosRuntimeConfigPatch;
using ::electro_optical_sensor::session::EosSession;
using ::electro_optical_sensor::session::EosSessionFactory;
}  // namespace session
}  // namespace core

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_H_
