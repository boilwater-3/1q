/**
 * @file EosSession.h
 * @brief 定义光学传感器对外会话门面。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/core/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace core {
namespace session {

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
  float visible_reference_irradiance_w_m2{800.0f}; /**< 可见光辐照度归一化参考值（单位：W/m^2） */
};

/**
 * @brief EosSession 提供单周期步进执行入口。
 */
class ONEQ_API EosSession {
 public:
  /**
   * @brief 构造光学传感器会话。
   * @param[in] config 会话初始化配置。
   */
  explicit EosSession(EosSessionConfig config = {});
  ~EosSession();

  EosSession(const EosSession&) = delete;
  EosSession& operator=(const EosSession&) = delete;

  /**
   * @brief 执行单周期并返回输出帧。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  common::EosOutputFrame Step(const context::EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EosCycleResult StepWithResult(const context::EosCycleInput& input);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_H_
