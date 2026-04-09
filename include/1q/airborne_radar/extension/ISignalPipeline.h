/**
 * @file ISignalPipeline.h
 * @brief 定义信号处理层的流水线抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_

#include <memory>

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace extension {
class IEnvironmentService;

struct SignalPipelineRuntimeState {
  std::shared_ptr<void> opaque{}; /**< 由具体 pipeline 实现解释的运行态快照 */
};

/**
 * @brief ISignalPipeline 定义单周期内的探测与跟踪处理流程。
 */
class ONEQ_API ISignalPipeline {
 public:
  virtual ~ISignalPipeline() = default;

  /**
   * @brief 执行一次信号处理循环。
   * @param[in] input_state 当前周期目标特征列表。
   * @param[in] environment 环境服务只读接口。调用前必须已对该环境服务执行有效的
   *                        `BeginCycle(...)`，并确保其冻结快照携带正的 `cycle_dt_sec`。
   * @return 当前周期信号流水线输出结果。
   */
  virtual SignalCycleResult RunCycle(const model::TargetFeatureList& input_state,
                                     const extension::IEnvironmentService& environment) = 0;

  /**
   * @brief 更新当前搭载平台姿态。
   * @param[in] platform_attitude_deg 平台姿态角（单位：度）。
   */
  virtual void UpdatePlatformAttitude(
      const model::PlatformAttitudeDeg& platform_attitude_deg) = 0;

  /**
   * @brief 获取当前搭载平台姿态。
   * @return 平台姿态角（单位：度）。
   */
  virtual model::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

  /**
   * @brief 设置下一周期生效的控制真值。
   * @param[in] control_profile 控制真值。
   */
  virtual void SetControlProfile(const extension::control::RadarControlProfile& control_profile) = 0;

  /**
   * @brief 获取当前缓存的控制真值。
   * @return 当前缓存的控制真值。
   */
  virtual extension::control::RadarControlProfile GetControlProfile() const = 0;

  /**
   * @brief 更新流水线运行配置。
   * @param[in] config 新配置。
   */
  virtual void UpdateConfig(config::SignalPipelineConfig config) = 0;

  /**
   * @brief 获取上一周期关联质量指标。
   * @return 上一周期缓存的关联质量指标。
   */
  virtual AssociationQualityMetrics GetLastAssociationQualityMetrics() const = 0;

  /**
   * @brief 捕获当前 pipeline 运行态快照。
   * @return 可用于失败回滚的 pipeline 运行态快照。
   */
  virtual SignalPipelineRuntimeState CaptureRuntimeState() const = 0;

  /**
   * @brief 恢复此前捕获的 pipeline 运行态快照。
   * @param state 待恢复的 pipeline 运行态快照。
   */
  virtual void RestoreRuntimeState(const SignalPipelineRuntimeState& state) = 0;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
