/**
 * @file ISignalPipeline.h
 * @brief 定义信号处理层的流水线抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_

#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace environment {
class IEnvironmentService;
}  // namespace environment
namespace signal {
namespace pipeline {

/**
 * @brief ISignalPipeline 定义单周期内的探测与跟踪处理流程。
 */
class ONEQ_API ISignalPipeline {
 public:
  virtual ~ISignalPipeline() = default;

  /**
   * @brief 执行一次信号处理循环。
   * @param[in] input_state 当前周期目标特征列表。
   * @param[in] environment 环境服务只读接口。
   * @return 当前周期信号流水线输出结果。
   */
  virtual SignalCycleResult RunCycle(const common::TargetFeatureList& input_state,
                                     const environment::IEnvironmentService& environment) = 0;

  /**
   * @brief 更新当前搭载平台姿态。
   * @param[in] platform_attitude_deg 平台姿态角（单位：度）。
   */
  virtual void UpdatePlatformAttitude(const common::PlatformAttitudeDeg& platform_attitude_deg) = 0;

  /**
   * @brief 获取当前搭载平台姿态。
   * @return 平台姿态角（单位：度）。
   */
  virtual common::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

  /**
   * @brief 设置下一周期生效的控制真值。
   * @param[in] control_profile 控制真值。
   */
  virtual void SetControlProfile(const common::RadarControlProfile& control_profile) = 0;

  /**
   * @brief 获取当前缓存的控制真值。
   * @return 当前缓存的控制真值。
   */
  virtual common::RadarControlProfile GetControlProfile() const = 0;
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
