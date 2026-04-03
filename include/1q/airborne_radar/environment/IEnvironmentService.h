/**
 * @file IEnvironmentService.h
 * @brief 定义环境建模层对外暴露的只读服务接口。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace environment {

/**
 * @brief IEnvironmentService 为信号处理与决策层提供只读环境查询接口。
 */
class ONEQ_API IEnvironmentService {
 public:
  virtual ~IEnvironmentService() = default;

  /**
   * @brief 冻结当前周期环境事实，供后续只读采样复用。
   * @param[in] cycle_context 当前周期的冻结上下文，包含周期号与步长。
   */
  virtual void BeginCycle(const EnvironmentCycleContext& cycle_context) = 0;

  /**
   * @brief 采样并返回当前处理周期的环境条件。
   * @return 当前周期的环境快照，包含传播损耗、杂波/干扰功率等信息。
   */
  virtual EnvironmentSnapshot SampleEnvironment() const = 0;

  /**
   * @brief 更新待生效场景状态。
   * @param[in] scene_state 新场景状态。
   */
  virtual void UpdateSceneState(const EnvironmentSceneState& scene_state) = 0;

  /**
   * @brief 更新环境模型配置。
   * @param[in] config 新环境模型配置。
   */
  virtual void UpdateModelConfig(const EnvironmentModelConfig& config) = 0;

  /**
   * @brief 设置干扰判定阈值。
   * @param[in] threshold_db 阈值（单位：dB）。
   */
  virtual void SetJammingDetectionThresholdDb(float threshold_db) = 0;
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
