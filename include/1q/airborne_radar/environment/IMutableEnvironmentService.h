/**
 * @file IMutableEnvironmentService.h
 * @brief 定义环境建模层可变状态写入接口。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_I_MUTABLE_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_I_MUTABLE_ENVIRONMENT_SERVICE_H_

#include "1q/airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief 在只读环境接口基础上补充写能力，供 Session 外部装配注入。
 */
class ONEQ_API IMutableEnvironmentService : public IEnvironmentService {
 public:
  ~IMutableEnvironmentService() override = default;

  /**
   * @brief 更新待生效场景状态。
   * @param scene_state 新场景状态。
   */
  virtual void UpdateSceneState(const EnvironmentSceneState& scene_state) = 0;

  /**
   * @brief 更新环境模型配置。
   * @param config 新环境模型配置。
   */
  virtual void UpdateModelConfig(const EnvironmentModelConfig& config) = 0;

  /**
   * @brief 设置干扰判定阈值。
   * @param threshold_db 阈值（单位：dB）。
   */
  virtual void SetJammingDetectionThresholdDb(float threshold_db) = 0;
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_I_MUTABLE_ENVIRONMENT_SERVICE_H_
