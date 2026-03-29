/**
 * @file IMutableEnvironmentService.h
 * @brief 定义环境建模层的可变状态写入接口。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_I_MUTABLE_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_I_MUTABLE_ENVIRONMENT_SERVICE_H_

#include "1q/airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief IMutableEnvironmentService 在 IEnvironmentService 只读接口基础上暴露场景状态
 * 写入能力，供 Session 层调用及测试桩按接口注入。
 *
 * @details IEnvironmentService 仅供信号处理与决策层进行只读查询。
 * 需要变更环境状态的调用方（RadarSession、测试夹具）应通过本接口操作，
 * 避免直接依赖具体实现类型。
 */
class IMutableEnvironmentService : public IEnvironmentService {
 public:
  virtual ~IMutableEnvironmentService() = default;

  /**
   * @brief 更新待生效场景状态。
   * @param scene_state 新的待生效场景状态。
   */
  virtual void UpdateSceneState(const EnvironmentSceneState& scene_state) = 0;

  /**
   * @brief 更新环境模型配置。
   * @param config 新的环境模型配置。
   */
  virtual void UpdateModelConfig(const EnvironmentModelConfig& config) = 0;

  /**
   * @brief 设置干扰判定阈值。
   * @param threshold_db 干扰判定阈值，单位为 dB。
   */
  virtual void SetJammingDetectionThresholdDb(float threshold_db) = 0;
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_I_MUTABLE_ENVIRONMENT_SERVICE_H_
