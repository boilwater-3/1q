/**
 * @file EnvironmentService.h
 * @brief 定义环境建模层的基础实现。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_

#include <memory>

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {

class SceneManager;
class PropagationModel;

/**
 * @brief 提供可配置的环境快照采样实现。
 *
 * @note 线程安全模型：本类假定单线程调用。BeginCycle()、UpdateSceneState()、
 *       SampleEnvironment() 不得跨线程并发调用。建议每线程独立持有
 *       EnvironmentService 实例，或由调用方序列化访问。
 */
class EnvironmentService final : public environment::IEnvironmentService {
 public:
  /**
   * @brief 使用配置构造环境模型。
   * @param config 环境模型初始配置。
   */
  explicit EnvironmentService(const EnvironmentModelConfig& config = {});
  ~EnvironmentService() override;

  /**
   * @brief 冻结当前周期环境事实。
   * @param cycle_context 当前周期上下文。
   */
  void BeginCycle(const EnvironmentCycleContext& cycle_context) override;

  /**
   * @brief 采样并返回当前处理周期的环境条件。
   * @return 当前周期冻结的环境快照。
   */
  EnvironmentSnapshot SampleEnvironment() const override;

  /**
   * @brief 更新待生效场景状态。
   * @param scene_state 新的待生效场景状态。
   */
  void UpdateSceneState(const EnvironmentSceneState& scene_state) override;

  /**
   * @brief 获取当前待生效场景状态。
   * @return 当前 pending 场景状态拷贝。
   */
  EnvironmentSceneState GetPendingSceneState() const override;

  /**
   * @brief 更新环境模型配置。
   * @param config 新的环境模型配置。
   */
  void UpdateModelConfig(const EnvironmentModelConfig& config) override;

  /**
    * @brief 设置干扰判定灵敏度语义档位。
    * @param profile 干扰判定灵敏度语义档位。
    */
  void SetJammingSensitivityProfile(JammingSensitivityProfile profile) override;

  environment::EnvironmentServiceRuntimeState CaptureRuntimeState() const override;

  void RestoreRuntimeState(const environment::EnvironmentServiceRuntimeState& state) override;

 private:
  void RefreshFrozenSnapshotFromActiveScene();

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<PropagationModel> propagation_model_;
  EnvironmentSnapshot frozen_snapshot_{};
  EnvironmentCycleContext current_cycle_context_{};
  JammingSensitivityProfile jamming_sensitivity_profile_{JammingSensitivityProfile::kBalanced};
  float effective_jamming_detection_threshold_db_{
      internal::ResolveJammingDetectionThresholdDb(JammingSensitivityProfile::kBalanced)};
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
