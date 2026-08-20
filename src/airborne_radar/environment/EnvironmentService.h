/**
 * @file EnvironmentService.h
 * @brief 定义环境建模层的基础实现。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_

#include <memory>

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {

// Using declarations for types migrated to config:: and session::
using config::EnvironmentScenarioConfig;
using session::EnvironmentCycleContext;
using session::EnvironmentSnapshot;
using session::EnvironmentSceneState;

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
   * @param[in] config 环境模型初始配置。
   */
  explicit EnvironmentService(const EnvironmentScenarioConfig& config = {});
  ~EnvironmentService() override;

  /**
   * @brief 冻结当前周期环境事实。
   * @param[in] cycle_context 当前周期上下文。
   */
  void BeginCycle(const EnvironmentCycleContext& cycle_context) override;

  /**
   * @brief 采样并返回当前处理周期的环境条件。
   * @return 当前周期冻结的环境快照。
   */
  EnvironmentSnapshot SampleEnvironment() const override;

  /**
   * @brief 更新待生效场景状态。
   * @param[in] scene_state 新的待生效场景状态。
   */
  void UpdateSceneState(const EnvironmentSceneState& scene_state) override;

  /**
   * @brief 获取当前待生效场景状态。
   * @return 当前 pending 场景状态拷贝。
   */
  EnvironmentSceneState GetPendingSceneState() const override;

  /**
   * @brief 更新环境模型配置。
   * @param[in] config 新的环境模型配置。
   */
  void UpdateModelConfig(const EnvironmentScenarioConfig& config) override;

  /**
   * @brief 捕获当前环境服务运行态快照。
   * @return 可用于失败回滚的环境运行态快照。
   */
  environment::EnvironmentServiceRuntimeState CaptureRuntimeState() const override;

  /**
   * @brief 恢复此前捕获的环境服务运行态快照。
   * @param[in] state 待恢复的环境运行态快照。
   */
  void RestoreRuntimeState(const environment::EnvironmentServiceRuntimeState& state) override;

 private:
  void RefreshFrozenSnapshotFromActiveScene();

  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<PropagationModel> propagation_model_;
  EnvironmentSnapshot frozen_snapshot_{};
  EnvironmentCycleContext current_cycle_context_{};
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
