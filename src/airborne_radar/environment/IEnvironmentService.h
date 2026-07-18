/**
 * @file IEnvironmentService.h
 * @brief 环境服务接口（内部实现细节，不对外暴露）。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/session/ArEnvironmentInput.h"

namespace airborne_radar {
namespace environment {

// Using declarations for types migrated to config:: and session::
using config::EnvironmentScenarioConfig;
using config::JammingSensitivityProfile;
using session::EnvironmentCycleContext;
using session::EnvironmentSnapshot;
using session::EnvironmentSceneState;

/**
 * @brief 环境服务运行态快照 (POD)，用于失败回滚等场景的整快照捕获/恢复。
 */
struct EnvironmentServiceRuntimeState {
  EnvironmentSceneState active_scene_state{};
  EnvironmentSceneState pending_scene_state{};
  EnvironmentCycleContext active_cycle_context{};
  JammingSensitivityProfile jamming_sensitivity_profile{JammingSensitivityProfile::kBalanced};
};

/**
 * @brief IEnvironmentService 为信号处理与决策层提供环境查询与运行态更新能力。
 */
class IEnvironmentService {
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
   * @brief 获取当前待生效场景状态。
   * @return 当前 pending 场景状态拷贝。
   */
  virtual EnvironmentSceneState GetPendingSceneState() const = 0;

  /**
   * @brief 更新环境模型配置。
   * @param[in] config 新环境模型配置。
   */
  virtual void UpdateModelConfig(const EnvironmentScenarioConfig& config) = 0;

  /**
   * @brief 设置干扰判定灵敏度语义档位。
   * @param[in] profile 灵敏度语义档位。
   */
    virtual void SetJammingSensitivityProfile(JammingSensitivityProfile profile) = 0;

  /**
   * @brief 捕获当前环境服务运行态快照。
   * @return 可用于失败回滚的环境运行态快照。
   */
  virtual EnvironmentServiceRuntimeState CaptureRuntimeState() const = 0;

  /**
   * @brief 恢复此前捕获的环境服务运行态快照。
   * @param[in] state 待恢复的环境运行态快照。
   */
  virtual void RestoreRuntimeState(const EnvironmentServiceRuntimeState& state) = 0;
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_I_ENVIRONMENT_SERVICE_H_
