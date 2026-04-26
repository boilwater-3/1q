/**
 * @file ITrackLifecycleManager.h
 * @brief 定义轨迹生命周期管理抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_LIFECYCLE_MANAGER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_LIFECYCLE_MANAGER_H_

#include <vector>


#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief ITrackLifecycleManager 抽象轨迹生命周期推进与快照导出能力。
 */
class ITrackLifecycleManager {
 public:
  virtual ~ITrackLifecycleManager() = default;
  /**
   * @brief 用本周期量测更新轨迹状态机。
   * @param cycle 周期上下文。
   * @param measurements 关联后量测列表。
   */
  virtual void Update(const CycleContext& cycle,
                      const std::vector<TrackMeasurement>& measurements) = 0;
  /**
   * @brief 导出供事件广播和外围观测消费的目标特征快照。
   * @return 获取到的轻量级目标特征列表。
   */
  virtual session::RadarSceneTargetList BuildSceneTargetSnapshot() const = 0;
  /**
   * @brief 导出供决策引擎消费的活跃轨迹快照。
   * @return 包含 tentative/confirmed/lost 状态且未回收的轨迹列表。
   */
  virtual model::TrackStateSnapshotList BuildTrackStateSnapshots() const = 0;
  /**
   * @brief 导出供关联阶段使用的上一周期轨迹种子。
   * @return 由当前未回收轨迹（tentative/confirmed/lost）组成的关联候选列表。
   */
  virtual std::vector<AssociationTrackSeed> BuildAssociationSeeds() const = 0;
  /**
   * @brief 同步运行时 lifecycle/KF/IMM 参数。
   * @note 默认空实现，供不支持在线调参的实现安全忽略。
   */
  virtual void SyncRuntimeTuning(const LifecycleConfig& lifecycle_config,
                                 float kalman_noise_diff_coeff,
                                 float kalman_measurement_noise_std,
                                 const std::vector<float>& imm_model_noise_diff_coeffs,
                                 const Eigen::MatrixXf& imm_transition_probability,
                                 const Eigen::VectorXf& imm_initial_weights) {
    (void)lifecycle_config;
    (void)kalman_noise_diff_coeff;
    (void)kalman_measurement_noise_std;
    (void)imm_model_noise_diff_coeffs;
    (void)imm_transition_probability;
    (void)imm_initial_weights;
  }
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_LIFECYCLE_MANAGER_H_
