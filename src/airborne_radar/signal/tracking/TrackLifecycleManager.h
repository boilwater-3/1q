// Copyright 2026. All Rights Reserved.
//
// Description: 定义轨迹生命周期管理器接口。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "1q/airborne_radar/signal/tracking/LifecycleConfig.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "airborne_radar/signal/tracking/ITrackPool.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

class IKalmanPredictor;
class IKalmanUpdater;
class ImmFilter;

/// @brief TrackLifecycleManager 负责轨迹生命周期状态推进。
/// 该类依赖 ITrackPool 管理对象复用，并提供快照导出能力。
class TrackLifecycleManager : public ITrackLifecycleManager {
public:
  /// @brief 构造函数。
  /// @param pool 轨迹对象池抽象。
  /// @param config 生命周期阈值配置。
  /// @param predictor 可选的 Kalman 预测器（为 nullptr 时不执行状态外推）。
  /// @param updater 可选的 Kalman 更新器（为 nullptr 时不执行状态更新）。
  TrackLifecycleManager(ITrackPool &pool, const LifecycleConfig &config,
                        const IKalmanPredictor *predictor = nullptr,
                        const IKalmanUpdater *updater = nullptr);

  /// @brief 构造函数（IMM 多模型路径）。
  /// @param pool 轨迹对象池抽象。
  /// @param config 生命周期阈值配置。
  /// @param imm_predictors IMM 各模型预测器集合（非拥有）。
  /// @param imm_updaters IMM 各模型更新器集合（非拥有）。
  /// @param imm_transition_probability 模型转移概率矩阵。
  /// @param imm_initial_weights 模型初始权重。
  TrackLifecycleManager(
      ITrackPool &pool, const LifecycleConfig &config,
      const std::vector<const IKalmanPredictor *> &imm_predictors,
      const std::vector<const IKalmanUpdater *> &imm_updaters,
      const Eigen::MatrixXf &imm_transition_probability = Eigen::MatrixXf(),
      const Eigen::VectorXf &imm_initial_weights = Eigen::VectorXf());

  ~TrackLifecycleManager() override;

  /// @brief 用本周期量测更新轨迹生命周期。
  /// @param cycle 当前周期上下文。
  /// @param measurements 本周期关联后的量测集合。
  void Update(const CycleContext &cycle,
              const std::vector<TrackMeasurement> &measurements) override;

  /// @brief 导出当前活跃轨迹（只读指针）。
  /// @return 活跃轨迹列表。
  std::vector<const common::TrackState *> GetActiveTracks() const;

  /// @brief 导出兼容现有决策链路的轻量目标特征快照。
  /// @return 可直接用于 DecisionContext 的目标特征列表。
  common::TargetFeatureList BuildFeatureSnapshot() const override;

  /// @brief 导出供关联阶段消费的轨迹种子。
  /// @return 由当前活跃轨迹构成的关联种子列表。
  std::vector<AssociationTrackSeed> BuildAssociationSeeds() const override;

private:

  /// @brief 判断当前是否启用了 IMM 多模型路径。
  /// @return 若已配置有效 IMM 模型集合则返回 true。
  bool IsImmEnabled() const;

  /// @brief 从组合量测构造初始高斯状态。
  /// @param measurement 当前组合量测。
  /// @return 初始化后的高斯状态。
  GaussianTrackState BuildInitialGaussianState(
      const TrackMeasurement &measurement) const;

  /// @brief 判断指定轨迹在当前命中周期是否应走 IMM 路径。
  /// @param track_existed_before_cycle 命中前轨迹是否已存在。
  /// @param status_before_update 命中前轨迹状态。
  /// @param measurement 当前量测。
  /// @return 若应创建或使用 IMM 路径则返回 true。
  bool ShouldUseImmForMeasurement(
      bool track_existed_before_cycle,
      common::TrackStatus status_before_update,
      const TrackMeasurement &measurement) const;

  /// @brief 判断指定轨迹在当前失配周期是否应走 IMM 预测路径。
  /// @param status_before_prediction 预测前轨迹状态。
  /// @return 若应使用 IMM 预测则返回 true。
  bool ShouldUseImmForMiss(common::TrackStatus status_before_prediction) const;

  /// @brief 获取或创建指定轨迹键对应的 IMM 运行态。
  /// @param association_key 轨迹关联键。
  /// @param initial_state 初始化状态。
  /// @return IMM 运行态指针。
  ImmFilter *GetOrCreateImmFilter(std::uint64_t association_key,
                                  const GaussianTrackState &initial_state);

  /// @brief 查找指定轨迹键对应的 IMM 运行态。
  /// @param association_key 轨迹关联键。
  /// @return 若存在则返回 IMM 运行态指针，否则返回空指针。
  ImmFilter *FindImmFilter(std::uint64_t association_key) const;

  /// @brief 将高斯状态回写到轨迹的位置/速度，并由速度变化估计加速度。
  /// @param track 待写回轨迹对象。
  /// @param state 高斯状态。
  /// @param previous_velocity 回写前速度，用于估计滤波加速度。
  /// @param dt 周期步长（秒）。
  void ApplyGaussianState(common::TrackState &track,
                          const GaussianTrackState &state,
                          const Eigen::Vector3f &previous_velocity,
                          float dt) const;

  /// @brief 根据命中结果推进单条轨迹状态。
  /// @param track 轨迹对象。
  /// @param cycle_index 当前周期号。
  /// @param hit_this_cycle 本周期是否命中。
  void PromoteState(common::TrackState &track,
                    std::uint32_t cycle_index,
                    bool hit_this_cycle);

  /// @brief 将对象重置为可复用状态，避免脏数据泄露。
  /// @param track 待重置轨迹对象。
  void ResetForReuse(common::TrackState &track) const;

  /// @brief 解析本周期状态估计使用的有效时间步长。
  /// @param cycle 当前周期上下文。
  /// @param dt_fallback_used 输出是否触发内部兜底。
  /// @return 有效时间步长（秒）。
  float ResolveEffectiveCycleDeltaTimeSec(const CycleContext &cycle,
                                          bool *dt_fallback_used) const;

private:
	/// @brief 轨迹对象池抽象，用于对象申请与归还。
  ITrackPool *pool_{nullptr};

  /// @brief 可选线程安全对象池包装器。
  std::unique_ptr<ITrackPool> owned_pool_wrapper_;

	/// @brief 生命周期阈值配置。
  LifecycleConfig config_;

	/// @brief 下一个待分配轨迹 ID。
  std::uint64_t next_track_id_{1};

  /// @brief 活跃轨迹表，key 为关联键。
  std::unordered_map<std::uint64_t, common::TrackState *> tracks_by_key_;

  /// @brief 可选 Kalman 预测器（非拥有）。
  const IKalmanPredictor *kalman_predictor_{nullptr};

  /// @brief 可选 Kalman 更新器（非拥有）。
  const IKalmanUpdater *kalman_updater_{nullptr};

  /// @brief IMM 各模型预测器集合（非拥有）。
  std::vector<const IKalmanPredictor *> imm_predictors_;

  /// @brief IMM 各模型更新器集合（非拥有）。
  std::vector<const IKalmanUpdater *> imm_updaters_;

  /// @brief IMM 模型转移概率矩阵。
  Eigen::MatrixXf imm_transition_probability_;

  /// @brief IMM 模型初始权重。
  Eigen::VectorXf imm_initial_weights_;

  /// @brief 每条轨迹对应的 IMM 运行态。
  std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter> >
      imm_filters_by_key_;

  /// @brief 上一周期编号，用于计算 dt。
  std::uint32_t last_cycle_index_{0};
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_
