/**
 * @file TrackLifecycleManager.h
 * @brief 定义轨迹生命周期管理器接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/ImmFilter.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/ITrackPool.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "airborne_radar/signal/tracking/TrackStateSnapshotEmitter.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief TrackLifecycleManager 负责轨迹生命周期状态推进。
 * 该类依赖 ITrackPool 管理对象复用，并提供快照导出能力。
 */
class TrackLifecycleManager : public ITrackLifecycleManager {
 public:
  /**
   * @brief 构造函数。
   * @param pool 轨迹对象池抽象。
   * @param config 生命周期阈值配置。
   * @param predictor 可选的 Kalman 预测器（为 nullptr 时不执行状态外推）。
   * @param updater 可选的 Kalman 更新器（为 nullptr 时不执行状态更新）。
   */
  TrackLifecycleManager(ITrackPool& pool, const LifecycleConfig& config,
                        IKalmanPredictor* predictor = nullptr,
                        IKalmanUpdater* updater = nullptr);
  /**
   * @brief 构造函数（IMM 多模型路径）。
   * @param pool 轨迹对象池抽象。
   * @param config 生命周期阈值配置。
   * @param imm_predictors IMM 各模型预测器集合（非拥有）。
   * @param imm_updaters IMM 各模型更新器集合（非拥有）。
   * @param imm_transition_probability 模型转移概率矩阵。
   * @param imm_initial_weights 模型初始权重。
   */
  TrackLifecycleManager(ITrackPool& pool, const LifecycleConfig& config,
                        const std::vector<IKalmanPredictor*>& imm_predictors,
                        const std::vector<IKalmanUpdater*>& imm_updaters,
                        const Eigen::MatrixXf& imm_transition_probability = Eigen::MatrixXf(),
                        const Eigen::VectorXf& imm_initial_weights = Eigen::VectorXf());

  ~TrackLifecycleManager() override;
  /**
   * @brief 用本周期量测更新轨迹生命周期。
   * @param cycle 当前周期上下文。
   * @param measurements 本周期关联后的量测集合。
   */
  void Update(const CycleContext& cycle,
              const std::vector<TrackMeasurement>& measurements) override;
  /**
   * @brief 导出当前活跃轨迹（只读指针）。
   * @return 活跃轨迹列表。
   */
  std::vector<const TrackState*> GetActiveTracks() const;
  /**
   * @brief 导出供外围事件链路消费的轻量目标特征快照。
   * @return 可直接用于外部状态广播的目标特征列表。
   */
  session::ArSceneTargetList BuildSceneTargetSnapshot() const override;
  /**
   * @brief 导出供决策层消费的活跃轨迹快照。
   * @return 包含 tentative/confirmed/lost 状态且未回收的决策快照列表。
   */
  session::TrackStateSnapshotList BuildTrackStateSnapshots() const override;

  /**
   * @brief 导出供关联阶段消费的轨迹种子。
   * @return 由当前未回收轨迹（tentative/confirmed/lost）构成的关联种子列表。
   */
  std::vector<AssociationTrackSeed> BuildAssociationSeeds() const override;
  TrackLifecycleRuntimeState CaptureRuntimeState() const override;
  void RestoreRuntimeState(const TrackLifecycleRuntimeState& state) override;
  void SyncRuntimeTuning(const LifecycleConfig& lifecycle_config, float kalman_noise_diff_coeff,
                         float kalman_measurement_noise_std,
                         const std::vector<float>& imm_model_noise_diff_coeffs,
                         const Eigen::MatrixXf& imm_transition_probability,
                         const Eigen::VectorXf& imm_initial_weights) override;

 private:
  // ------------------------------------------------------------------
  // 单周期更新的内部辅助类型（仅供 Update() 及 Phase 方法使用）
  // ------------------------------------------------------------------

  /** @brief 单条轨迹工作单元的命中类型。 */
  enum class WorkItemKind { kHit = 0, kMiss };

  /** @brief 单条轨迹在计算阶段的只读输入。 */
  struct TrackUpdateWorkItem {
    std::uint64_t association_key{0};
    WorkItemKind kind{WorkItemKind::kMiss};
    TrackState* track{nullptr};
    const TrackMeasurement* measurement{nullptr};
    bool track_existed_before_cycle{false};
    bool use_imm{false};
    ImmFilter* imm_filter{nullptr};
    TrackStatus status_before_update{TrackStatus::kTentative};
    TrackState track_before_update;
  };

  /** @brief 单条轨迹计算阶段产出的写回结果。 */
  struct TrackUpdateResult {
    std::uint64_t association_key{0};
    TrackState* track{nullptr};
    TrackState track_after_update;
    bool should_recycle{false};
  };

  /** @brief 聚合单周期生命周期更新的中间缓存。 */
  struct LifecycleUpdateScratch {
    std::unordered_map<std::uint64_t, const TrackMeasurement*> measurement_by_key;
    std::unordered_map<std::uint64_t, TrackState> track_snapshots;
    std::unordered_set<std::uint64_t> hit_keys;
    std::vector<TrackUpdateWorkItem> work_items;
    std::vector<TrackUpdateResult> results;
    std::vector<std::uint64_t> keys_to_recycle;
    std::size_t new_track_count{0};
    std::size_t updated_track_count{0};
    std::size_t predicted_without_hit_count{0};
  };

  // ------------------------------------------------------------------
  // Update() 内部 Phase 方法
  // ------------------------------------------------------------------

  /**
   * @brief Phase 1：建立量测路由并快照周期开始时的轨迹状态。
   * @param scratch 单周期中间缓存。
   * @param measurements 本周期量测集合。
   */
  void PreparePhase(LifecycleUpdateScratch& scratch,
                    const std::vector<TrackMeasurement>& measurements) const;
  /**
   * @brief Phase 2：串行补齐本周期所需的轨迹对象和 IMM 运行态，构造工作单元列表。
   * @param scratch 单周期中间缓存。
   * @param measurements 本周期量测集合。
   * @param cycle 当前周期上下文。
   */
  void EnsurePhase(LifecycleUpdateScratch& scratch,
                   const std::vector<TrackMeasurement>& measurements, const CycleContext& cycle);
  /**
   * @brief Phase 3：只读容器并按工作单元计算写回结果。
   * @param scratch 单周期中间缓存。
   * @param cycle 当前周期上下文。
   * @param effective_dt_sec 有效时间步长（秒）。
   */
  void ComputePhase(LifecycleUpdateScratch& scratch, const CycleContext& cycle,
                    float effective_dt_sec) const;
  /**
   * @brief Phase 4：串行写回所有轨迹结果并更新周期索引。
   * @param scratch 单周期中间缓存。
   * @param cycle_index 当前周期号。
   */
  void CommitPhase(LifecycleUpdateScratch& scratch, std::uint32_t cycle_index);
  /**
   * @brief Phase 5：串行回收已进入 recycled 状态的轨迹与 IMM 运行态。
   * @param scratch 单周期中间缓存。
   */
  void RecyclePhase(LifecycleUpdateScratch& scratch);

  /**
   * @brief 对命中工作单元应用 Kalman/IMM 滤波更新。
   * @param work_item 当前命中工作单元。
   * @param measurement 当前量测。
   * @param track 待更新轨迹状态（in-out）。
   * @param effective_dt_sec 有效时间步长（秒）。
   */
  void ApplyKalmanHitUpdate(const TrackUpdateWorkItem& work_item,
                            const TrackMeasurement& measurement, TrackState& track,
                            float effective_dt_sec) const;
  /**
   * @brief 对失配工作单元应用 Kalman/IMM 预测外推。
   * @param work_item 当前失配工作单元。
   * @param track 待更新轨迹状态（in-out）。
   * @param effective_dt_sec 有效时间步长（秒）。
   */
  void ApplyKalmanMissPredict(const TrackUpdateWorkItem& work_item, TrackState& track,
                              float effective_dt_sec) const;

  // ------------------------------------------------------------------
  // 其他内部辅助方法
  // ------------------------------------------------------------------

  /**
   * @brief 判断当前是否启用了 IMM 多模型路径。
   * @return 若已配置有效 IMM 模型集合则返回 true。
   */
  bool IsImmEnabled() const;
  /**
   * @brief 从组合量测构造初始高斯状态。
   * @param measurement 当前组合量测。
   * @return 初始化后的高斯状态。
   */
  GaussianTrackState BuildInitialGaussianState(const TrackMeasurement& measurement) const;
  /**
   * @brief 判断指定轨迹在当前命中周期是否应走 IMM 路径。
   * @param track_existed_before_cycle 命中前轨迹是否已存在。
   * @param status_before_update 命中前轨迹状态。
   * @param measurement 当前量测。
   * @return 若应创建或使用 IMM 路径则返回 true。
   */
  bool ShouldUseImmForMeasurement(bool track_existed_before_cycle, TrackStatus status_before_update,
                                  bool matched_existing_track) const;
  /**
   * @brief 判断指定轨迹在当前失配周期是否应走 IMM 预测路径。
   * @param status_before_prediction 预测前轨迹状态。
   * @return 若应使用 IMM 预测则返回 true。
   */
  bool ShouldUseImmForMiss(TrackStatus status_before_prediction) const;
  /**
   * @brief 获取或创建指定轨迹键对应的 IMM 运行态。
   * @param association_key 轨迹关联键。
   * @param initial_state 初始化状态。
   * @return IMM 运行态指针。
   */
  ImmFilter* GetOrCreateImmFilter(std::uint64_t association_key,
                                  const GaussianTrackState& initial_state);
  /**
   * @brief 查找指定轨迹键对应的 IMM 运行态。
   * @param association_key 轨迹关联键。
   * @return 若存在则返回 IMM 运行态指针，否则返回空指针。
   */
  ImmFilter* FindImmFilter(std::uint64_t association_key) const;
  /**
   * @brief 将高斯状态回写到轨迹的位置/速度，并由速度变化估计加速度。
   * @param track 待写回轨迹对象。
   * @param state 高斯状态。
   * @param previous_velocity 回写前速度，用于估计滤波加速度。
   * @param dt 周期步长（秒）。
   */
  void ApplyGaussianState(TrackState& track, const GaussianTrackState& state,
                          const Eigen::Vector3f& previous_velocity, float dt) const;
  /**
   * @brief 根据命中结果推进单条轨迹状态。
   * @param track 轨迹对象。
   * @param cycle_index 当前周期号。
   * @param hit_this_cycle 本周期是否命中。
   * @param extra_miss_tolerance 控制平面注入的额外失配容忍周期数。
   * @param classified_as_false_target 本周期量测是否被观测层判定为疑似假目标；
   *        启用假目标鉴别时，疑似假目标的量测不会把 tentative 航迹晋升为 confirmed。
   */
  void PromoteState(TrackState& track, std::uint32_t cycle_index, bool hit_this_cycle,
                    std::uint32_t extra_miss_tolerance,
                    bool classified_as_false_target = false) const;
  /**
   * @brief 将对象重置为可复用状态，避免脏数据泄露。
   * @note `track_id` 在重置时清零，下一次建轨时重新分配；
   *       `generation` 不在此处清零，保持对象复用代次单调递增语义。
   * @param track 待重置轨迹对象。
   */
  void ResetForReuse(TrackState& track) const;
  /**
   * @brief 校验并解析本周期状态估计使用的时间步长。
   * @param cycle 当前周期上下文。
   * @param effective_dt_sec [out] 解析得到的有效时间步长（秒）。
   * @return 输入 `dt_sec` 有效时返回 true，否则返回 false。
   */
  bool TryResolveEffectiveCycleDeltaTimeSec(const CycleContext& cycle,
                                            float* effective_dt_sec) const;
  /**
   * @brief 遍历所有未回收轨迹，对每条轨迹调用 callback(key, track)。
   * @tparam Callback 可调用类型，签名为 void(std::uint64_t, const TrackState&)。
   * @param callback 每条活跃轨迹的处理逻辑。
   */
  template <typename Callback>
  void ForEachActiveTrack(Callback&& callback) const {
    for (std::unordered_map<std::uint64_t, TrackState*>::const_iterator it =
             tracks_by_key_.cbegin();
         it != tracks_by_key_.cend(); ++it) {
      if (it->second->status != TrackStatus::kRecycled) {
        callback(it->first, *it->second);
      }
    }
  }

  // data members
  ITrackPool* pool_{nullptr};      /**< 轨迹对象池抽象，用于对象申请与归还。 */
  LifecycleConfig config_;         /**< 生命周期阈值配置。 */
  std::uint64_t next_track_id_{1}; /**< 下一个待分配轨迹 ID。 */
  std::unordered_map<std::uint64_t, TrackState*> tracks_by_key_; /**< 活跃轨迹表，key 为关联键。 */
  IKalmanPredictor* kalman_predictor_{nullptr};   /**< 可选 Kalman 预测器（非拥有）。 */
  IKalmanUpdater* kalman_updater_{nullptr};       /**< 可选 Kalman 更新器（非拥有）。 */
  std::vector<IKalmanPredictor*> imm_predictors_; /**< IMM 各模型预测器集合（非拥有）。 */
  std::vector<IKalmanUpdater*> imm_updaters_;     /**< IMM 各模型更新器集合（非拥有）。 */
  Eigen::MatrixXf imm_transition_probability_;          /**< IMM 模型转移概率矩阵。 */
  Eigen::VectorXf imm_initial_weights_;                 /**< IMM 模型初始权重。 */
  std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter>>
      imm_filters_by_key_;                     /**< 每条轨迹对应的 IMM 运行态。 */
  std::uint32_t last_cycle_index_{0};          /**< 上一周期编号，用于计算 dt。 */
  TrackStateSnapshotEmitter snapshot_emitter_; /**< 快照导出器，每周期末刷新。 */
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_
