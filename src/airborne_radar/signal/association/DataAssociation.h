/**
 * @file DataAssociation.h
 * @brief 定义基于距离度量与线性指派的数据关联组件。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/association/Hypothesiser.h"
#include "airborne_radar/signal/association/LapjvSolver.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace association {
/**
 * @brief 数据关联配置。
 */
struct DataAssociationConfig {
  float unassigned_cost{9.0f};               /**< 未分配虚拟槽代价上限。 */
  float kalman_noise_diff_coeff{1.0f};       /**< 位置关联使用的过程噪声扩散系数。 */
  float kalman_measurement_noise_std{10.0f}; /**< 位置关联使用的默认量测噪声标准差（米）。 */
};
/**
 * @brief 单个成功关联结果。
 */
struct AssociationMatch {
  AssociationMatch() = default;
  /**
   * @brief 参数构造。
   */
  AssociationMatch(std::uint64_t key, std::size_t target_idx, float c)
      : association_key(key), target_index(target_idx), cost(c) {}
  std::uint64_t association_key{0}; /**< 稳定关联键。 */
  std::size_t target_index{0};      /**< 输入目标在当前批次中的索引。 */
  float cost{0.0f};                 /**< 本次关联代价。 */
};
/**
 * @brief 一次关联计算的质量观测指标快照。
 */
struct AssociationQualityMetrics {
  std::size_t prior_track_count{0};  /**< 进入关联阶段的历史先验轨迹数。 */
  std::size_t detection_count{0};    /**< 本周期探测成功并参与关联的量测数。 */
  std::size_t matched_count{0};      /**< 命中已有轨迹的关联数。 */
  std::size_t new_track_count{0};    /**< 触发新建轨迹键的量测数。 */
  std::size_t missed_track_count{0}; /**< 未命中任何量测的历史轨迹数。 */
  float match_rate{0.0f};            /**< 命中率（matched_count / detection_count）。 */
  float new_track_rate{0.0f};        /**< 新生率（new_track_count / detection_count）。 */
  float missed_track_rate{0.0f};     /**< 漏失率（missed_track_count / prior_track_count）。 */
  float mean_match_cost{0.0f};       /**< 命中关联代价均值（仅统计 matches）。 */
  float p95_match_cost{0.0f};        /**< 命中关联代价 P95（仅统计 matches）。 */
};
/**
 * @brief 一次关联计算的完整输出。
 */
struct AssociationResult {
  std::vector<AssociationMatch> matches;        /**< 成功命中已有轨迹的关联结果列表。 */
  std::vector<std::uint64_t> missed_track_keys; /**< 本周期未命中量测的历史轨迹键。 */
  std::vector<std::size_t>
      unassociated_target_indices;        /**< 本周期未命中历史轨迹、需要新建键的目标索引。 */
  std::vector<std::uint64_t> target_keys; /**< 与输入目标索引对齐的稳定关联键列表。 */
  bool used_position_association{false};  /**< 本周期是否实际执行了位置空间关联路径。 */
  bool used_external_association_seeds{
      false}; /**< 本周期是否使用了外部注入的轨迹种子作为关联先验。 */
  AssociationQualityMetrics quality_metrics; /**< 本周期关联质量观测指标。 */
};

struct DataAssociationRuntimeState {
  std::uint64_t next_key{1U};
  bool using_external_seeds{false};
  std::vector<tracking::AssociationTrackSeed> external_seeds{};
};
/**
 * @brief 数据关联引擎。
 * @details 仅消费外部注入的 Lifecycle 轨迹种子作为关联先验；无外部 seeds 时按无先验模式执行。
 */
class DataAssociationEngine {
 public:
  /**
   * @brief 构造数据关联引擎。
   * @param config 关联配置。
   */
  explicit DataAssociationEngine(DataAssociationConfig config = {});
  /**
   * @brief 更新关联配置。
   * @param config 新配置。
   */
  void UpdateConfig(DataAssociationConfig config);
  /**
   * @brief 关联当前周期探测并返回结构化结果。
   * @param targets 当前周期输入目标特征集合。
   * @param detection_succeeded 探测阶段输出的有效标记。
   * @param dt_sec 当前周期实际步长（单位：s），用于先验预测。默认值 `1.0f` 向后兼容。
   * @return 包含命中、失配和未关联目标的完整结果。
   */
  AssociationResult AssociateDetections(const session::RadarSceneTargetList& targets,
                                        const std::vector<std::uint8_t>& detection_succeeded,
                                        float dt_sec = 1.0f);
  /**
   * @brief 关联当前周期探测并返回结构化结果（使用动态量测协方差）。
   * @param targets 当前周期输入目标特征集合。
   * @param detection_succeeded 探测阶段输出的有效标记。
   * @param measurement_covariances 与目标索引对齐的动态量测协方差。
   * @param dt_sec 当前周期实际步长（单位：s），用于先验预测。默认值 `1.0f` 向后兼容。
   * @return 包含命中、失配和未关联目标的完整结果。
   */
  AssociationResult AssociateDetections(
      const session::RadarSceneTargetList& targets,
      const std::vector<std::uint8_t>& detection_succeeded,
      const std::vector<tracking::MeasurementCovariance>& measurement_covariances,
      float dt_sec = 1.0f);
  /**
   * @brief 关联当前周期探测并返回稳定关联键。
   * @param targets 当前周期输入目标特征集合。
   * @param detection_succeeded 探测阶段输出的有效标记。
   * @return 与目标索引对齐的关联键列表；0 表示未关联。
   */
  std::vector<std::uint64_t> Associate(const session::RadarSceneTargetList& targets,
                                       const std::vector<std::uint8_t>& detection_succeeded);
  /**
   * @brief 关联当前周期探测并返回稳定关联键（使用动态量测协方差）。
   * @param targets 当前周期输入目标特征集合。
   * @param detection_succeeded 探测阶段输出的有效标记。
   * @param measurement_covariances 与目标索引对齐的动态量测协方差。
   * @return 与目标索引对齐的关联键列表；0 表示未关联。
   */
  std::vector<std::uint64_t> Associate(
      const session::RadarSceneTargetList& targets,
      const std::vector<std::uint8_t>& detection_succeeded,
      const std::vector<tracking::MeasurementCovariance>& measurement_covariances);
  /**
   * @brief 使用生命周期侧导出的轨迹种子覆盖关联引擎历史状态。
   * @param seeds 上一周期轨迹种子列表。
   * @return 若成功进入 external-seed 模式则返回 true；空输入或非法输入会回退 stateless 并返回
   *         false。
   */
  bool SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds);
  /**
   * @brief 重置外部 seeds 注入状态并回归无先验（stateless）模式。
   * @details 用于收敛 external seeds 单一入口，清理旁路注入的临时外部 seeds。
   */
  void ResetAssociationSeedModeToStateless();

  DataAssociationRuntimeState CaptureRuntimeState() const;

  void RestoreRuntimeState(const DataAssociationRuntimeState& state);

 private:
  enum class AssociationSeedMode {
    kStateless = 0, /**< 无 external seeds 时的无先验关联模式。 */
    kExternalSeeds, /**< 使用外部注入的 Lifecycle 轨迹种子。 */
  };
  /**
   * @brief 外部 Lifecycle 注入的关联种子签名。
   */
  struct ExternalSeedTrackSignature {
    ExternalSeedTrackSignature() = default;
    /**
     * @brief 参数构造。
     */
    explicit ExternalSeedTrackSignature(std::uint64_t k) : key(k) {}
    std::uint64_t key{0};                              /**< 历史轨迹稳定键。 */
    bool has_position{false};                          /**< 当前轨迹是否具有笛卡尔位置量测。 */
    Eigen::Vector3f position{Eigen::Vector3f::Zero()}; /**< 当前轨迹位置量测/预测量测。 */
    bool has_gaussian_state{false};                    /**< 当前轨迹是否具有高斯状态。 */
    tracking::GaussianTrackState gaussian_state;       /**< 用于位置空间关联预测的高斯状态。 */
  };
  /**
   * @brief 位置空间关联输入先验的统一视图。
   */
  struct PositionAssociationPriors {
    std::vector<std::uint64_t> keys;    /**< 与预测轨迹按行对齐的稳定键列表。 */
    FeatureVectorList predicted_tracks; /**< 进入位置空间代价计算的预测轨迹集合。 */
    std::vector<Eigen::Matrix3f>
        projected_measurement_covariances; /**< 与预测轨迹按行对齐的投影协方差 HPH^T。 */
  };
  /**
   * @brief 构建笛卡尔位置量测向量。
   * @param target 输入目标特征。
   * @return 由 x、y、z 组成的三维位置向量。
   */
  Eigen::Vector3f BuildPositionVector(const session::RadarSceneTarget& target) const;
  /**
   * @brief 判断目标是否带有可用的笛卡尔位置量测。
   * @param target 输入目标特征。
   * @return 若存在位置量测则返回 true。
   */
  bool HasPositionMeasurement(const session::RadarSceneTarget& target) const;
  /**
   * @brief 从位置量测初始化高斯状态。
   * @param position 位置量测。
   * @return 初始化后的高斯状态。
   */
  tracking::GaussianTrackState InitializeGaussianState(const Eigen::Vector3f& position) const;
  /**
   * @brief 计算预测状态投影到量测空间后的协方差 HPH^T。
   * @param predicted 预测后的高斯状态。
   * @return 对应的位置量测空间协方差。
   */
  tracking::MeasurementCovariance ComputeProjectedMeasurementCovariance(
      const tracking::GaussianTrackState& predicted) const;
  /**
   * @brief 当前周期是否使用外部注入的 Lifecycle seeds。
   */
  bool UsingExternalSeeds() const;
  /**
   * @brief 清理本周期消费后的先验状态。
   */
  void ClearConsumedAssociationPriors();
  /**
   * @brief 构建 external-seed 路径的 position-only 关联先验视图。
   * @param external_priors 外部注入的关联先验。
   * @return 位置空间关联统一先验。
   */
  PositionAssociationPriors BuildExternalPositionAssociationPriors(
      const std::vector<ExternalSeedTrackSignature>& external_priors, float dt_sec) const;
  DataAssociationConfig config_{};                     /**< 当前引擎配置。 */
  FullMahalanobisDistanceMetric full_distance_metric_; /**< 完整协方差位置空间马氏距离度量实现。 */
  DenseCostHypothesiser position_hypothesiser_;        /**< 位置空间候选假设生成器。 */
  LapjvSolver assignment_solver_;                      /**< LAPJV 指派求解器。 */
  tracking::KalmanPredictor kalman_predictor_;         /**< 位置空间关联使用的内部预测器。 */
  std::uint64_t next_key_{1};                          /**< 下一次分配给新目标的稳定键。 */
  std::vector<ExternalSeedTrackSignature>
      external_seed_tracks_; /**< 当前周期外部注入的 Lifecycle 轨迹种子缓存。 */
  AssociationSeedMode association_seed_mode_{
      AssociationSeedMode::kStateless}; /**< 当前周期关联先验状态来源模式。 */
};

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_
