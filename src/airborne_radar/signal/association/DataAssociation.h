// Copyright 2026. All Rights Reserved.
//
// 文件说明：定义基于距离度量与线性指派的数据关联组件。

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/tracking/GaussianTrackState.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/association/Gater.h"
#include "airborne_radar/signal/association/Hypothesiser.h"
#include "airborne_radar/signal/association/LapjvSolver.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace association {

/// @brief 数据关联配置。
struct DataAssociationConfig {
	/// @brief 未分配虚拟槽代价上限。
  float unassigned_cost{9.0f};

	/// @brief 位置关联使用的过程噪声扩散系数。
  float kalman_noise_diff_coeff{1.0f};

	/// @brief 位置关联使用的默认量测噪声标准差（米）。
  float kalman_measurement_noise_std{10.0f};
};

/// @brief 单个成功关联结果。
struct AssociationMatch {
	/// @brief 默认构造。
  AssociationMatch() = default;
	/// @brief 参数构造。
  AssociationMatch(std::uint64_t key, std::size_t target_idx, float c)
      : association_key(key), target_index(target_idx), cost(c) {}

	/// @brief 稳定关联键。
  std::uint64_t association_key{0};
	/// @brief 输入目标在当前批次中的索引。
  std::size_t target_index{0};
	/// @brief 本次关联代价。
  float cost{0.0f};
};

/// @brief 一次关联计算的完整输出。
struct AssociationResult {
	/// @brief 成功命中已有轨迹的关联结果列表。
  std::vector<AssociationMatch> matches;
	/// @brief 本周期未命中量测的历史轨迹键。
  std::vector<std::uint64_t> missed_track_keys;
	/// @brief 本周期未命中历史轨迹、需要新建键的目标索引。
  std::vector<std::size_t> unassociated_target_indices;
	/// @brief 与输入目标索引对齐的稳定关联键列表。
  std::vector<std::uint64_t> target_keys;
	/// @brief 本周期是否实际执行了位置空间关联路径。
  bool used_position_association{false};
	/// @brief 本周期是否使用了外部注入的轨迹种子作为关联先验。
  bool used_external_association_seeds{false};
};

/// @brief 数据关联引擎。
/// @details 维护轻量历史轨迹签名，并将当前周期有效探测关联到上一周期轨迹。
class DataAssociationEngine {
public:
	/// @brief 构造数据关联引擎。
	/// @param config 关联配置。
  explicit DataAssociationEngine(DataAssociationConfig config = {});

	/// @brief 更新关联配置。
	/// @param config 新配置。
  void UpdateConfig(DataAssociationConfig config);

	/// @brief 关联当前周期探测并返回结构化结果。
	/// @param targets 当前周期输入目标特征集合。
	/// @param detection_succeeded 探测阶段输出的有效标记。
	/// @return 包含命中、失配和未关联目标的完整结果。
  AssociationResult AssociateDetections(
      const common::TargetFeatureList &targets,
      const std::vector<std::uint8_t> &detection_succeeded);

	/// @brief 关联当前周期探测并返回结构化结果（使用动态量测协方差）。
	/// @param targets 当前周期输入目标特征集合。
	/// @param detection_succeeded 探测阶段输出的有效标记。
	/// @param measurement_covariances 与目标索引对齐的动态量测协方差。
	/// @return 包含命中、失配和未关联目标的完整结果。
  AssociationResult AssociateDetections(
	  const common::TargetFeatureList &targets,
	  const std::vector<std::uint8_t> &detection_succeeded,
	  const std::vector<tracking::MeasurementCovariance> &measurement_covariances);

	/// @brief 关联当前周期探测并返回稳定关联键。
	/// @param targets 当前周期输入目标特征集合。
	/// @param detection_succeeded 探测阶段输出的有效标记。
	/// @return 与目标索引对齐的关联键列表；0 表示未关联。
  std::vector<std::uint64_t> Associate(
      const common::TargetFeatureList &targets,
      const std::vector<std::uint8_t> &detection_succeeded);

	/// @brief 关联当前周期探测并返回稳定关联键（使用动态量测协方差）。
	/// @param targets 当前周期输入目标特征集合。
	/// @param detection_succeeded 探测阶段输出的有效标记。
	/// @param measurement_covariances 与目标索引对齐的动态量测协方差。
	/// @return 与目标索引对齐的关联键列表；0 表示未关联。
  std::vector<std::uint64_t> Associate(
	  const common::TargetFeatureList &targets,
	  const std::vector<std::uint8_t> &detection_succeeded,
	  const std::vector<tracking::MeasurementCovariance> &measurement_covariances);

	/// @brief 使用生命周期侧导出的轨迹种子覆盖关联引擎历史状态。
	/// @param seeds 上一周期轨迹种子列表。
  void SetAssociationSeeds(
      const std::vector<tracking::AssociationTrackSeed> &seeds);

private:
	/// @brief 关联先验状态来源模式。
	enum class AssociationSeedMode {
		/// @brief 使用关联引擎内部维护的 fallback 兼容历史缓存。
		kFallbackHistoryCache = 0,
		/// @brief 使用外部注入的 Lifecycle 轨迹种子。
		kExternalSeeds,
	};

	/// @brief 轻量轨迹签名。
  struct TrackSignature {
		/// @brief 默认构造。
    TrackSignature() = default;
		/// @brief 参数构造。
		explicit TrackSignature(std::uint64_t k)
				: key(k) {}

		/// @brief 历史轨迹稳定键。
    std::uint64_t key{0};
		/// @brief 当前轨迹是否具有笛卡尔位置量测。
		bool has_position{false};
		/// @brief 当前轨迹位置量测/预测量测。
		Eigen::Vector3f position{Eigen::Vector3f::Zero()};
		/// @brief 当前轨迹是否具有高斯状态。
		bool has_gaussian_state{false};
		/// @brief 用于位置空间关联预测的高斯状态。
		tracking::GaussianTrackState gaussian_state;
  };

	/// @brief 构建笛卡尔位置量测向量。
	/// @param target 输入目标特征。
	/// @return 由 x、y、z 组成的三维位置向量。
  Eigen::Vector3f BuildPositionVector(const common::TargetFeature &target) const;

	/// @brief 判断目标是否带有可用的笛卡尔位置量测。
	/// @param target 输入目标特征。
	/// @return 若存在位置量测则返回 true。
  bool HasPositionMeasurement(const common::TargetFeature &target) const;

	/// @brief 校验所有成功探测目标都携带笛卡尔位置量测。
	/// @param targets 当前周期输入目标。
	/// @param detection_succeeded 当前周期探测标记。
  void ValidateDetectedTargetsHavePosition(
      const common::TargetFeatureList &targets,
      const std::vector<std::uint8_t> &detection_succeeded) const;

	/// @brief 从位置量测初始化高斯状态。
	/// @param position 位置量测。
	/// @return 初始化后的高斯状态。
  tracking::GaussianTrackState InitializeGaussianState(
      const Eigen::Vector3f &position) const;

	/// @brief 计算预测状态投影到量测空间后的协方差 HPH^T。
	/// @param predicted 预测后的高斯状态。
 	/// @return 对应的位置量测空间协方差。
  tracking::MeasurementCovariance ComputeProjectedMeasurementCovariance(
      const tracking::GaussianTrackState &predicted) const;

	/// @brief 当前引擎配置。
  DataAssociationConfig config_{};
	/// @brief 完整协方差位置空间马氏距离度量实现。
  FullMahalanobisDistanceMetric full_distance_metric_;
	/// @brief 基于代价阈值的波门器。
  CostThresholdGater gater_;
	/// @brief 位置空间候选假设生成器。
  DenseCostHypothesiser position_hypothesiser_;
	/// @brief LAPJV 指派求解器。
  LapjvSolver assignment_solver_;
	/// @brief 位置空间关联使用的内部预测器。
  tracking::KalmanPredictor kalman_predictor_;
	/// @brief 位置空间关联使用的内部更新器。
  tracking::KalmanUpdater kalman_updater_;
	/// @brief 下一次分配给新目标的稳定键。
  std::uint64_t next_key_{1};
	/// @brief 仅在未接入 Lifecycle 种子时启用的 fallback 兼容历史缓存。
  std::vector<TrackSignature> fallback_history_tracks_;
	/// @brief 当前周期外部注入的 Lifecycle 轨迹种子缓存。
	std::vector<TrackSignature> external_seed_tracks_;
	/// @brief 当前周期关联先验状态来源模式。
  AssociationSeedMode association_seed_mode_{AssociationSeedMode::kFallbackHistoryCache};
};

} // namespace association
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_
