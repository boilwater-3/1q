/**
 * @file ImmFilter.h
 * @brief 定义交互多模型（IMM）滤波器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_IMM_FILTER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_IMM_FILTER_H_

#include <vector>

#include <Eigen/Core>

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief IMM 模型分支状态。
 */
struct ImmModelState {
  ImmModelState() = default;
  ImmModelState(const GaussianTrackState &s, float w) : state(s), weight(w) {}
/**
 * @brief 当前模型的高斯状态估计。
 */
  GaussianTrackState state;
/**
 * @brief 当前模型权重（概率）。
 */
  float weight{0.0f};
};
/**
 * @brief IMM 滤波器配置。
 */
struct ImmConfig {
/**
 * @brief 模型转移概率矩阵（N×N）。
 * @details pi(i,j) = P(模型j在k时刻 | 模型i在k-1时刻)。
 *          每行之和应为 1.0。
 */
  Eigen::MatrixXf transition_probability;
/**
 * @brief 各模型初始权重。
 */
  Eigen::VectorXf initial_weights;
};
/**
 * @brief 交互多模型（IMM）滤波器。
 * @details 实现 Bar-Shalom 的标准 IMM 算法，包含四个步骤：
 *          1. 交互/混合（Interaction/Mixing）
 *          2. 模型条件预测（Model-Conditioned Prediction）
 *          3. 模型条件更新（Model-Conditioned Update）
 *          4. 组合（Combination）
 *
 *          参考文献：
 *          - Bar-Shalom, Li, Kirubarajan, "Estimation with Applications
 *            to Tracking and Navigation", 2001, Chapter 11.6.
 *
 *          本实现假设每个模型拥有独立的 IKalmanPredictor 和 IKalmanUpdater，
 *          通过构造时注入。模型数量 N 由 predictors/updaters 向量长度决定。
 */
class ImmFilter {
 public:
/**
 * @brief 构造函数。
 * @param config IMM 配置（转移概率矩阵、初始权重）。
 * @param predictors 各模型的预测器（非拥有指针）。
 * @param updaters 各模型的更新器（非拥有指针）。
 */
  ImmFilter(ImmConfig config,
            std::vector<const IKalmanPredictor *> predictors,
            std::vector<const IKalmanUpdater *> updaters);
/**
 * @brief 执行完整的 IMM 循环：混合 → 预测 → 更新 → 组合。
 * @param measurement 量测向量。
 * @param dt 时间步长。
 */
  void Process(const MeasurementVector &measurement, float dt);
/**
 * @brief 仅执行预测步骤（无量测时）。
 * @param dt 时间步长。
 */
  void Predict(float dt);
/**
 * @brief 获取组合后的状态估计。
 * @return 组合高斯状态。
 */
  GaussianTrackState GetCombinedState() const;
/**
 * @brief 获取各模型当前权重。
 * @return 模型权重向量。
 */
  Eigen::VectorXf GetModelWeights() const;
/**
 * @brief 获取各模型分支状态。
 * @return 模型状态列表。
 */
  const std::vector<ImmModelState> &GetModelStates() const;
/**
 * @brief 设置模型状态（用于初始化）。
 * @param states 各模型初始状态。
 */
  void SetModelStates(const std::vector<ImmModelState> &states);

 private:
/**
 * @brief 步骤 1：交互/混合。
 * @details 计算混合概率 μ_{i|j}，生成各模型的混合状态。
 */
  void MixStates();
/**
 * @brief 步骤 2：模型条件预测。
 * @param dt 时间步长。
 */
  void PredictModels(float dt);
/**
 * @brief 步骤 3：模型条件更新。
 * @param measurement 量测向量。
 */
  void UpdateModels(const MeasurementVector &measurement);
/**
 * @brief 步骤 4：组合。
 */
  void CombineEstimates();
/**
 * @brief 计算高斯似然 N(y; 0, S)。
 * @param innovation 新息向量。
 * @param S 新息协方差。
 * @return 似然值（Likelihood）。
 */
  static float GaussianLikelihood(const MeasurementVector &innovation,
                                  const MeasurementCovariance &S);
/**
 * @brief 模型数量。
 */
  int num_models_{0};
/**
 * @brief IMM 配置。
 */
  ImmConfig config_;
/**
 * @brief 各模型的预测器。
 */
  std::vector<const IKalmanPredictor *> predictors_;
/**
 * @brief 各模型的更新器。
 */
  std::vector<const IKalmanUpdater *> updaters_;
/**
 * @brief 各模型分支状态。
 */
  std::vector<ImmModelState> model_states_;
/**
 * @brief 混合后的各模型状态（临时缓冲）。
 */
  std::vector<GaussianTrackState> mixed_states_;
/**
 * @brief 预测后的各模型状态（临时缓冲）。
 */
  std::vector<GaussianTrackState> predicted_states_;
/**
 * @brief 各模型更新结果缓冲，避免高频重复分配。
 */
  std::vector<KalmanUpdateResult> update_results_;
/**
 * @brief 各模型似然缓冲。
 */
  Eigen::VectorXf likelihoods_;
/**
 * @brief 各模型归一化常数缓冲。
 */
  Eigen::VectorXf c_bar_;
/**
 * @brief 各模型新权重缓冲。
 */
  Eigen::VectorXf new_weights_;
/**
 * @brief 组合后的最终状态。
 */
  GaussianTrackState combined_state_;
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_IMM_FILTER_H_
