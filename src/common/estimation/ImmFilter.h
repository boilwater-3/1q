/**
 * @file ImmFilter.h
 * @brief 定义交互多模型（IMM）滤波器（维度模板化）。
 *
 * 模型数 N 由构造时注入的 predictors/updaters 向量长度决定（动态）；状态/量测维度由模板参数
 * 表达（编译期固定）。
 */

#ifndef COMMON_ESTIMATION_IMM_FILTER_H_
#define COMMON_ESTIMATION_IMM_FILTER_H_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/IKalmanUpdater.h"

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief IMM 模型分支状态。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
struct ImmModelState {
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;

  ImmModelState() = default;
  ImmModelState(const GaussianStateT& s, float w) : state(s), weight(w) {}
  GaussianStateT state; /**< 当前模型的高斯状态估计。 */
  float weight{0.0f};   /**< 当前模型权重（概率）。 */
};

/**
 * @brief IMM 滤波器配置。
 * @details 模型数 N 动态，因此转移概率矩阵与初始权重用动态大小 Eigen 类型。
 */
struct ImmConfig {
  Eigen::MatrixXf transition_probability; /**< 模型转移概率矩阵（N×N）。pi(i,j) = P(模型j在k时刻 |
                                             模型i在k-1时刻)，每行之和应为 1.0。 */
  Eigen::VectorXf initial_weights;        /**< 各模型初始权重。 */
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
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class ImmFilter {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using Predictor = IKalmanPredictor<kStateDim, kMeasurementDim>;
  using Updater = IKalmanUpdater<kStateDim, kMeasurementDim>;
  using ModelState = ImmModelState<kStateDim, kMeasurementDim>;
  using StateVector = typename GaussianStateT::StateVector;
  using StateCovariance = typename GaussianStateT::StateCovariance;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using Result = KalmanUpdateResult<kStateDim, kMeasurementDim>;

  /**
   * @brief 构造函数。
   * @param[in] config IMM 配置（转移概率矩阵、初始权重）。
   * @param[in] predictors 各模型的预测器（非拥有指针）。
   * @param[in] updaters 各模型的更新器（非拥有指针）。
   */
  ImmFilter(ImmConfig config, std::vector<Predictor*> predictors, std::vector<Updater*> updaters)
      : num_models_(static_cast<int>(predictors.size())),
        config_(std::move(config)),
        predictors_(std::move(predictors)),
        updaters_(std::move(updaters)),
        model_states_(static_cast<std::size_t>(num_models_)),
        mixed_states_(static_cast<std::size_t>(num_models_)),
        predicted_states_(static_cast<std::size_t>(num_models_)),
        update_results_(static_cast<std::size_t>(num_models_)),
        log_likelihoods_(Eigen::VectorXf::Zero(num_models_)),
        c_bar_(Eigen::VectorXf::Zero(num_models_)),
        new_weights_(Eigen::VectorXf::Zero(num_models_)) {
    for (int j = 0; j < num_models_; ++j) {
      model_states_[static_cast<std::size_t>(j)].weight = config_.initial_weights(j);
    }
  }

  /**
   * @brief 执行完整的 IMM 循环：混合 → 预测 → 更新 → 组合。
   * @param[in] measurement 量测向量。
   * @param[in] dt 时间步长。
   */
  void Process(const MeasurementVector& measurement, float dt) {
    MixStates();
    PredictModels(dt);
    UpdateModels(measurement);
    CombineEstimates();
  }
  /**
   * @brief 仅执行预测步骤（无量测时）。
   * @param[in] dt 时间步长。
   */
  void Predict(float dt) {
    MixStates();
    PredictModels(dt);

    const int N = num_models_;
    for (int j = 0; j < N; ++j) {
      float c_bar = 0.0f;
      for (int i = 0; i < N; ++i) {
        c_bar += config_.transition_probability(i, j) *
                 model_states_[static_cast<std::size_t>(i)].weight;
      }
      c_bar_(j) = c_bar < 1e-30f ? 1e-30f : c_bar;
    }
    float total = 0.0f;
    for (int j = 0; j < N; ++j) {
      total += c_bar_(j);
    }
    if (total < 1e-30f) {
      total = 1e-30f;
    }
    for (int j = 0; j < N; ++j) {
      model_states_[static_cast<std::size_t>(j)].weight = c_bar_(j) / total;
    }

    for (int j = 0; j < num_models_; ++j) {
      const auto ju = static_cast<std::size_t>(j);
      model_states_[ju].state = predicted_states_[ju];
    }

    CombineEstimates();
  }
  /** @return 组合高斯状态。 */
  GaussianStateT GetCombinedState() const { return combined_state_; }
  /** @return 模型权重向量。 */
  Eigen::VectorXf GetModelWeights() const {
    Eigen::VectorXf weights(num_models_);
    for (int j = 0; j < num_models_; ++j) {
      weights(j) = model_states_[static_cast<std::size_t>(j)].weight;
    }
    return weights;
  }
  /** @return 模型状态列表。 */
  const std::vector<ModelState>& GetModelStates() const { return model_states_; }
  /**
   * @brief 设置模型状态（用于初始化）。
   * @param[in] states 各模型初始状态。
   */
  void SetModelStates(const std::vector<ModelState>& states) { model_states_ = states; }
  /**
   * @brief 在线同步 IMM 运行参数与模型滤波器指针。
   * @return 同步成功返回 true；模型维度不一致时返回 false。
   */
  bool UpdateRuntimeTuning(const ImmConfig& config, const std::vector<Predictor*>& predictors,
                           const std::vector<Updater*>& updaters) {
    if (predictors.empty() || predictors.size() != updaters.size()) {
      return false;
    }
    if (config.transition_probability.rows() != static_cast<Eigen::Index>(predictors.size()) ||
        config.transition_probability.cols() != static_cast<Eigen::Index>(predictors.size()) ||
        config.initial_weights.size() != static_cast<Eigen::Index>(predictors.size())) {
      return false;
    }

    num_models_ = static_cast<int>(predictors.size());
    config_ = config;
    predictors_ = predictors;
    updaters_ = updaters;
    if (model_states_.size() != predictors.size()) {
      model_states_.assign(predictors.size(), ModelState());
    }
    mixed_states_.assign(predictors.size(), GaussianStateT());
    predicted_states_.assign(predictors.size(), GaussianStateT());
    update_results_.assign(predictors.size(), Result());
    log_likelihoods_ = Eigen::VectorXf::Zero(num_models_);
    c_bar_ = Eigen::VectorXf::Zero(num_models_);
    new_weights_ = Eigen::VectorXf::Zero(num_models_);
    return true;
  }

 private:
  /** @brief 步骤 1：交互/混合，计算混合概率 μ_{i|j}，生成各模型的混合状态。 */
  void MixStates() {
    const int N = num_models_;

    for (int j = 0; j < N; ++j) {
      const auto ju = static_cast<std::size_t>(j);

      c_bar_(j) = 0.0f;
      for (int i = 0; i < N; ++i) {
        c_bar_(j) += config_.transition_probability(i, j) *
                     model_states_[static_cast<std::size_t>(i)].weight;
      }
      if (c_bar_(j) < 1e-30f) {
        c_bar_(j) = 1e-30f;  // 防止除零
      }
      const float c_bar = c_bar_(j);

      StateVector mixed_mean = StateVector::Zero();
      for (int i = 0; i < N; ++i) {
        const auto iu = static_cast<std::size_t>(i);
        const float mu_ij =
            config_.transition_probability(i, j) * model_states_[iu].weight / c_bar;
        mixed_mean += mu_ij * model_states_[iu].state.mean;
      }

      StateCovariance mixed_cov = StateCovariance::Zero();
      for (int i = 0; i < N; ++i) {
        const auto iu = static_cast<std::size_t>(i);
        const float mu_ij =
            config_.transition_probability(i, j) * model_states_[iu].weight / c_bar;
        const StateVector diff = model_states_[iu].state.mean - mixed_mean;
        mixed_cov += mu_ij * (model_states_[iu].state.covariance + diff * diff.transpose());
      }

      mixed_states_[ju].mean = mixed_mean;
      mixed_states_[ju].covariance = mixed_cov;
    }
  }
  /** @brief 步骤 2：模型条件预测。 */
  void PredictModels(float dt) {
    for (int j = 0; j < num_models_; ++j) {
      const auto ju = static_cast<std::size_t>(j);
      predicted_states_[ju] = predictors_[ju]->Predict(mixed_states_[ju], dt);
    }
  }
  /** @brief 步骤 3：模型条件更新。 */
  void UpdateModels(const MeasurementVector& measurement) {
    const int N = num_models_;
    constexpr float kWeightFloor = 1e-30f;

    for (int j = 0; j < N; ++j) {
      const auto ju = static_cast<std::size_t>(j);
      update_results_[ju] = updaters_[ju]->Update(predicted_states_[ju], measurement);
      model_states_[ju].state = update_results_[ju].posterior;

      log_likelihoods_(j) = static_cast<float>(GaussianLogLikelihood(
          update_results_[ju].innovation, update_results_[ju].innovation_covariance));
    }

    // 1) log-sum-exp 归一化，避免 exp(log_likelihood) 下溢导致权重塌陷。
    double max_log_weight = -std::numeric_limits<double>::infinity();
    for (int j = 0; j < N; ++j) {
      const double c_bar = std::max(static_cast<double>(c_bar_(j)), static_cast<double>(kWeightFloor));
      const double log_weight = std::log(c_bar) + static_cast<double>(log_likelihoods_(j));
      new_weights_(j) = static_cast<float>(log_weight);
      max_log_weight = std::max(max_log_weight, log_weight);
    }

    double total = 0.0;
    if (std::isfinite(max_log_weight)) {
      for (int j = 0; j < N; ++j) {
        const double shifted = static_cast<double>(new_weights_(j)) - max_log_weight;
        const float scaled = shifted < -80.0 ? 0.0f : static_cast<float>(std::exp(shifted));
        new_weights_(j) = scaled;
        total += static_cast<double>(scaled);
      }
    }

    if (std::isfinite(total) && total > static_cast<double>(kWeightFloor)) {
      for (int j = 0; j < N; ++j) {
        model_states_[static_cast<std::size_t>(j)].weight =
            static_cast<float>(static_cast<double>(new_weights_(j)) / total);
      }
      return;
    }

    // 2) log-sum-exp 异常时回退到 c_bar_ 归一化。
    total = 0.0;
    for (int j = 0; j < N; ++j) {
      const float c_bar = std::max(c_bar_(j), 0.0f);
      new_weights_(j) = c_bar;
      total += static_cast<double>(c_bar);
    }

    if (std::isfinite(total) && total > static_cast<double>(kWeightFloor)) {
      for (int j = 0; j < N; ++j) {
        model_states_[static_cast<std::size_t>(j)].weight =
            static_cast<float>(static_cast<double>(new_weights_(j)) / total);
      }
      return;
    }

    // 3) 仍异常时使用均匀权重兜底。
    const float uniform_weight = (N > 0) ? (1.0f / static_cast<float>(N)) : 0.0f;
    for (int j = 0; j < N; ++j) {
      model_states_[static_cast<std::size_t>(j)].weight = uniform_weight;
    }
  }
  /** @brief 步骤 4：组合。 */
  void CombineEstimates() {
    StateVector combined_mean = StateVector::Zero();
    for (int j = 0; j < num_models_; ++j) {
      const auto ju = static_cast<std::size_t>(j);
      combined_mean += model_states_[ju].weight * model_states_[ju].state.mean;
    }

    StateCovariance combined_cov = StateCovariance::Zero();
    for (int j = 0; j < num_models_; ++j) {
      const auto ju = static_cast<std::size_t>(j);
      const StateVector diff = model_states_[ju].state.mean - combined_mean;
      combined_cov +=
          model_states_[ju].weight * (model_states_[ju].state.covariance + diff * diff.transpose());
    }

    combined_state_.mean = combined_mean;
    combined_state_.covariance = combined_cov;
  }
  /**
   * @brief 计算高斯似然的对数值 log N(y; 0, S)。
   * @param[in] innovation 新息向量。
   * @param[in] S 新息协方差。
   * @return 对数似然值。
   */
  static double GaussianLogLikelihood(const MeasurementVector& innovation,
                                      const MeasurementCovariance& S) {
    const Eigen::LLT<MeasurementCovariance> llt(S);
    if (llt.info() != Eigen::Success) {
      return -std::numeric_limits<double>::infinity();
    }

    // log|S| = 2 * Σ log(L_ii)：提升为 double 精度，并对对角元素加 epsilon 防 log(0)
    constexpr double kLogDetEps = 1.0e-18;
    const Eigen::Matrix<double, kMeasurementDim, kMeasurementDim> L =
        llt.matrixL().toDenseMatrix().template cast<double>();
    double log_det_d = 0.0;
    for (int k = 0; k < kMeasurementDim; ++k) {
      log_det_d += std::log(std::max(L(k, k), kLogDetEps));
    }
    log_det_d *= 2.0;

    // yᵀ S⁻¹ y
    const MeasurementVector s_inv_y = llt.solve(innovation);
    const float mahal_sq = innovation.dot(s_inv_y);
    if (!std::isfinite(mahal_sq)) {
      return -std::numeric_limits<double>::infinity();
    }

    // log-likelihood（用 double 中间值避免精度丢失）
    constexpr double kLogTwoPi = 1.8378770664093455;  // log(2π)
    return -0.5 * (static_cast<double>(kMeasurementDim) * kLogTwoPi + log_det_d +
                   static_cast<double>(mahal_sq));
  }

  int num_models_{0};                           /**< 模型数量。 */
  ImmConfig config_;                            /**< IMM 配置。 */
  std::vector<Predictor*> predictors_;          /**< 各模型的预测器。 */
  std::vector<Updater*> updaters_;              /**< 各模型的更新器。 */
  std::vector<ModelState> model_states_;        /**< 各模型分支状态。 */
  std::vector<GaussianStateT> mixed_states_;    /**< 混合后的各模型状态（临时缓冲）。 */
  std::vector<GaussianStateT> predicted_states_;/**< 预测后的各模型状态（临时缓冲）。 */
  std::vector<Result> update_results_;          /**< 各模型更新结果缓冲，避免高频重复分配。 */
  Eigen::VectorXf log_likelihoods_;             /**< 各模型对数似然缓冲。 */
  Eigen::VectorXf c_bar_;                       /**< 各模型归一化常数缓冲。 */
  Eigen::VectorXf new_weights_;                 /**< 各模型新权重缓冲。 */
  GaussianStateT combined_state_;               /**< 组合后的最终状态。 */
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_IMM_FILTER_H_
