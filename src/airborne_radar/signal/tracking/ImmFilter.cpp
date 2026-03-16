// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现交互多模型（IMM）滤波器。

#include "airborne_radar/signal/tracking/ImmFilter.h"

#include <cmath>

#include <Eigen/Cholesky>

namespace airborne_radar {
namespace signal {
namespace tracking {

ImmFilter::ImmFilter(ImmConfig config,
                     std::vector<const IKalmanPredictor *> predictors,
                     std::vector<const IKalmanUpdater *> updaters)
    : num_models_(static_cast<int>(predictors.size())),
      config_(std::move(config)),
      predictors_(std::move(predictors)),
      updaters_(std::move(updaters)),
      model_states_(static_cast<std::size_t>(num_models_)),
      mixed_states_(static_cast<std::size_t>(num_models_)),
      predicted_states_(static_cast<std::size_t>(num_models_)),
      update_results_(static_cast<std::size_t>(num_models_)),
      likelihoods_(Eigen::VectorXf::Zero(num_models_)),
      c_bar_(Eigen::VectorXf::Zero(num_models_)),
      new_weights_(Eigen::VectorXf::Zero(num_models_)) {
  // 初始化模型权重
  for (int j = 0; j < num_models_; ++j) {
    model_states_[static_cast<std::size_t>(j)].weight =
        config_.initial_weights(j);
  }
}

void ImmFilter::Process(const MeasurementVector &measurement, float dt) {
  MixStates();
  PredictModels(dt);
  UpdateModels(measurement);
  CombineEstimates();
}

void ImmFilter::Predict(float dt) {
  MixStates();
  PredictModels(dt);

  for (int j = 0; j < num_models_; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    model_states_[ju].state = predicted_states_[ju];
  }

  CombineEstimates();
}

/// @brief 步骤 1：交互/混合。
/// @details Bar-Shalom 11.6.6-2：
///          c̄_j = Σ_i π_{ij} · μ_{i,k-1}     （归一化常数）
///          μ_{i|j} = π_{ij} · μ_{i,k-1} / c̄_j （混合概率）
///          x̂⁰_{j} = Σ_i μ_{i|j} · x̂_i        （混合均值）
///          P⁰_{j} = Σ_i μ_{i|j} · [P_i + (x̂_i - x̂⁰_j)(x̂_i - x̂⁰_j)ᵀ] （混合协方差）
void ImmFilter::MixStates() {
  const int N = num_models_;

  for (int j = 0; j < N; ++j) {
    const auto ju = static_cast<std::size_t>(j);

    // 计算归一化常数 c̄_j
    float c_bar = 0.0f;
    for (int i = 0; i < N; ++i) {
      c_bar += config_.transition_probability(i, j) *
               model_states_[static_cast<std::size_t>(i)].weight;
    }
    if (c_bar < 1e-30f) {
      c_bar = 1e-30f;  // 防止除零
    }

    // 混合均值
    StateVector mixed_mean = StateVector::Zero();
    for (int i = 0; i < N; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      const float mu_ij = config_.transition_probability(i, j) *
                          model_states_[iu].weight / c_bar;
      mixed_mean += mu_ij * model_states_[iu].state.mean;
    }

    // 混合协方差
    StateCovariance mixed_cov = StateCovariance::Zero();
    for (int i = 0; i < N; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      const float mu_ij = config_.transition_probability(i, j) *
                          model_states_[iu].weight / c_bar;
      const StateVector diff = model_states_[iu].state.mean - mixed_mean;
      mixed_cov += mu_ij * (model_states_[iu].state.covariance +
                            diff * diff.transpose());
    }

    mixed_states_[ju].mean = mixed_mean;
    mixed_states_[ju].covariance = mixed_cov;
  }
}

/// @brief 步骤 2：模型条件预测。
void ImmFilter::PredictModels(float dt) {
  for (int j = 0; j < num_models_; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    predicted_states_[ju] = predictors_[ju]->Predict(mixed_states_[ju], dt);
  }
}

/// @brief 步骤 3：模型条件更新 + 权重更新。
/// @details Bar-Shalom 11.6.6-8：
///          Λ_j = N(y_j; 0, S_j)              （模型似然）
///          μ_{j,k} = c̄_j · Λ_j / Σ(c̄_l · Λ_l) （更新权重）
void ImmFilter::UpdateModels(const MeasurementVector &measurement) {
  const int N = num_models_;

  for (int j = 0; j < N; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    update_results_[ju] = updaters_[ju]->Update(predicted_states_[ju], measurement);
    model_states_[ju].state = update_results_[ju].posterior;

    // 模型似然
    likelihoods_(j) = GaussianLikelihood(update_results_[ju].innovation,
                                         update_results_[ju].innovation_covariance);
  }

  // 计算归一化常数 c̄_j
  for (int j = 0; j < N; ++j) {
    c_bar_(j) = 0.0f;
    for (int i = 0; i < N; ++i) {
      c_bar_(j) += config_.transition_probability(i, j) *
                   model_states_[static_cast<std::size_t>(i)].weight;
    }
  }

  // 更新模型权重
  float total = 0.0f;
  for (int j = 0; j < N; ++j) {
    new_weights_(j) = c_bar_(j) * likelihoods_(j);
    total += new_weights_(j);
  }
  if (total < 1e-30f) {
    total = 1e-30f;
  }

  for (int j = 0; j < N; ++j) {
    model_states_[static_cast<std::size_t>(j)].weight = new_weights_(j) / total;
  }
}

/// @brief 步骤 4：组合各模型估计。
/// @details Bar-Shalom 11.6.6-9：
///          x̂ = Σ_j μ_j · x̂_j
///          P = Σ_j μ_j · [P_j + (x̂_j - x̂)(x̂_j - x̂)ᵀ]
void ImmFilter::CombineEstimates() {
  StateVector combined_mean = StateVector::Zero();
  for (int j = 0; j < num_models_; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    combined_mean += model_states_[ju].weight * model_states_[ju].state.mean;
  }

  StateCovariance combined_cov = StateCovariance::Zero();
  for (int j = 0; j < num_models_; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    const StateVector diff = model_states_[ju].state.mean - combined_mean;
    combined_cov += model_states_[ju].weight *
                    (model_states_[ju].state.covariance +
                     diff * diff.transpose());
  }

  combined_state_.mean = combined_mean;
  combined_state_.covariance = combined_cov;
}

/// @brief 计算多元高斯似然 N(y; 0, S)。
/// @details L = (2π)^{-d/2} |S|^{-1/2} exp(-½ yᵀ S⁻¹ y)
float ImmFilter::GaussianLikelihood(const MeasurementVector &innovation,
                                    const MeasurementCovariance &S) {
  const Eigen::LLT<MeasurementCovariance> llt(S);

  // log|S| = 2 * Σ log(L_ii)
  const float log_det = 2.0f * llt.matrixL().toDenseMatrix()
                            .diagonal().array().log().sum();

  // yᵀ S⁻¹ y
  const MeasurementVector s_inv_y = llt.solve(innovation);
  const float mahal_sq = innovation.dot(s_inv_y);

  // log-likelihood
  constexpr float kLogTwoPi = 1.8378770664093455f;  // log(2π)
  const float log_likelihood =
      -0.5f * (static_cast<float>(kMeasurementDim) * kLogTwoPi +
               log_det + mahal_sq);

  return std::exp(log_likelihood);
}

GaussianTrackState ImmFilter::GetCombinedState() const {
  return combined_state_;
}

Eigen::VectorXf ImmFilter::GetModelWeights() const {
  Eigen::VectorXf weights(num_models_);
  for (int j = 0; j < num_models_; ++j) {
    weights(j) = model_states_[static_cast<std::size_t>(j)].weight;
  }
  return weights;
}

const std::vector<ImmModelState> &ImmFilter::GetModelStates() const {
  return model_states_;
}

void ImmFilter::SetModelStates(const std::vector<ImmModelState> &states) {
  model_states_ = states;
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
