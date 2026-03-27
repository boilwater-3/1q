#include "airborne_radar/signal/tracking/ImmFilter.h"

#include <Eigen/Cholesky>
#include <cmath>

namespace airborne_radar {
namespace signal {
namespace tracking {

ImmFilter::ImmFilter(ImmConfig config, std::vector<const IKalmanPredictor*> predictors,
                     std::vector<const IKalmanUpdater*> updaters)
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
    model_states_[static_cast<std::size_t>(j)].weight = config_.initial_weights(j);
  }
}

void ImmFilter::Process(const MeasurementVector& measurement, float dt) {
  MixStates();
  PredictModels(dt);
  UpdateModels(measurement);
  CombineEstimates();
}

void ImmFilter::Predict(float dt) {
  MixStates();
  PredictModels(dt);

  // 通过转移矩阵扩散模型权重（标准 IMM coast 路径）
  const int N = num_models_;
  for (int j = 0; j < N; ++j) {
    float c_bar = 0.0f;
    for (int i = 0; i < N; ++i) {
      c_bar +=
          config_.transition_probability(i, j) * model_states_[static_cast<std::size_t>(i)].weight;
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

void ImmFilter::MixStates() {
  const int N = num_models_;

  for (int j = 0; j < N; ++j) {
    const auto ju = static_cast<std::size_t>(j);

    // 计算归一化常数 c̄_j，同时写入成员供 UpdateModels 复用
    c_bar_(j) = 0.0f;
    for (int i = 0; i < N; ++i) {
      c_bar_(j) +=
          config_.transition_probability(i, j) * model_states_[static_cast<std::size_t>(i)].weight;
    }
    if (c_bar_(j) < 1e-30f) {
      c_bar_(j) = 1e-30f;  // 防止除零
    }
    const float c_bar = c_bar_(j);

    // 混合均值
    StateVector mixed_mean = StateVector::Zero();
    for (int i = 0; i < N; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      const float mu_ij = config_.transition_probability(i, j) * model_states_[iu].weight / c_bar;
      mixed_mean += mu_ij * model_states_[iu].state.mean;
    }

    // 混合协方差
    StateCovariance mixed_cov = StateCovariance::Zero();
    for (int i = 0; i < N; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      const float mu_ij = config_.transition_probability(i, j) * model_states_[iu].weight / c_bar;
      const StateVector diff = model_states_[iu].state.mean - mixed_mean;
      mixed_cov += mu_ij * (model_states_[iu].state.covariance + diff * diff.transpose());
    }

    mixed_states_[ju].mean = mixed_mean;
    mixed_states_[ju].covariance = mixed_cov;
  }
}

void ImmFilter::PredictModels(float dt) {
  for (int j = 0; j < num_models_; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    predicted_states_[ju] = predictors_[ju]->Predict(mixed_states_[ju], dt);
  }
}

void ImmFilter::UpdateModels(const MeasurementVector& measurement) {
  const int N = num_models_;

  for (int j = 0; j < N; ++j) {
    const auto ju = static_cast<std::size_t>(j);
    update_results_[ju] = updaters_[ju]->Update(predicted_states_[ju], measurement);
    model_states_[ju].state = update_results_[ju].posterior;

    // 模型似然
    likelihoods_(j) = GaussianLikelihood(update_results_[ju].innovation,
                                         update_results_[ju].innovation_covariance);
  }

  // c_bar_ 已由 MixStates() 计算并写入，此处直接复用。
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
    combined_cov +=
        model_states_[ju].weight * (model_states_[ju].state.covariance + diff * diff.transpose());
  }

  combined_state_.mean = combined_mean;
  combined_state_.covariance = combined_cov;
}

float ImmFilter::GaussianLikelihood(const MeasurementVector& innovation,
                                    const MeasurementCovariance& S) {
  const Eigen::LLT<MeasurementCovariance> llt(S);

  // log|S| = 2 * Σ log(L_ii)：提升为 double 精度，并对对角元素加 epsilon 防 log(0)
  constexpr double kLogDetEps = 1.0e-18;
  const Eigen::Matrix<double, kMeasurementDim, kMeasurementDim> L =
      llt.matrixL().toDenseMatrix().cast<double>();
  double log_det_d = 0.0;
  for (int k = 0; k < kMeasurementDim; ++k) {
    log_det_d += std::log(std::max(L(k, k), kLogDetEps));
  }
  log_det_d *= 2.0;

  // yᵀ S⁻¹ y
  const MeasurementVector s_inv_y = llt.solve(innovation);
  const float mahal_sq = innovation.dot(s_inv_y);

  // log-likelihood（用 double 中间值避免精度丢失）
  constexpr double kLogTwoPi = 1.8378770664093455;  // log(2π)
  const double log_likelihood =
      -0.5 * (static_cast<double>(kMeasurementDim) * kLogTwoPi + log_det_d +
              static_cast<double>(mahal_sq));

  return static_cast<float>(std::exp(log_likelihood));
}

GaussianTrackState ImmFilter::GetCombinedState() const { return combined_state_; }

Eigen::VectorXf ImmFilter::GetModelWeights() const {
  Eigen::VectorXf weights(num_models_);
  for (int j = 0; j < num_models_; ++j) {
    weights(j) = model_states_[static_cast<std::size_t>(j)].weight;
  }
  return weights;
}

const std::vector<ImmModelState>& ImmFilter::GetModelStates() const { return model_states_; }

void ImmFilter::SetModelStates(const std::vector<ImmModelState>& states) { model_states_ = states; }

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
