/**
 * @file RirImmFilter.cpp
 * @brief RIR IMM 滤波器包装实现（阶段 2-T N4）。
 */

#include "remote_identification_radar/tracking/RirImmFilter.h"

#include <cmath>

namespace remote_identification_radar {
namespace tracking {

namespace {

/** @brief 对数等距过程噪声差异系数（口径同 AR BuildDefaultImmNoiseDiffCoeffs）。 */
std::vector<float> BuildDefaultNoiseDiffCoeffs(std::uint32_t model_count_hint) {
  const std::size_t model_count =
      static_cast<std::size_t>(model_count_hint < 2U ? 2U : model_count_hint);
  std::vector<float> coeffs;
  coeffs.reserve(model_count);
  for (std::size_t i = 0U; i < model_count; ++i) {
    coeffs.push_back(std::pow(10.0f, static_cast<float>(i) /
                                         static_cast<float>(model_count - 1U)));
  }
  return coeffs;
}

}  // namespace

RirImmFilter::RirImmFilter(const Config& config) {
  std::vector<float> coeffs = config.model_noise_diff_coeffs;
  if (coeffs.empty()) {
    coeffs = BuildDefaultNoiseDiffCoeffs(2U);
  }
  if (coeffs.size() < 2U) {
    // 单模型 IMM 无混合意义：保持惰性（IsValid == false），调用方回退 CV KF。
    model_count_ = 0U;
    return;
  }

  const std::size_t model_count = coeffs.size();
  predictors_.resize(model_count);
  updaters_.resize(model_count);
  std::vector<ImmFilterT::Predictor*> predictor_ptrs;
  std::vector<ImmFilterT::Updater*> updater_ptrs;
  predictor_ptrs.reserve(model_count);
  updater_ptrs.reserve(model_count);
  for (std::size_t i = 0U; i < model_count; ++i) {
    ::oneq::common::estimation::KalmanPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = coeffs[i];
    predictors_[i].UpdateConfig(predictor_config);
    // 更新器走动态 R 路径，缺省量测噪声不参与 IMM 数值。
    predictor_ptrs.push_back(&predictors_[i]);
    updater_ptrs.push_back(&updaters_[i]);
  }

  const Eigen::Index n = static_cast<Eigen::Index>(model_count);
  const float diagonal =
      config.transition_diagonal_probability >= 0.5f && config.transition_diagonal_probability < 1.0f
          ? config.transition_diagonal_probability
          : 0.95f;
  ::oneq::common::estimation::ImmConfig imm_config;
  imm_config.transition_probability =
      Eigen::MatrixXf::Constant(n, n, (1.0f - diagonal) / static_cast<float>(model_count - 1U));
  imm_config.transition_probability.diagonal().setConstant(diagonal);
  imm_config.initial_weights = Eigen::VectorXf::Constant(n, 1.0f / static_cast<float>(model_count));

  filter_.reset(new ImmFilterT(imm_config, predictor_ptrs, updater_ptrs));
  model_count_ = model_count;
}

void RirImmFilter::Initialize(const RirGaussianState& initial_state) {
  if (filter_ == nullptr) {
    return;
  }
  const Eigen::VectorXf weights = filter_->GetModelWeights();
  std::vector<ImmFilterT::ModelState> states;
  states.reserve(model_count_);
  for (std::size_t i = 0U; i < model_count_; ++i) {
    states.push_back(ImmFilterT::ModelState(
        initial_state, weights(static_cast<Eigen::Index>(i))));
  }
  filter_->SetModelStates(states);
}

void RirImmFilter::Process(const Eigen::Vector3f& position, float dt_sec,
                           const RirMeasurementCovariance& dynamic_R) {
  if (filter_ == nullptr) {
    return;
  }
  filter_->Process(position, dt_sec, dynamic_R);
}

void RirImmFilter::Predict(float dt_sec) {
  if (filter_ == nullptr) {
    return;
  }
  filter_->Predict(dt_sec);
}

RirGaussianState RirImmFilter::GetCombinedState() const {
  if (filter_ == nullptr) {
    return RirGaussianState();
  }
  return filter_->GetCombinedState();
}

Eigen::VectorXf RirImmFilter::GetModelWeights() const {
  if (filter_ == nullptr) {
    return Eigen::VectorXf();
  }
  return filter_->GetModelWeights();
}

}  // namespace tracking
}  // namespace remote_identification_radar
