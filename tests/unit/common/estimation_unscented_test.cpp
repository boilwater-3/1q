/**
 * @file estimation_unscented_test.cpp
 * @brief 验证 common/estimation 无迹（Unscented）滤波原语（6/3 实例化 + 6/2 非标准维度）。
 *
 * 核心正确性门：线性模型下无迹与标准 KF 数值一致（sigma 点对线性函数精确传播）；
 * 其余覆盖动态 R、fail-safe、非线性量测收敛、IMM 多态混合接入。
 */

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <array>
#include <cmath>

#include "common/estimation/EkfFilter.h"
#include "common/estimation/GaussianState.h"
#include "common/estimation/ImmFilter.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/estimation/KalmanPredictor.h"
#include "common/estimation/KalmanUpdater.h"
#include "common/estimation/UnscentedPredictor.h"
#include "common/estimation/UnscentedTransform.h"
#include "common/estimation/UnscentedUpdater.h"

namespace oneq {
namespace common {
namespace estimation {
namespace {

constexpr int kTestStateDim = 6;
constexpr int kTestMeasDim = 3;
constexpr float kTolerance = 1e-4f;

using State = GaussianState<kTestStateDim, kTestMeasDim>;

/** @brief 构造初始先验：位置 (100,0,0)，各维方差 100，速度方差 25（与 kalman 测试同源）。 */
State MakePrior() {
  State prior;
  prior.mean << 100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  prior.covariance = State::StateCovariance::Zero();
  for (int i = 0; i < kTestStateDim; i += 2) {
    prior.covariance(i, i) = 100.0f;
    prior.covariance(i + 1, i + 1) = 25.0f;
  }
  return prior;
}

/** @brief 距离-方位非线性量测模型（6 维状态 / 2 维量测），供非线性路径与 6/2 实例化测试。 */
class RangeBearingMeasurementModel final : public IMeasurementModel<6, 2> {
 public:
  MeasurementVector Function(const StateVector& state) const override {
    MeasurementVector z;
    const double x = static_cast<double>(state(0));
    const double y = static_cast<double>(state(2));
    const double zz = static_cast<double>(state(4));
    z(0) = static_cast<float>(std::sqrt(x * x + y * y + zz * zz));
    z(1) = static_cast<float>(std::atan2(y, x));
    return z;
  }
  MeasurementMatrix Jacobian(const StateVector& state) const override {
    MeasurementMatrix H = MeasurementMatrix::Zero();
    const double x = static_cast<double>(state(0));
    const double y = static_cast<double>(state(2));
    const double zz = static_cast<double>(state(4));
    const double range = std::sqrt(x * x + y * y + zz * zz);
    const double horizontal_sq = x * x + y * y;
    if (range > 1.0e-6 && horizontal_sq > 1.0e-6) {
      H(0, 0) = static_cast<float>(x / range);
      H(0, 2) = static_cast<float>(y / range);
      H(0, 4) = static_cast<float>(zz / range);
      H(1, 0) = static_cast<float>(-y / horizontal_sq);
      H(1, 2) = static_cast<float>(x / horizontal_sq);
    }
    return H;
  }
};

TEST(UnscentedTransformTest, MeanWeightsSumToOneAndBetaTermOnCenterWeight) {
  Eigen::Matrix<float, 2, 1> mean;
  mean << 1.0f, 2.0f;
  Eigen::Matrix<float, 2, 2> covariance = Eigen::Matrix<float, 2, 2>::Identity();

  UnscentedPointSet<2> point_set;
  ASSERT_TRUE(UnscentedTransform<2>::GenerateSigmaPoints(mean, covariance,
                                                         UnscentedTransformConfig{}, &point_set));

  float mean_weight_sum = 0.0f;
  for (std::size_t i = 0U; i < static_cast<std::size_t>(UnscentedPointSet<2>::kPointCount); ++i) {
    mean_weight_sum += point_set.mean_weights[i];
  }
  EXPECT_NEAR(mean_weight_sum, 1.0f, kTolerance);
  /* 默认 alpha=1, beta=2：中心协方差权重 = 中心均值权重 + (1 - 1 + 2) = +2 项。 */
  EXPECT_NEAR(point_set.covariance_weights[0U], point_set.mean_weights[0U] + 2.0f, kTolerance);
  EXPECT_EQ(UnscentedPointSet<2>::kPointCount, 5);
}

TEST(UnscentedPredictorTest, LinearModelMatchesStandardKalman) {
  KalmanPredictorConfig std_config;
  std_config.noise_diff_coeff = 1.0f;
  KalmanPredictor<kTestStateDim, kTestMeasDim> std_predictor(std_config);

  LinearCvTransitionModel<kTestStateDim> cv_model;
  UnscentedPredictorConfig config;
  config.noise_diff_coeff = 1.0f;
  UnscentedPredictor<kTestStateDim, kTestMeasDim> unscented_predictor(&cv_model, config);

  const auto prior = MakePrior();
  const auto std_predicted = std_predictor.Predict(prior, 1.0f);
  const auto unscented_predicted = unscented_predictor.Predict(prior, 1.0f);
  EXPECT_TRUE(std_predicted.mean.isApprox(unscented_predicted.mean, kTolerance));
  EXPECT_TRUE(std_predicted.covariance.isApprox(unscented_predicted.covariance, kTolerance));
}

TEST(UnscentedPredictorTest, NullModelFailsSafeToPrior) {
  UnscentedPredictor<kTestStateDim, kTestMeasDim> predictor(nullptr);
  const auto prior = MakePrior();
  const auto predicted = predictor.Predict(prior, 1.0f);
  EXPECT_TRUE(predicted.mean.isApprox(prior.mean, 1.0e-6f));
  EXPECT_TRUE(predicted.covariance.isApprox(prior.covariance, 1.0e-6f));
}

TEST(UnscentedUpdaterTest, LinearModelMatchesStandardKalman) {
  KalmanPredictor<kTestStateDim, kTestMeasDim> predictor;
  const auto predicted = predictor.Predict(MakePrior(), 1.0f);
  State::MeasurementVector measurement(105.0f, 0.0f, 0.0f);

  KalmanUpdaterConfig std_config;
  std_config.measurement_noise_std = 10.0f;
  KalmanUpdater<kTestStateDim, kTestMeasDim> std_updater(std_config);
  LinearPositionMeasurementModel<kTestStateDim, kTestMeasDim> meas_model;
  UnscentedUpdaterConfig config;
  config.measurement_noise_std = 10.0f;
  UnscentedUpdater<kTestStateDim, kTestMeasDim> unscented_updater(&meas_model, config);

  const auto std_result = std_updater.Update(predicted, measurement);
  const auto unscented_result = unscented_updater.Update(predicted, measurement);
  EXPECT_TRUE(std_result.posterior.mean.isApprox(unscented_result.posterior.mean, kTolerance));
  EXPECT_TRUE(
      std_result.posterior.covariance.isApprox(unscented_result.posterior.covariance, kTolerance));
  EXPECT_NEAR(unscented_result.innovation(0), 5.0f, kTolerance);
  EXPECT_NEAR(unscented_result.innovation_covariance(0, 0), predicted.covariance(0, 0) + 100.0f,
              1.0f);
}

TEST(UnscentedUpdaterTest, DynamicRCovariancePathMatchesProvidedR) {
  KalmanPredictor<kTestStateDim, kTestMeasDim> predictor;
  LinearPositionMeasurementModel<kTestStateDim, kTestMeasDim> meas_model;
  UnscentedUpdater<kTestStateDim, kTestMeasDim> updater(&meas_model);
  const auto predicted = predictor.Predict(MakePrior(), 1.0f);
  State::MeasurementVector measurement(105.0f, 0.0f, 0.0f);
  State::MeasurementCovariance dynamic_R = State::MeasurementCovariance::Identity() * 100.0f;
  const auto result = updater.Update(predicted, measurement, dynamic_R);
  EXPECT_NEAR(result.innovation_covariance(0, 0), predicted.covariance(0, 0) + 100.0f, 1.0f);
}

TEST(UnscentedUpdaterTest, NullModelFailsSafeToPredicted) {
  UnscentedUpdater<kTestStateDim, kTestMeasDim> updater(nullptr);
  KalmanPredictor<kTestStateDim, kTestMeasDim> predictor;
  const auto predicted = predictor.Predict(MakePrior(), 1.0f);
  State::MeasurementVector measurement(105.0f, 0.0f, 0.0f);
  const auto result = updater.Update(predicted, measurement);
  EXPECT_TRUE(result.posterior.mean.isApprox(predicted.mean, 1.0e-6f));
  EXPECT_TRUE(result.posterior.covariance.isApprox(predicted.covariance, 1.0e-6f));
}

/* 无噪确定性场景：真值匀速运动，量测 = h(真值)；两滤波器均应收敛且有限（弱断言）。 */
TEST(UnscentedFilterTest, NonlinearMeasurementConvergesDeterministically) {
  using State6x2 = GaussianState<6, 2>;
  LinearCvTransitionModel<6> cv_model;
  RangeBearingMeasurementModel meas_model;

  State6x2 state;
  state.mean << 100000.0f, 0.0f, 0.0f, 800.0f, 0.0f, 0.0f;
  state.covariance = State6x2::StateCovariance::Zero();
  for (int i = 0; i < 6; i += 2) {
    state.covariance(i, i) = 5000.0f * 5000.0f;
    state.covariance(i + 1, i + 1) = 100.0f * 100.0f;
  }

  UnscentedPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = 1.0f;
  UnscentedPredictor<6, 2> predictor(&cv_model, predictor_config);
  UnscentedUpdaterConfig updater_config;
  updater_config.measurement_noise_std = 1.0f;
  UnscentedUpdater<6, 2> updater(&meas_model, updater_config);
  State6x2::MeasurementCovariance dynamic_R;
  dynamic_R.setZero();
  dynamic_R(0, 0) = 2000.0f * 2000.0f;  /* 距离噪声（m）。 */
  dynamic_R(1, 1) = 0.003f * 0.003f;    /* 方位噪声（rad）。 */

  /* 真值：匀速直线 */
  Eigen::Matrix<float, 6, 1> truth;
  truth << 100000.0f, 0.0f, 0.0f, 800.0f, 0.0f, 0.0f;

  const float initial_trace = state.covariance.trace();
  for (int step = 0; step < 20; ++step) {
    truth(0) += truth(1);
    truth(2) += truth(3);
    truth(4) += truth(5);
    const auto predicted = predictor.Predict(state, 1.0f);
    state = updater.Update(predicted, meas_model.Function(truth), dynamic_R).posterior;
  }

  EXPECT_TRUE(state.mean.allFinite());
  EXPECT_TRUE(state.covariance.allFinite());
  EXPECT_LT(state.covariance.trace(), initial_trace);
  const double position_error = std::sqrt(
      std::pow(static_cast<double>(state.mean(0) - truth(0)), 2.0) +
      std::pow(static_cast<double>(state.mean(2) - truth(2)), 2.0) +
      std::pow(static_cast<double>(state.mean(4) - truth(4)), 2.0));
  EXPECT_LT(position_error, 2000.0);
}

TEST(UnscentedNonStandardDimTest, SixByTwoInstantiatesAndPropagates) {
  using State6x2 = GaussianState<6, 2>;
  State6x2 s;
  s.mean << 1, 2, 3, 4, 5, 6;
  s.covariance = State6x2::StateCovariance::Identity();

  LinearCvTransitionModel<6> cv_model;
  UnscentedPredictor<6, 2> predictor(&cv_model);
  const auto predicted = predictor.Predict(s, 1.0f);
  EXPECT_FALSE(predicted.mean.isApprox(s.mean));
  EXPECT_GT(predicted.covariance(0, 0), s.covariance(0, 0));

  RangeBearingMeasurementModel meas_model;
  UnscentedUpdater<6, 2> updater(&meas_model);
  State6x2::MeasurementVector measurement;
  measurement << 10.0f, 0.5f;
  const auto result = updater.Update(predicted, measurement);
  EXPECT_TRUE(result.posterior.mean.allFinite());
  EXPECT_TRUE(result.posterior.covariance.allFinite());
}

TEST(UnscentedImmCompatTest, MixesIntoImmFilter) {
  using Filter = ImmFilter<kTestStateDim, kTestMeasDim>;
  LinearCvTransitionModel<kTestStateDim> cv_model;
  LinearPositionMeasurementModel<kTestStateDim, kTestMeasDim> meas_model;

  KalmanPredictorConfig linear_config;
  linear_config.noise_diff_coeff = 1.0f;
  KalmanPredictor<kTestStateDim, kTestMeasDim> linear_predictor(linear_config);
  KalmanUpdater<kTestStateDim, kTestMeasDim> linear_updater;
  UnscentedPredictorConfig unscented_predictor_config;
  unscented_predictor_config.noise_diff_coeff = 25.0f;
  UnscentedPredictor<kTestStateDim, kTestMeasDim> unscented_predictor(&cv_model,
                                                                      unscented_predictor_config);
  UnscentedUpdaterConfig unscented_updater_config;
  unscented_updater_config.measurement_noise_std = 10.0f;
  UnscentedUpdater<kTestStateDim, kTestMeasDim> unscented_updater(&meas_model,
                                                                  unscented_updater_config);

  ImmConfig config;
  config.transition_probability.resize(2, 2);
  config.transition_probability << 0.95f, 0.05f, 0.10f, 0.90f;
  config.initial_weights.resize(2);
  config.initial_weights << 0.6f, 0.4f;
  std::vector<Filter::Predictor*> predictors{&linear_predictor, &unscented_predictor};
  std::vector<Filter::Updater*> updaters{&linear_updater, &unscented_updater};
  Filter filter(config, predictors, updaters);
  ASSERT_TRUE(filter.IsValid());

  State::MeasurementVector measurement(106.0f, 2.0f, -1.0f);
  filter.Process(measurement, 0.5f);
  EXPECT_TRUE(filter.GetCombinedState().mean.allFinite());
  EXPECT_TRUE(filter.GetCombinedState().covariance.allFinite());
  EXPECT_NEAR(filter.GetModelWeights().sum(), 1.0f, 1.0e-3f);
}

}  // namespace
}  // namespace estimation
}  // namespace common
}  // namespace oneq
