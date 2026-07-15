/**
 * @file estimation_kalman_test.cpp
 * @brief 验证 common/estimation 模板化 Kalman 滤波家族（6/3 实例化）。
 *
 * 覆盖 GaussianState 维度、KF/EKF/UDKF/SRIF 后端 predict-update、动态 R、以及非 6/3 维度
 * 模板实例化（2 维量测）。数值断言与 airborne_radar 原实现保持同一量级，确保迁移零回归。
 */

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "common/estimation/EkfFilter.h"
#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/estimation/ImmFilter.h"
#include "common/estimation/KalmanPredictor.h"
#include "common/estimation/KalmanUpdater.h"
#include "common/estimation/SrifPredictor.h"
#include "common/estimation/SrifUpdater.h"
#include "common/estimation/UdkfPredictor.h"
#include "common/estimation/UdkfUpdater.h"

namespace oneq {
namespace common {
namespace estimation {
namespace {

constexpr int kTestStateDim = 6;
constexpr int kTestMeasDim = 3;
constexpr float kTolerance = 1e-4f;

using State = GaussianState<kTestStateDim, kTestMeasDim>;

/** @brief 构造初始先验：位置 (100,0,0)，各维方差 100，速度方差 25。 */
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

TEST(EstimationGaussianStateTest, DefaultStateIsZeroMeanIdentityCovariance) {
  GaussianState<4, 2> s;
  EXPECT_TRUE(s.mean.isZero());
  EXPECT_TRUE(s.covariance.isIdentity());
  EXPECT_EQ(s.state_dim, 4);
  EXPECT_EQ(s.measurement_dim, 2);
}

TEST(EstimationGaussianStateTest, GaussianTrackStateAliasIsSixByThree) {
  EXPECT_EQ(GaussianTrackState::state_dim, 6);
  EXPECT_EQ(GaussianTrackState::measurement_dim, 3);
  EXPECT_EQ(GaussianTrackState::StateVector::RowsAtCompileTime, 6);
  EXPECT_EQ(GaussianTrackState::MeasurementVector::RowsAtCompileTime, 3);
}

TEST(EstimationKalmanPredictorTest, TransitionMatrixAppliesCvCoupling) {
  using F = KalmanPredictor<kTestStateDim, kTestMeasDim>;
  const auto Fmat = F::BuildTransitionMatrix(1.0f);
  // CV: F(0,1)=F(2,3)=F(4,5)=dt
  EXPECT_FLOAT_EQ(Fmat(0, 1), 1.0f);
  EXPECT_FLOAT_EQ(Fmat(2, 3), 1.0f);
  EXPECT_FLOAT_EQ(Fmat(4, 5), 1.0f);
  EXPECT_FLOAT_EQ(Fmat(1, 0), 0.0f);
}

TEST(EstimationKalmanPredictorTest, PredictPropagatesMeanByTransitionMatrix) {
  KalmanPredictorConfig config;
  config.noise_diff_coeff = 1.0f;
  KalmanPredictor<kTestStateDim, kTestMeasDim> predictor(config);
  const auto prior = MakePrior();
  const auto predicted = predictor.Predict(prior, 1.0f);
  // x stays 100, vx=0 → no drift; covariance grows by Q
  EXPECT_NEAR(predicted.mean(0), 100.0f, kTolerance);
  EXPECT_GT(predicted.covariance(0, 0), prior.covariance(0, 0));
}

TEST(EstimationKalmanUpdaterTest, PerfectMeasurementKeepsPosition) {
  KalmanUpdaterConfig config;
  config.measurement_noise_std = 0.001f;
  KalmanUpdater<kTestStateDim, kTestMeasDim> updater(config);
  KalmanPredictor<kTestStateDim, kTestMeasDim> predictor;
  const auto predicted = predictor.Predict(MakePrior(), 1.0f);
  State::MeasurementVector measurement(105.0f, 0.0f, 0.0f);
  const auto result = updater.Update(predicted, measurement);
  // 近完美量测 → 后验位置接近测量值
  EXPECT_NEAR(result.posterior.mean(0), 105.0f, 1.0f);
  // 新息指向测量与预测之差
  EXPECT_NEAR(result.innovation(0), 5.0f, kTolerance);
}

TEST(EstimationKalmanUpdaterTest, DynamicRCovariancePathMatchesUpdate) {
  KalmanUpdater<kTestStateDim, kTestMeasDim> updater;
  KalmanPredictor<kTestStateDim, kTestMeasDim> predictor;
  const auto predicted = predictor.Predict(MakePrior(), 1.0f);
  State::MeasurementVector measurement(105.0f, 0.0f, 0.0f);
  State::MeasurementCovariance dynamic_R = State::MeasurementCovariance::Identity() * 100.0f;
  const auto result = updater.Update(predicted, measurement, dynamic_R);
  EXPECT_NEAR(result.innovation_covariance(0, 0),
              predicted.covariance(0, 0) + 100.0f, 1.0f);
}

TEST(EstimationEkfTest, LinearModelsMatchStandardKalman) {
  KalmanPredictorConfig pc;
  pc.noise_diff_coeff = 1.0f;
  KalmanPredictor<kTestStateDim, kTestMeasDim> std_predictor(pc);
  KalmanUpdaterConfig uc;
  uc.measurement_noise_std = 10.0f;
  KalmanUpdater<kTestStateDim, kTestMeasDim> std_updater(uc);

  LinearCvTransitionModel<kTestStateDim> cv_model;
  EkfPredictorConfig epc;
  epc.noise_diff_coeff = 1.0f;
  EkfPredictor<kTestStateDim, kTestMeasDim> ekf_predictor(&cv_model, epc);
  LinearPositionMeasurementModel<kTestStateDim, kTestMeasDim> meas_model;
  EkfUpdaterConfig euc;
  euc.measurement_noise_std = 10.0f;
  EkfUpdater<kTestStateDim, kTestMeasDim> ekf_updater(&meas_model, euc);

  const auto prior = MakePrior();
  const auto std_predicted = std_predictor.Predict(prior, 1.0f);
  const auto ekf_predicted = ekf_predictor.Predict(prior, 1.0f);
  EXPECT_TRUE(std_predicted.mean.isApprox(ekf_predicted.mean, kTolerance));
  EXPECT_TRUE(std_predicted.covariance.isApprox(ekf_predicted.covariance, kTolerance));

  State::MeasurementVector measurement(105.0f, 0.0f, 0.0f);
  const auto std_result = std_updater.Update(std_predicted, measurement);
  const auto ekf_result = ekf_updater.Update(ekf_predicted, measurement);
  EXPECT_TRUE(std_result.posterior.mean.isApprox(ekf_result.posterior.mean, kTolerance));
  EXPECT_TRUE(
      std_result.posterior.covariance.isApprox(ekf_result.posterior.covariance, kTolerance));
}

TEST(EstimationUdkfAndSrifTest, BothBackendsProduceFiniteUpdate) {
  KalmanPredictor<kTestStateDim, kTestMeasDim> predictor;
  const auto predicted = predictor.Predict(MakePrior(), 1.0f);
  State::MeasurementVector measurement(105.0f, 0.0f, 0.0f);

  UdkfUpdater<kTestStateDim, kTestMeasDim> udkf;
  SrifUpdater<kTestStateDim, kTestMeasDim> srif;
  const auto udkf_result = udkf.Update(predicted, measurement);
  const auto srif_result = srif.Update(predicted, measurement);

  EXPECT_TRUE(udkf_result.posterior.mean.allFinite());
  EXPECT_TRUE(udkf_result.posterior.covariance.allFinite());
  EXPECT_TRUE(srif_result.posterior.mean.allFinite());
  EXPECT_TRUE(srif_result.posterior.covariance.allFinite());
  EXPECT_NEAR(udkf_result.posterior.mean(0), srif_result.posterior.mean(0), 5.0f);
}

// 验证非 6/3 维度也能模板实例化（2 维量测，模拟 SBIRS 角度量测未来场景）
TEST(EstimationNonStandardDimTest, TwoDimMeasurementInstantiates) {
  // 6 维状态 / 2 维量测：量测维不影响 predict（CV 仍按 6 维状态工作）。
  using State6x2 = GaussianState<6, 2>;
  State6x2 s;
  s.mean << 1, 2, 3, 4, 5, 6;
  s.covariance = State6x2::StateCovariance::Identity();
  EXPECT_EQ(s.measurement_dim, 2);
  EXPECT_EQ(State6x2::MeasurementMatrix::RowsAtCompileTime, 2);

  // 量测维≠3 时位置提取矩阵应为零（无 CV 默认布局），但不影响实例化
  const auto H = IKalmanUpdater<6, 2>::BuildPositionMeasurementMatrix();
  EXPECT_TRUE(H.isZero());

  // 状态维仍为 6 → CV predict 正常工作（mean 经 F 传播，协方差增长）
  KalmanPredictor<6, 2> predictor;
  const auto predicted = predictor.Predict(s, 1.0f);
  EXPECT_FALSE(predicted.mean.isApprox(s.mean));  // F 非单位，mean 改变
  EXPECT_GT(predicted.covariance(0, 0), s.covariance(0, 0));  // Q 注入，方差增长

  // 状态维≠6 时 CV 结构退化：F=单位、Q=零，predict 应保持原状态不变
  using State4x2 = GaussianState<4, 2>;
  State4x2 s4;
  s4.mean << 1, 2, 3, 4;
  s4.covariance = State4x2::StateCovariance::Identity();
  KalmanPredictor<4, 2> predictor4;
  const auto predicted4 = predictor4.Predict(s4, 1.0f);
  EXPECT_TRUE(predicted4.mean.isApprox(s4.mean));
  EXPECT_TRUE(predicted4.covariance.isApprox(s4.covariance));
}

TEST(EstimationImmFilterTest, SplitPredictCorrectMatchesProcessWithDynamicR) {
  using Filter = ImmFilter<kTestStateDim, kTestMeasDim>;
  KalmanPredictorConfig slow_config;
  slow_config.noise_diff_coeff = 1.0f;
  KalmanPredictorConfig fast_config;
  fast_config.noise_diff_coeff = 25.0f;
  KalmanPredictor<kTestStateDim, kTestMeasDim> slow_predictor(slow_config);
  KalmanPredictor<kTestStateDim, kTestMeasDim> fast_predictor(fast_config);
  KalmanUpdater<kTestStateDim, kTestMeasDim> slow_updater;
  KalmanUpdater<kTestStateDim, kTestMeasDim> fast_updater;

  ImmConfig config;
  config.transition_probability.resize(2, 2);
  config.transition_probability << 0.95f, 0.05f, 0.10f, 0.90f;
  config.initial_weights.resize(2);
  config.initial_weights << 0.6f, 0.4f;
  std::vector<Filter::Predictor*> predictors{&slow_predictor, &fast_predictor};
  std::vector<Filter::Updater*> updaters{&slow_updater, &fast_updater};
  Filter complete(config, predictors, updaters);
  Filter split(config, predictors, updaters);

  const State prior = MakePrior();
  std::vector<Filter::ModelState> states(2);
  states[0] = Filter::ModelState(prior, 0.6f);
  states[1] = Filter::ModelState(prior, 0.4f);
  complete.SetModelStates(states);
  split.SetModelStates(states);

  State::MeasurementVector measurement(106.0f, 2.0f, -1.0f);
  State::MeasurementCovariance dynamic_R =
      State::MeasurementCovariance::Identity() * 16.0f;
  complete.Process(measurement, 0.5f, dynamic_R);
  split.Predict(0.5f);
  split.Correct(measurement, dynamic_R);

  EXPECT_TRUE(complete.GetCombinedState().mean.isApprox(split.GetCombinedState().mean, 1.0e-6f));
  EXPECT_TRUE(complete.GetCombinedState().covariance.isApprox(
      split.GetCombinedState().covariance, 1.0e-5f));
  EXPECT_TRUE(complete.GetModelWeights().isApprox(split.GetModelWeights(), 1.0e-6f));
  const auto& complete_results = complete.GetModelUpdateResults();
  const auto& split_results = split.GetModelUpdateResults();
  ASSERT_EQ(complete_results.size(), split_results.size());
  for (std::size_t index = 0U; index < complete_results.size(); ++index) {
    EXPECT_TRUE(complete_results[index].innovation.isApprox(split_results[index].innovation,
                                                            1.0e-6f));
    EXPECT_TRUE(complete_results[index].innovation_covariance.isApprox(
        split_results[index].innovation_covariance, 1.0e-5f));
  }
}

}  // namespace
}  // namespace estimation
}  // namespace common
}  // namespace oneq
