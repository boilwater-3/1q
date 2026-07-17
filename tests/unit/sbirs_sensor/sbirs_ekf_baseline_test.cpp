// design.md 2.5a 验证入口：SBIRS EKF 基线 NIS 诊断。
// 覆盖恒速 CV 适配场景与突发机动失配场景，为是否需要 IMM 提供最小可执行证据。
#include <gtest/gtest.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <vector>

#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace {

constexpr float kChiSquare2Dof95 = 5.99f;

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::tracking::SbirsGaussianState MakeState(double x, double y, double z, double vx,
                                                     double vy, double vz, float position_std_m,
                                                     float velocity_std_m_per_s) {
  sbirs_sensor::tracking::SbirsGaussianState state;
  state.mean.setZero();
  state.mean(0) = static_cast<float>(x);
  state.mean(1) = static_cast<float>(vx);
  state.mean(2) = static_cast<float>(y);
  state.mean(3) = static_cast<float>(vy);
  state.mean(4) = static_cast<float>(z);
  state.mean(5) = static_cast<float>(vz);

  const float pos_var = position_std_m * position_std_m;
  const float vel_var = velocity_std_m_per_s * velocity_std_m_per_s;
  state.covariance = sbirs_sensor::tracking::SbirsStateCovariance::Zero();
  state.covariance(0, 0) = pos_var;
  state.covariance(1, 1) = vel_var;
  state.covariance(2, 2) = pos_var;
  state.covariance(3, 3) = vel_var;
  state.covariance(4, 4) = pos_var;
  state.covariance(5, 5) = vel_var;
  return state;
}

float NormalizedInnovationSquared(
    const sbirs_sensor::tracking::SbirsKalmanUpdateResult& update_result) {
  const Eigen::LLT<sbirs_sensor::tracking::SbirsMeasurementCovariance> llt(
      update_result.innovation_covariance);
  const sbirs_sensor::tracking::SbirsMeasurementVector solved =
      llt.solve(update_result.innovation);
  return update_result.innovation.dot(solved);
}

struct NisScenarioSummary {
  float max_nis{0.0f};
  int gate_exceeded_count{0};
};

NisScenarioSummary RunLateralOffsetScenario(const std::vector<double>& lateral_offsets_m) {
  sbirs_sensor::tracking::SbirsAngleMeasurementModel measurement_model;
  measurement_model.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));

  sbirs_sensor::tracking::SbirsEkfPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = 0.1f;
  sbirs_sensor::tracking::SbirsCvTransitionModel transition_model;
  const sbirs_sensor::tracking::SbirsEkfPredictor predictor(&transition_model, predictor_config);
  const sbirs_sensor::tracking::SbirsEkfUpdater updater(&measurement_model);

  sbirs_sensor::config::SbirsErrorModelConfig error_model;
  error_model.attitude_sigma_deg = 0.02f;
  error_model.orbit_sigma_deg = 0.0f;
  error_model.fov_sigma_deg = 0.0f;
  const sbirs_sensor::tracking::SbirsMeasurementCovariance R =
      sbirs_sensor::tracking::BuildMeasurementCovariance(error_model, 1000000.0, 0.0f, 0.0f);

  sbirs_sensor::tracking::SbirsGaussianState state =
      MakeState(8000000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 100.0f, 1.0f);

  NisScenarioSummary summary;
  for (std::size_t i = 0U; i < lateral_offsets_m.size(); ++i) {
    const int cycle = static_cast<int>(i) + 1;
    const sbirs_sensor::tracking::SbirsGaussianState predicted = predictor.Predict(state, 1.0f);
    const double truth_y = 1000.0 * static_cast<double>(cycle) + lateral_offsets_m[i];
    const sbirs_sensor::tracking::SbirsGaussianState truth =
        MakeState(8000000.0, truth_y, 0.0, 0.0, 1000.0, 0.0, 100.0f, 1.0f);
    const sbirs_sensor::tracking::SbirsMeasurementVector measurement =
        measurement_model.Function(truth.mean);
    const sbirs_sensor::tracking::SbirsKalmanUpdateResult update_result =
        updater.Update(predicted, measurement, R);
    const float nis = NormalizedInnovationSquared(update_result);
    summary.max_nis = std::max(summary.max_nis, nis);
    if (nis > kChiSquare2Dof95) {
      ++summary.gate_exceeded_count;
    }
    state = update_result.posterior;
  }
  return summary;
}

}  // namespace

TEST(SbirsEkfBaselineTest, ConstantVelocityMeasurementsKeepNisBelowGate) {
  sbirs_sensor::tracking::SbirsAngleMeasurementModel measurement_model;
  measurement_model.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));

  sbirs_sensor::tracking::SbirsEkfPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = 0.1f;
  sbirs_sensor::tracking::SbirsCvTransitionModel transition_model;
  const sbirs_sensor::tracking::SbirsEkfPredictor predictor(&transition_model, predictor_config);
  const sbirs_sensor::tracking::SbirsEkfUpdater updater(&measurement_model);

  sbirs_sensor::config::SbirsErrorModelConfig error_model;
  error_model.attitude_sigma_deg = 0.02f;
  error_model.orbit_sigma_deg = 0.0f;
  error_model.fov_sigma_deg = 0.0f;
  const sbirs_sensor::tracking::SbirsMeasurementCovariance R =
      sbirs_sensor::tracking::BuildMeasurementCovariance(error_model, 1000000.0, 0.0f, 0.0f);

  sbirs_sensor::tracking::SbirsGaussianState state =
      MakeState(8000000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 100.0f, 1.0f);

  for (int cycle = 1; cycle <= 5; ++cycle) {
    const sbirs_sensor::tracking::SbirsGaussianState predicted = predictor.Predict(state, 1.0f);
    const sbirs_sensor::tracking::SbirsGaussianState truth =
        MakeState(8000000.0, 1000.0 * static_cast<double>(cycle), 0.0, 0.0, 1000.0, 0.0,
                  100.0f, 1.0f);
    const sbirs_sensor::tracking::SbirsMeasurementVector measurement =
        measurement_model.Function(truth.mean);
    const sbirs_sensor::tracking::SbirsKalmanUpdateResult update_result =
        updater.Update(predicted, measurement, R);

    EXPECT_LT(NormalizedInnovationSquared(update_result), kChiSquare2Dof95);
    state = update_result.posterior;
  }
}

TEST(SbirsEkfBaselineTest, SuddenCrossRangeManeuverRaisesNisAboveGate) {
  sbirs_sensor::tracking::SbirsAngleMeasurementModel measurement_model;
  measurement_model.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));

  sbirs_sensor::tracking::SbirsEkfPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = 0.1f;
  sbirs_sensor::tracking::SbirsCvTransitionModel transition_model;
  const sbirs_sensor::tracking::SbirsEkfPredictor predictor(&transition_model, predictor_config);
  const sbirs_sensor::tracking::SbirsEkfUpdater updater(&measurement_model);

  sbirs_sensor::config::SbirsErrorModelConfig error_model;
  error_model.attitude_sigma_deg = 0.02f;
  error_model.orbit_sigma_deg = 0.0f;
  error_model.fov_sigma_deg = 0.0f;
  const sbirs_sensor::tracking::SbirsMeasurementCovariance R =
      sbirs_sensor::tracking::BuildMeasurementCovariance(error_model, 1000000.0, 0.0f, 0.0f);

  const sbirs_sensor::tracking::SbirsGaussianState initial =
      MakeState(8000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0f, 1.0f);
  const sbirs_sensor::tracking::SbirsGaussianState predicted = predictor.Predict(initial, 1.0f);
  const sbirs_sensor::tracking::SbirsGaussianState maneuvered_truth =
      MakeState(8000000.0, 50000.0, 0.0, 0.0, 50000.0, 0.0, 100.0f, 1.0f);
  const sbirs_sensor::tracking::SbirsMeasurementVector measurement =
      measurement_model.Function(maneuvered_truth.mean);
  const sbirs_sensor::tracking::SbirsKalmanUpdateResult update_result =
      updater.Update(predicted, measurement, R);

  EXPECT_GT(NormalizedInnovationSquared(update_result), kChiSquare2Dof95);
}

TEST(SbirsEkfBaselineTest, ScenarioMatrixSeparatesCvTransientAndSustainedMismatch) {
  const NisScenarioSummary cv =
      RunLateralOffsetScenario(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
  EXPECT_LT(cv.max_nis, kChiSquare2Dof95);
  EXPECT_EQ(cv.gate_exceeded_count, 0);

  const NisScenarioSummary transient =
      RunLateralOffsetScenario(std::vector<double>{0.0, 0.0, 50000.0, 0.0, 0.0});
  EXPECT_GT(transient.max_nis, kChiSquare2Dof95);
  EXPECT_GT(transient.gate_exceeded_count, 0);

  const NisScenarioSummary sustained =
      RunLateralOffsetScenario(std::vector<double>{0.0, 25000.0, 50000.0, 75000.0, 100000.0});
  EXPECT_GT(sustained.max_nis, transient.max_nis);
  EXPECT_GT(sustained.gate_exceeded_count, transient.gate_exceeded_count);
  EXPECT_GE(sustained.gate_exceeded_count, 3);
}
