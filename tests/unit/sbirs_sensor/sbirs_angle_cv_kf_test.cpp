// 验证角度域线性标准 KF（kAngleCvKf）：预测、更新、方位过零、角速率收敛、禁止真值三维初始化。
#include <gtest/gtest.h>

#include <cmath>

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "common/numerics/Constants.h"
#include "sbirs_sensor/pipeline/SbirsEciScene.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"
#include "sbirs_sensor/pipeline/SbirsTrackingCoordinator.h"
#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"
#include "sbirs_sensor/tracking/SbirsAngleCvKalman.h"

namespace {

using sbirs_sensor::tracking::ClampElevationRad;
using sbirs_sensor::tracking::MakeInitialAngleCvState;
using sbirs_sensor::tracking::SbirsAngleCvGaussianState;
using sbirs_sensor::tracking::SbirsAngleCvPredictor;
using sbirs_sensor::tracking::SbirsAngleCvUpdater;
using sbirs_sensor::tracking::ShortestAzimuthDeltaRad;
using sbirs_sensor::tracking::WrapAzimuthRad;

constexpr float kPi = 3.14159265f;

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::session::SbirsSceneTarget HotTarget(std::uint64_t id, double y) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = id;
  target.target_name = "target";
  target.position_ecef_m = Vector(8000000.0, y, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  return target;
}

sbirs_sensor::config::SbirsSessionConfig PipelineConfig() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  config.policy.tracking.estimated_backend =
      sbirs_sensor::config::SbirsEstimatedTrackingBackend::kAngleCvKf;
  config.policy.tracking.process_noise_diff_coeff = 1.0e-8f;
  return config;
}

}  // namespace

TEST(SbirsAngleCvKfTest, PredictAdvancesAngleByRateTimesDt) {
  SbirsAngleCvGaussianState prior = MakeInitialAngleCvState(0.1f, 0.2f);
  prior.mean(1) = 0.05f;
  prior.mean(3) = -0.01f;
  oneq::common::estimation::KalmanPredictorConfig config;
  config.noise_diff_coeff = 1.0e-8f;
  const SbirsAngleCvPredictor predictor(config);
  const SbirsAngleCvGaussianState predicted = predictor.Predict(prior, 2.0f);
  EXPECT_NEAR(predicted.mean(0), 0.1f + 0.05f * 2.0f, 1.0e-5f);
  EXPECT_NEAR(predicted.mean(2), 0.2f - 0.01f * 2.0f, 1.0e-5f);
  EXPECT_FLOAT_EQ(predicted.mean(1), 0.05f);
  EXPECT_FLOAT_EQ(predicted.mean(3), -0.01f);
}

TEST(SbirsAngleCvKfTest, UpdatePullsAngleTowardMeasurement) {
  SbirsAngleCvGaussianState predicted = MakeInitialAngleCvState(0.0f, 0.0f);
  SbirsAngleCvGaussianState::MeasurementVector z;
  z << 0.02f, -0.01f;
  SbirsAngleCvGaussianState::MeasurementCovariance R =
      SbirsAngleCvGaussianState::MeasurementCovariance::Identity() * 1.0e-6f;
  const SbirsAngleCvUpdater updater;
  const auto result = updater.Update(predicted, z, R);
  EXPECT_GT(result.posterior.mean(0), 0.0f);
  EXPECT_LT(result.posterior.mean(0), 0.02f);
  EXPECT_LT(result.posterior.mean(2), 0.0f);
  EXPECT_GT(result.posterior.mean(2), -0.01f);
}

TEST(SbirsAngleCvKfTest, AzimuthWrapInnovationStaysSmall) {
  SbirsAngleCvGaussianState predicted = MakeInitialAngleCvState(kPi - 0.01f, 0.1f);
  SbirsAngleCvGaussianState::MeasurementVector z;
  z << -kPi + 0.01f, 0.1f;
  SbirsAngleCvGaussianState::MeasurementCovariance R =
      SbirsAngleCvGaussianState::MeasurementCovariance::Identity() * 1.0e-6f;
  const SbirsAngleCvUpdater updater;
  const auto result = updater.Update(predicted, z, R);
  EXPECT_LT(std::fabs(result.innovation(0)), 0.05f);
  EXPECT_LT(std::fabs(ShortestAzimuthDeltaRad(result.posterior.mean(0), -kPi + 0.01f)), 0.05f);
}

TEST(SbirsAngleCvKfTest, ConstantRateTrackRmseBelowMeasurementNoise) {
  const float true_az0 = 0.3f;
  const float true_el0 = -0.1f;
  const float true_waz = 0.02f;
  const float true_wel = 0.005f;
  const float dt = 0.1f;
  const float meas_std = 0.003f;
  oneq::common::estimation::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0e-10f;
  const SbirsAngleCvPredictor predictor(pred_cfg);
  const SbirsAngleCvUpdater updater;
  SbirsAngleCvGaussianState::MeasurementCovariance R =
      SbirsAngleCvGaussianState::MeasurementCovariance::Identity() * (meas_std * meas_std);

  SbirsAngleCvGaussianState state = MakeInitialAngleCvState(true_az0, true_el0);
  double az_err_sq = 0.0;
  const int steps = 80;
  for (int i = 1; i <= steps; ++i) {
    const float t = dt * static_cast<float>(i);
    state = predictor.Predict(state, dt);
    SbirsAngleCvGaussianState::MeasurementVector z;
    // 确定性"噪声"：交替符号，幅度为 meas_std，避免引入随机源。
    const float dither = ((i % 2) == 0) ? meas_std : -meas_std;
    z << WrapAzimuthRad(true_az0 + true_waz * t + dither),
        ClampElevationRad(true_el0 + true_wel * t + 0.5f * dither);
    state = updater.Update(state, z, R).posterior;
    const float true_az = WrapAzimuthRad(true_az0 + true_waz * t);
    const float true_el = true_el0 + true_wel * t;
    const float daz = ShortestAzimuthDeltaRad(state.mean(0), true_az);
    const float del = state.mean(2) - true_el;
    az_err_sq += static_cast<double>(daz * daz + del * del);
  }
  const float rmse = static_cast<float>(std::sqrt(az_err_sq / static_cast<double>(steps)));
  EXPECT_LT(rmse, meas_std);
  EXPECT_NEAR(state.mean(1), true_waz, 0.01f);
  EXPECT_NEAR(state.mean(3), true_wel, 0.01f);
}

TEST(SbirsAngleCvKfTest, InitializeIgnoresTruthEciPosition) {
  sbirs_sensor::pipeline::SbirsEciSceneTarget target;
  target.target_id = 9U;
  target.position_eci_m = Vector(1.0e7, 2.0e7, 3.0e7);
  target.has_velocity_eci_m_per_s = true;
  target.velocity_eci_m_per_s = Vector(1000.0, 2000.0, 3000.0);
  sbirs_sensor::config::SbirsTrackingConfig tracking;
  tracking.estimated_backend = sbirs_sensor::config::SbirsEstimatedTrackingBackend::kAngleCvKf;
  sbirs_sensor::pipeline::SbirsTrackingCoordinator coordinator;
  coordinator.InitializeTarget(9U, target, tracking, 12.0f, -4.0f);
  const auto runtime = coordinator.CaptureRuntimeState();
  ASSERT_EQ(runtime.filter_states.count(9U), 0U);
  ASSERT_EQ(runtime.angle_kf_states.count(9U), 1U);
  const auto& mean = runtime.angle_kf_states.at(9U).mean;
  EXPECT_NEAR(mean(0), oneq::common::numerics::DegToRad(12.0f), 1.0e-6f);
  EXPECT_NEAR(mean(2), oneq::common::numerics::DegToRad(-4.0f), 1.0e-6f);
  EXPECT_FLOAT_EQ(mean(1), 0.0f);
  EXPECT_FLOAT_EQ(mean(3), 0.0f);
}

TEST(SbirsAngleCvKfTest, PipelineOptInProducesEstimatedAnglesWithoutSixDFilter) {
  sbirs_sensor::config::SbirsSessionConfig config = PipelineConfig();
  sbirs_sensor::pipeline::SbirsPipeline pipeline(
      sbirs_sensor::runtime::MapSessionToInternal(config));
  sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(HotTarget(7U, 0.0))
          .Build();
  pipeline.RunCycle(input);
  input.cycle_index = 2U;
  input.scene[0] = HotTarget(7U, 5000.0);
  const auto result = pipeline.RunCycle(input);
  ASSERT_FALSE(result.detections.empty());
  EXPECT_TRUE(result.detections.front().record.detected);
  EXPECT_TRUE(result.detections.front().attribution.has_estimation_nis);
  const auto snapshot = pipeline.CaptureRuntimeState();
  EXPECT_TRUE(snapshot.filter_states.empty());
  EXPECT_FALSE(snapshot.angle_kf_states.empty());
}
