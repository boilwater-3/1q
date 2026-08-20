#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "precision_evaluation/SbirsBearingAdapter.h"

namespace {

namespace pe = precision_evaluation;

constexpr double kPi = 3.14159265358979323846;
// GMST≈0 的儒略日（ECI≡ECEF，同 SBIRS 单测约定）。
constexpr double kUtcJulianDay = 2451544.2230698913;

sbirs_sensor::config::SbirsSessionConfig SatelliteConfig(float scan_start_az_deg) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  config.mission.scan_start_az_deg = scan_start_az_deg;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  // 零误差注入：量测角=真值角（角度/双星交会误差应≈0）。
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  return config;
}

// 双星场景：A 星 (7e6,0,0)（目标方位 ≈80.5°）、B 星 (0,7e6,0)（目标方位 ≈352.9°），
// 目标 (8e6,6e6,0)（|r|=1e7）+ 指向地心的径向下降速度 −4000·r̂（径向入射保证推演
// 时域内落地；纯切向速度会先绕半圈椭圆、远超 1200 s 时域）。
pe::EvaluationTruthTarget DescendingTarget() {
  pe::EvaluationTruthTarget truth;
  truth.key = 1U;
  truth.position_ecef_m = oneq::coordinate::EcefPositionM(8.0e6, 6.0e6, 0.0);
  truth.has_velocity = true;
  truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(-3200.0, -2400.0, 0.0);
  truth.radiant_intensity_w_per_sr = 1.0e8;
  return truth;
}

pe::PrecisionEvaluationConfig EvaluationConfig() {
  pe::PrecisionEvaluationConfig config;
  config.satellite_a = SatelliteConfig(79.5f);   // 周期 1 扫描中心 ≈80.5°（正对目标）
  config.satellite_b = SatelliteConfig(351.9f);  // 周期 1 扫描中心 ≈352.9°
  config.inference.prediction_horizon_sec = 1200.0;  // 径向入射 ~480 s 落地
  return config;
}

pe::DualSatEphemerisInput DualSatEphemeris() {
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = oneq::coordinate::EcefPositionM(7.0e6, 0.0, 0.0);
  ephemeris.satellite_b_position_ecef_m = oneq::coordinate::EcefPositionM(0.0, 7.0e6, 0.0);
  return ephemeris;
}

class PrecisionEvaluationSessionRunner {
 public:
  explicit PrecisionEvaluationSessionRunner(const pe::PrecisionEvaluationConfig& config)
      : session_(config) {}

  pe::PrecisionEvaluationReport Run(std::uint32_t cycle_count,
                                    pe::EvaluationTruthTarget truth = DescendingTarget()) {
    for (std::uint32_t cycle = 1U; cycle <= cycle_count; ++cycle) {
      // 真值弹道逐周期推进（位置 += 速度·dt）：真值状态必须自洽，否则融合会正确地
      // 收敛到静止目标、落点推演超时域。
      truth.position_ecef_m.x_m += truth.velocity_ecef_m_per_s.x_mps;
      truth.position_ecef_m.y_m += truth.velocity_ecef_m_per_s.y_mps;
      truth.position_ecef_m.z_m += truth.velocity_ecef_m_per_s.z_mps;
      last_cycle_result_ = session_.Step(cycle, 1.0f, kUtcJulianDay, DualSatEphemeris(), {truth});
    }
    return session_.Summarize();
  }

  const pe::PrecisionEvaluationCycleResult& last_cycle_result() const { return last_cycle_result_; }

 private:
  pe::PrecisionEvaluationSession session_;
  pe::PrecisionEvaluationCycleResult last_cycle_result_;
};

TEST(PrecisionEvaluationSessionTest, ZeroNoiseScenarioYieldsSmallAngularAndDualSatErrors) {
  PrecisionEvaluationSessionRunner runner(EvaluationConfig());
  const pe::PrecisionEvaluationReport report = runner.Run(25U);

  // 角度误差：零误差注入下量测角=真值角（含估计跟踪模式，真值初始化后验≈真值）。
  ASSERT_GT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kAngular)].count, 0U);
  EXPECT_LT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kAngular)].max, 0.01);

  // 双星交会：两条精确视线交于真值位置，误差为浮点噪声量级。
  ASSERT_GT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)].count, 0U);
  EXPECT_LT(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)].max, 50.0);
}

TEST(PrecisionEvaluationSessionTest, VelocityAndImpactSamplesAreProducedAndFinite) {
  PrecisionEvaluationSessionRunner runner(EvaluationConfig());
  const pe::PrecisionEvaluationReport report = runner.Run(60U);

  // 速度误差：融合逐航迹滤波（评估会话强制开启）产出运动学估计后逐周期采样。
  const pe::ErrorMetricSummary& velocity =
      report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kVelocity)];
  ASSERT_GT(velocity.count, 0U);
  EXPECT_TRUE(std::isfinite(velocity.rmse));
  EXPECT_GE(velocity.rmse, 0.0);

  // 落点误差：真值状态（下降目标，时域内落地）与估计状态经同一推演引擎的落点之差。
  const pe::ErrorMetricSummary& impact =
      report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kImpactPoint)];
  ASSERT_GT(impact.count, 0U);
  EXPECT_TRUE(std::isfinite(impact.max));
  // 序列 max 含滤波早期未收敛状态（状态误差传播 ~480 s 放大），不做紧上界断言；
  // 收敛性由末周期样本验证（双源零噪声方位 60 周期收敛后应远小于 300 km）。
  ASSERT_FALSE(runner.last_cycle_result().keypoints.empty());
  const pe::KeyPointErrorSample& last_keypoint = runner.last_cycle_result().keypoints.back();
  ASSERT_TRUE(last_keypoint.has_impact);
  EXPECT_LT(last_keypoint.impact_error_m, 3.0e5);

  // AHP：默认等权矩阵合法，综合分在 (0,1]。
  EXPECT_TRUE(report.ahp_valid);
  EXPECT_TRUE(report.ahp.is_consistent);
  EXPECT_GT(report.composite_score, 0.0);
  EXPECT_LE(report.composite_score, 1.0);
}

TEST(PrecisionEvaluationSessionTest, SessionIsDeterministicAcrossRuns) {
  const pe::PrecisionEvaluationReport first =
      PrecisionEvaluationSessionRunner(EvaluationConfig()).Run(12U);
  const pe::PrecisionEvaluationReport second =
      PrecisionEvaluationSessionRunner(EvaluationConfig()).Run(12U);
  EXPECT_DOUBLE_EQ(first.composite_score, second.composite_score);
  for (std::size_t index = 0U; index < pe::kPrecisionMetricCount; ++index) {
    EXPECT_EQ(first.metrics[index].count, second.metrics[index].count);
    EXPECT_DOUBLE_EQ(first.metrics[index].rmse, second.metrics[index].rmse);
  }
}

TEST(PrecisionEvaluationSessionTest, InvalidJudgmentMatrixYieldsInvalidAhpAndZeroComposite) {
  pe::PrecisionEvaluationConfig config = EvaluationConfig();
  config.ahp.values[2][2] = 2.0;  // 对角非 1 → 非法判断矩阵
  PrecisionEvaluationSessionRunner runner(config);
  const pe::PrecisionEvaluationReport report = runner.Run(3U);
  EXPECT_FALSE(report.ahp_valid);
  EXPECT_DOUBLE_EQ(report.composite_score, 0.0);  // 不静默退化为等权
}

TEST(PrecisionEvaluationSessionTest, EmptyTruthYieldsZeroEvidenceAndZeroComposite) {
  // 空真值：无样本、不崩溃；零证据 = 零分（无样本指标按 0 分计入综合，不因
  // count=0 → rmse=0 而得满分）。
  pe::PrecisionEvaluationSession session(EvaluationConfig());
  const pe::PrecisionEvaluationCycleResult empty =
      session.Step(1U, 1.0f, kUtcJulianDay, DualSatEphemeris(), {});
  EXPECT_TRUE(empty.angular.empty());
  EXPECT_TRUE(empty.dual_sat.empty());
  EXPECT_TRUE(empty.velocity.empty());
  EXPECT_TRUE(empty.keypoints.empty());

  const pe::PrecisionEvaluationReport report = session.Summarize();
  EXPECT_TRUE(report.ahp_valid);  // 矩阵本身合法
  EXPECT_FALSE(report.all_metrics_sampled);
  EXPECT_DOUBLE_EQ(report.composite_score, 0.0);
  for (std::size_t index = 0U; index < pe::kPrecisionMetricCount; ++index) {
    EXPECT_EQ(report.metrics[index].count, 0U);
    EXPECT_DOUBLE_EQ(report.metric_scores[index], 0.0);
  }
}

TEST(PrecisionEvaluationSessionTest, SingleSatelliteDetectionSkipsDualSatFix) {
  // B 星扫描背向目标 → 仅 A 星检出：角度样本只来自卫星 0，无双星交会样本。
  pe::PrecisionEvaluationConfig config = EvaluationConfig();
  config.satellite_b.mission.scan_start_az_deg = 200.0f;
  PrecisionEvaluationSessionRunner runner(config);
  const pe::PrecisionEvaluationReport report = runner.Run(5U);
  const pe::PrecisionEvaluationCycleResult& last = runner.last_cycle_result();
  ASSERT_FALSE(last.angular.empty());
  for (const pe::AngularErrorSample& sample : last.angular) {
    EXPECT_EQ(sample.satellite_index, 0);
  }
  EXPECT_TRUE(last.dual_sat.empty());
  EXPECT_EQ(report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)].count, 0U);
  // 零证据=零分：无样本指标 score=0 拖低综合；有样本指标照常计分（综合分因此
  // 介于 0 与"3/5 满分"之间——五个等权指标中双星/发射点两项零分）。
  EXPECT_DOUBLE_EQ(
      report.metric_scores[static_cast<std::size_t>(pe::PrecisionMetric::kDualSatFix)], 0.0);
  EXPECT_DOUBLE_EQ(
      report.metric_scores[static_cast<std::size_t>(pe::PrecisionMetric::kLaunchPoint)], 0.0);
  EXPECT_GT(report.metric_scores[static_cast<std::size_t>(pe::PrecisionMetric::kAngular)], 0.0);
  EXPECT_GT(report.composite_score, 0.0);
  EXPECT_LT(report.composite_score, 0.61);
}

TEST(PrecisionEvaluationSessionTest, AscendingTargetYieldsLaunchPointSamples) {
  // 径向上升目标：前推无落点（时域外）、回推解出发射点（反向下降至地表）。
  pe::EvaluationTruthTarget truth = DescendingTarget();
  truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(3200.0, 2400.0, 0.0);
  PrecisionEvaluationSessionRunner runner(EvaluationConfig());
  const pe::PrecisionEvaluationReport report = runner.Run(60U, truth);

  const pe::ErrorMetricSummary& launch =
      report.metrics[static_cast<std::size_t>(pe::PrecisionMetric::kLaunchPoint)];
  ASSERT_GT(launch.count, 0U);
  EXPECT_TRUE(std::isfinite(launch.max));
  ASSERT_FALSE(runner.last_cycle_result().keypoints.empty());
  EXPECT_TRUE(runner.last_cycle_result().keypoints.back().has_launch);
}

// --- SBIRS→fusion 适配器几何往返（评估侧跨系对齐的专项验证） ---

TEST(SbirsBearingAdapterTest, EciBearingMapsToSatelliteLocalEnu) {
  // 赤道 x 轴卫星 (7e6,0,0)（大地=地心坐标重合），目标 (8e6,2e6,0)：ECI 视线
  // (1,2,0)/√5 → ENU（east=(0,1,0), north=(0,0,1), up=(1,0,0)）：
  // az=atan2(east,north)=90°、el=asin(up 分量)=asin(1/√5)。
  sbirs_sensor::session::SbirsCycleResult result;
  result.status = sbirs_sensor::session::SbirsCycleStatus::kCompleted;
  sbirs_sensor::output::SbirsDetectionRecord detection;
  detection.detection_id = 1U;
  const double los_x = 1.0e6;
  const double los_y = 2.0e6;
  const double los_norm = std::sqrt(los_x * los_x + los_y * los_y);
  detection.azimuth_rad = static_cast<float>(std::atan2(los_y, los_x));
  detection.elevation_rad = 0.0f;
  detection.infrared_snr_linear = 500.0f;
  detection.detected = true;
  result.output_frame.detections.push_back(detection);
  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 1U;
  attribution.target_id = 42U;
  result.detection_attributions.push_back(attribution);

  const oneq::coordinate::EcefPositionM satellite(7.0e6, 0.0, 0.0);
  const std::vector<fusion::DetectionRecord> records =
      precision_evaluation::internal::AdaptSbirsResultToDetectionRecords(result, satellite, 0.0,
                                                                         7U);
  ASSERT_EQ(records.size(), 1U);
  const fusion::DetectionRecord& record = records.front();
  EXPECT_EQ(record.key, 42U);
  EXPECT_EQ(record.source_id, 7U);
  EXPECT_TRUE(record.has_bearing);
  EXPECT_NEAR(record.bearing_az_deg, 90.0, 1.0e-4);
  EXPECT_NEAR(record.bearing_el_deg, std::asin(los_x / los_norm) * 180.0 / kPi, 1.0e-4);
  EXPECT_TRUE(record.has_sensor_origin);
  oneq::coordinate::LlaPositionDegM expected_origin;
  ASSERT_TRUE(oneq::coordinate::TryEcefToLla(satellite, &expected_origin));
  EXPECT_DOUBLE_EQ(record.sensor_origin.latitude_deg, expected_origin.latitude_deg);
  EXPECT_DOUBLE_EQ(record.sensor_origin.longitude_deg, expected_origin.longitude_deg);
  EXPECT_DOUBLE_EQ(record.sensor_origin.altitude_m, expected_origin.altitude_m);
  EXPECT_NEAR(record.quality, 0.5, 1.0e-9);  // SNR 500 / 1000

  // 非执行周期（关机/待机）显式空返回，不产出量测。
  result.status = sbirs_sensor::session::SbirsCycleStatus::kPoweredOff;
  EXPECT_TRUE(precision_evaluation::internal::AdaptSbirsResultToDetectionRecords(result, satellite,
                                                                                0.0, 7U)
                  .empty());
}

}  // namespace
