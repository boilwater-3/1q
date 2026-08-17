/**
 * @file target_inference_engine_test.cpp
 * @brief 验证 target_inference 推演引擎：弹道守恒、发射/落点一致性、误差预算出口、
 *        类型融合与确定性。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/InferenceTrackState.h"
#include "1q/target_inference/TargetInferenceConfig.h"
#include "1q/target_inference/TargetInferenceEngine.h"

namespace target_inference {
namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kEarthMu = 3.986004418e14;

/** @brief 构造弹道中段状态：赤道上方 150 km，水平速度 1.5 km/s（无阻力弹道弧）。 */
InferenceTrackState MakeBallisticMidcourse() {
  InferenceTrackState track;
  track.key = 42U;
  track.position = oneq::coordinate::EcefPositionM(kEarthRadiusM + 150000.0, 0.0, 0.0);
  track.velocity_ecef_m_per_s = {0.0, 1500.0, 0.0};
  return track;
}

InferenceTrackState WithCovariance(InferenceTrackState track, double position_sigma_m,
                                   double velocity_sigma_m_per_s) {
  track.has_covariance = true;
  for (int i = 0; i < 6; ++i) {
    const auto j = static_cast<std::size_t>(i);
    track.covariance_ecef[j * 6U + j] =
        (i % 2 == 0) ? position_sigma_m * position_sigma_m
                     : velocity_sigma_m_per_s * velocity_sigma_m_per_s;
  }
  return track;
}

TEST(TargetInferenceEngineTest, BallisticPredictionConservesEnergy) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  const auto results = engine.Infer({MakeBallisticMidcourse()});
  ASSERT_EQ(results.size(), 1U);
  ASSERT_TRUE(results.front().trajectory.valid);
  EXPECT_FALSE(results.front().trajectory.waypoints.empty());
  // 无阻力弹道：能量守恒（<0.1% 漂移）；近远地点输入 → 弧线高度有界（积分器门）。
  const double initial_energy = 0.5 * 1500.0 * 1500.0 - kEarthMu / (kEarthRadiusM + 150000.0);
  for (const auto& waypoint : results.front().trajectory.waypoints) {
    oneq::coordinate::EcefPositionM ecef{};
    ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(waypoint.position, &ecef));
    const double radius = std::sqrt(ecef.x_m * ecef.x_m + ecef.y_m * ecef.y_m +
                                    ecef.z_m * ecef.z_m);
    const double potential = -kEarthMu / radius;
    const double speed_squared = 2.0 * (initial_energy - potential);
    EXPECT_GT(speed_squared, 0.0) << "speed must stay real along the arc";
    EXPECT_LE(waypoint.position.altitude_m, 152000.0)
        << "near-apogee input must bound the arc altitude";
  }
}

TEST(TargetInferenceEngineTest, LaunchBacktrackReachesGroundAndImpactResolves) {
  // 150 km + 1.5 km/s 水平的椭圆弹道落 地表约需 600+ s：测试时域放宽到 900 s。
  TargetInferenceConfig config;
  config.prediction_horizon_sec = 900.0;
  TargetInferenceEngine engine(config);
  const auto results = engine.Infer({MakeBallisticMidcourse()});
  const TrajectoryPrediction& trajectory = results.front().trajectory;
  ASSERT_TRUE(trajectory.valid);

  ASSERT_TRUE(trajectory.has_launch);
  EXPECT_LT(trajectory.launch_time_offset_sec, 0.0);
  EXPECT_NEAR(trajectory.launch_point.altitude_m, 0.0, 5000.0)
      << "launch point should sit on the surface";

  // 中段水平速度 1.5 km/s @150 km：弧长远大于地平线盲区，预测时域内应出落点。
  EXPECT_TRUE(trajectory.has_impact);
  if (trajectory.has_impact) {
    EXPECT_GT(trajectory.impact_time_offset_sec, 0.0);
    EXPECT_NEAR(trajectory.impact_point.altitude_m, 0.0, 5000.0);
  }
}

TEST(TargetInferenceEngineTest, UncertaintyBudgetScalesWithInputCovariance) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  const auto tight = engine.Infer({WithCovariance(MakeBallisticMidcourse(), 100.0, 1.0)});
  const auto loose = engine.Infer({WithCovariance(MakeBallisticMidcourse(), 10000.0, 10.0)});
  ASSERT_EQ(tight.size(), 1U);
  ASSERT_EQ(loose.size(), 1U);

  ASSERT_TRUE(tight.front().trajectory.has_uncertainty);
  ASSERT_TRUE(loose.front().trajectory.has_uncertainty);
  EXPECT_GT(tight.front().trajectory.launch_position_sigma_m, 0.0);
  EXPECT_GT(loose.front().trajectory.launch_position_sigma_m,
            tight.front().trajectory.launch_position_sigma_m)
      << "launch sigma must grow with input covariance";
  EXPECT_GT(loose.front().trajectory.launch_position_sigma_m, 5000.0)
      << "10km input sigma must produce km-level launch budget";

  // 无协方差输入：显式无预算标记（sigma=0 不冒充零误差）。
  const auto bare = engine.Infer({MakeBallisticMidcourse()});
  ASSERT_TRUE(bare.front().trajectory.has_launch);
  EXPECT_FALSE(bare.front().trajectory.has_uncertainty);
  EXPECT_EQ(bare.front().trajectory.launch_position_sigma_m, 0.0);
}

TEST(TargetInferenceEngineTest, TypeAssessmentCombinesKinematicsAndEvidence) {
  TargetInferenceEngine engine(TargetInferenceConfig{});

  // 高速高空无证据：弹道先验胜出。
  const auto kinematic = engine.Infer({MakeBallisticMidcourse()});
  ASSERT_EQ(kinematic.size(), 1U);
  EXPECT_EQ(kinematic.front().type.category, InferenceTargetCategory::kBallistic);
  EXPECT_GT(kinematic.front().type.probability, 0.3);

  // 同状态 + 导弹强证据：证据主导（w=0.5：导弹 0.5·0+0.5·1=0.5 > 弹道 0.275）。
  InferenceTrackState evidenced = MakeBallisticMidcourse();
  evidenced.has_type_evidence = true;
  evidenced.type_evidence[static_cast<std::size_t>(InferenceTargetCategory::kMissile)] = 1.0;
  const auto fused = engine.Infer({evidenced});
  EXPECT_EQ(fused.front().type.category, InferenceTargetCategory::kMissile);

  // 低速低空无证据：无人机倾向。
  InferenceTrackState slow;
  slow.key = 7U;
  slow.position = oneq::coordinate::EcefPositionM(kEarthRadiusM + 1000.0, 0.0, 0.0);
  slow.velocity_ecef_m_per_s = {0.0, 80.0, 0.0};
  const auto slow_result = engine.Infer({slow});
  EXPECT_EQ(slow_result.front().type.category, InferenceTargetCategory::kUav);
}

TEST(TargetInferenceEngineTest, InvalidInputMarksTrajectoryInvalid) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  InferenceTrackState buried;
  buried.key = 1U;
  buried.position = oneq::coordinate::EcefPositionM(1000.0, 0.0, 0.0);  // 地表之下
  const auto results = engine.Infer({buried});
  ASSERT_EQ(results.size(), 1U);
  EXPECT_FALSE(results.front().trajectory.valid);
  EXPECT_FALSE(results.front().trajectory.has_launch);
}

TEST(TargetInferenceEngineTest, DeterministicAndOrderPreserving) {
  TargetInferenceEngine engine(TargetInferenceConfig{});
  std::vector<InferenceTrackState> tracks;
  tracks.push_back(MakeBallisticMidcourse());
  tracks.push_back(WithCovariance(MakeBallisticMidcourse(), 500.0, 5.0));
  tracks.back().key = 43U;
  tracks.back().position.x_m += 20000.0;

  const auto first = engine.Infer(tracks);
  const auto second = engine.Infer(tracks);
  ASSERT_EQ(first.size(), 2U);
  ASSERT_EQ(second.size(), 2U);
  for (std::size_t i = 0U; i < first.size(); ++i) {
    EXPECT_EQ(first[i].key, tracks[i].key);
    EXPECT_EQ(first[i].key, second[i].key);
    EXPECT_DOUBLE_EQ(first[i].trajectory.launch_point.latitude_deg,
                     second[i].trajectory.launch_point.latitude_deg);
    EXPECT_DOUBLE_EQ(first[i].trajectory.launch_position_sigma_m,
                     second[i].trajectory.launch_position_sigma_m);
    EXPECT_DOUBLE_EQ(first[i].type.probability, second[i].type.probability);
  }
}

}  // namespace
}  // namespace target_inference
