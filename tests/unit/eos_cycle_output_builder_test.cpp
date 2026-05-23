/**
 * @file eos_cycle_output_builder_test.cpp
 * @brief 验证 EosCycleOutputBuilder 将内部 EOS 输出转换回外部 ECEF 输出。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleInputBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleOutputBuilder.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosSessionFactory.h"

namespace {

namespace eos_config = ::electro_optical_sensor::config;
namespace eos_session = ::electro_optical_sensor::session;

eos_session::EosExternalPoseInput MakePlatformInput() {
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1200.0;
  oneq::coordinate::EcefPositionM platform_ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  eos_session::EosExternalPoseInput platform;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  return platform;
}

std::vector<eos_session::EosExternalTargetInput> MakeMovingTargets(std::size_t count) {
  std::vector<eos_session::EosExternalTargetInput> targets;
  targets.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    oneq::coordinate::LlaPositionDegM lla;
    lla.latitude_deg = 31.0;
    lla.longitude_deg = 121.001 + static_cast<double>(i) * 0.00005;
    lla.altitude_m = 1200.0;
    oneq::coordinate::EcefPositionM ecef;
    EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(lla, &ecef));

    eos_session::EosExternalTargetInput target;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = ecef;
    target.appearance.apparent_temperature_k = 600.0f;
    target.appearance.emissivity = 0.95f;
    target.appearance.reflectance = 0.4f;
    target.appearance.projected_area_m2 = 12.0f;
    targets.push_back(target);
  }
  return targets;
}

void AdvanceTargets(double dt_sec, std::vector<eos_session::EosExternalTargetInput>* targets) {
  ASSERT_NE(targets, nullptr);
  for (std::size_t i = 0; i < targets->size(); ++i) {
    eos_session::EosExternalTargetInput& target = (*targets)[i];
    target.kinematics.position_ecef_m.x_m += (4.0 + static_cast<double>(i % 3U)) * dt_sec;
    target.kinematics.position_ecef_m.y_m += (-2.0 + static_cast<double>(i % 2U)) * dt_sec;
    target.kinematics.position_ecef_m.z_m += 0.1 * static_cast<double>(i % 2U) * dt_sec;
  }
}

eos_session::EosSessionConfig MakeConfig() {
  eos_session::EosSessionConfig config =
      eos_config::EosSessionConfigBuilder()
          .Mission()
          .WithWorkMode(eos_config::EosWorkMode::kFused)
          .WithScanRateDegPerSec(1.0f)
          .End()
          .Detection()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .End()
          .Build();
  config.mission.scan_start_az_deg = -90.0f;
  config.mission.scan_end_az_deg = 90.0f;
  config.mission.horizontal_fov_deg = 180.0f;
  config.mission.vertical_fov_deg = 90.0f;
  return config;
}

}  // namespace

TEST(EosCycleOutputBuilderTest, MultiCycleMovingTargetsStayNearExternalTruth) {
  const eos_session::EosExternalPoseInput platform = MakePlatformInput();
  std::vector<eos_session::EosExternalTargetInput> targets = MakeMovingTargets(10U);
  eos_session::EosSession session = eos_session::EosSessionFactory::Create(MakeConfig());

  const std::size_t cycle_count = 36U;
  const float dt_sec = 0.5f;
  std::size_t compared_detection_count = 0U;
  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    eos_session::EosCycleInput input;
    ASSERT_TRUE(eos_session::EosCycleInputBuilder::Build(platform, targets, dt_sec, &input))
        << "cycle=" << cycle;
    input.cycle_index = static_cast<std::uint32_t>(cycle);
    input.environment.solar_irradiance_w_m2 = 900.0f;
    input.environment.solar_altitude_deg = 45.0f;
    input.environment.cloud_coverage_ratio = 0.05f;
    input.environment.background_temperature_k = 285.0f;
    input.environment.day_night_type = eos_session::DayNightType::kDay;

    const eos_session::EosCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(result.has_validation_error) << "cycle=" << cycle;

    eos_session::EosExternalOutputFrame external_frame;
    ASSERT_TRUE(
        eos_session::EosCycleOutputBuilder::Build(platform, result.output_frame, &external_frame))
        << "cycle=" << cycle;

    for (std::size_t detection_index = 0; detection_index < external_frame.detections.size();
         ++detection_index) {
      const eos_session::EosExternalDetectionRecord& detection =
          external_frame.detections[detection_index];
      ASSERT_LT(detection.target_id, targets.size()) << "cycle=" << cycle;
      const std::size_t target_index = static_cast<std::size_t>(detection.target_id);
      const oneq::coordinate::EcefPositionM& truth = targets[target_index].kinematics.position_ecef_m;
      EXPECT_NEAR(detection.target_position_ecef_m.x_m, truth.x_m, 10.0)
          << "cycle=" << cycle << " target=" << target_index;
      EXPECT_NEAR(detection.target_position_ecef_m.y_m, truth.y_m, 10.0)
          << "cycle=" << cycle << " target=" << target_index;
      EXPECT_NEAR(detection.target_position_ecef_m.z_m, truth.z_m, 10.0)
          << "cycle=" << cycle << " target=" << target_index;
      ++compared_detection_count;
    }

    AdvanceTargets(dt_sec, &targets);
  }
  EXPECT_GT(compared_detection_count, 40U);
}
