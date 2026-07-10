#include <gtest/gtest.h>

#include "1q/sar/config/SarMissionConfig.h"
#include "1q/sar/session/SarCycleInputAdapter.h"
#include "1q/sar/session/SarExternalInputAdapter.h"

namespace sar {
namespace session {
namespace {

config::SarMissionConfig MakeMissionAtOrigin() {
  config::SarMissionConfig mission;
  mission.scene_center_latitude_deg = 0.0;
  mission.scene_center_longitude_deg = 0.0;
  mission.scene_center_altitude_m = 0.0;
  return mission;
}

SarPlatformState MakePlatformAtOrigin() {
  SarPlatformState platform;
  platform.latitude_deg = 0.0;
  platform.longitude_deg = 0.0;
  platform.altitude_m = 8000.0;
  platform.velocity_north_mps = 0.0;
  platform.velocity_east_mps = 180.0;
  platform.velocity_down_mps = 0.0;
  return platform;
}

TEST(SarCycleInputAdapterTest, EmptyPulsesProducesNoRawIq) {
  const auto mission = MakeMissionAtOrigin();
  const auto platform = MakePlatformAtOrigin();
  const SarPointTargetList targets;

  SarCycleInput output;
  SarCoordinateStatus status;
  ASSERT_TRUE(SarCycleInputAdapter::Build(platform, targets, mission, 1.0f, {}, &output, &status));
  EXPECT_EQ(status, SarCoordinateStatus::kOk);

  // 平台/目标透传。
  EXPECT_DOUBLE_EQ(output.platform.altitude_m, 8000.0);
  EXPECT_DOUBLE_EQ(output.platform.velocity_east_mps, 180.0);
  EXPECT_TRUE(output.point_targets.empty());
  // 无外部脉冲，raw_iq 保持空。
  EXPECT_TRUE(output.raw_iq.pulse_states.empty());
  EXPECT_EQ(output.raw_iq.pulse_count, 0U);
}

TEST(SarCycleInputAdapterTest, PulsesConvertedToSceneCenterEnu) {
  const auto mission = MakeMissionAtOrigin();
  const auto platform = MakePlatformAtOrigin();

  // 构造两个连续脉冲，位置在 scene_center 正东方向。
  std::vector<SarExternalPulseInput> pulses;
  for (std::uint64_t i = 0; i < 2; ++i) {
    SarExternalPulseInput pulse;
    pulse.pulse_id = i + 1U;
    pulse.time_s = static_cast<double>(i);
    pulse.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
    pulse.kinematics.position_lla_deg_m.latitude_deg = 0.0;
    pulse.kinematics.position_lla_deg_m.longitude_deg = 0.001;
    pulse.kinematics.position_lla_deg_m.altitude_m = 8000.0;
    pulses.push_back(pulse);
  }

  SarCycleInput output;
  ASSERT_TRUE(SarCycleInputAdapter::Build(platform, {}, mission, 1.0f, pulses, &output, nullptr));

  EXPECT_EQ(output.raw_iq.pulse_count, 2U);
  ASSERT_EQ(output.raw_iq.pulse_states.size(), 2U);

  // 两个脉冲位置相同，East 分量应都为正且相等。
  EXPECT_GT(output.raw_iq.pulse_states[0].position_x_m, 100.0);
  EXPECT_NEAR(output.raw_iq.pulse_states[0].position_x_m,
              output.raw_iq.pulse_states[1].position_x_m, 1.0e-3);
  // 脉冲序号连续。
  EXPECT_EQ(output.raw_iq.pulse_states[1].pulse_id, 2U);
}

TEST(SarCycleInputAdapterTest, NullOutputFailsWithNullStatus) {
  const auto mission = MakeMissionAtOrigin();
  const auto platform = MakePlatformAtOrigin();

  SarCoordinateStatus status;
  EXPECT_FALSE(SarCycleInputAdapter::Build(platform, {}, mission, 1.0f, {}, nullptr, &status));
  EXPECT_EQ(status, SarCoordinateStatus::kNullOutput);
}

TEST(SarCycleInputAdapterTest, FailingPulseStopsBuild) {
  const auto mission = MakeMissionAtOrigin();
  const auto platform = MakePlatformAtOrigin();

  // 第一个脉冲合法，第二个脉冲纬度非法。
  std::vector<SarExternalPulseInput> pulses;
  SarExternalPulseInput good;
  good.pulse_id = 1U;
  good.time_s = 0.0;
  good.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  good.kinematics.position_lla_deg_m.latitude_deg = 0.0;
  pulses.push_back(good);

  SarExternalPulseInput bad;
  bad.pulse_id = 2U;
  bad.time_s = 1.0;
  bad.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  bad.kinematics.position_lla_deg_m.latitude_deg = 999.0;  // 非法
  pulses.push_back(bad);

  SarCycleInput output;
  SarCoordinateStatus status;
  EXPECT_FALSE(SarCycleInputAdapter::Build(platform, {}, mission, 1.0f, pulses, &output, &status));
  EXPECT_EQ(status, SarCoordinateStatus::kCoordinateTransformFail);
}

}  // namespace
}  // namespace session
}  // namespace sar
