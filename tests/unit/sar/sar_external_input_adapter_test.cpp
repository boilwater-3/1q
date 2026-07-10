#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/sar/config/SarMissionConfig.h"
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

SarExternalPulseInput MakeLlaPulse(double lat_deg, double lon_deg, double alt_m) {
  SarExternalPulseInput input;
  input.pulse_id = 1U;
  input.time_s = 0.0;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  input.kinematics.position_lla_deg_m.latitude_deg = lat_deg;
  input.kinematics.position_lla_deg_m.longitude_deg = lon_deg;
  input.kinematics.position_lla_deg_m.altitude_m = alt_m;
  // 速度设零。
  return input;
}

TEST(SarExternalInputAdapterTest, PulseAtSceneCenterYieldsZeroOffset) {
  const auto mission = MakeMissionAtOrigin();
  const auto reference = BuildSceneCenterReference(mission);

  SarExternalPulseInput input = MakeLlaPulse(0.0, 0.0, 0.0);
  SarRawIqFrame::PulseState pulse;
  SarCoordinateStatus status;
  ASSERT_TRUE(TryMakePulseFromExternalKinematics(input, reference, &pulse, &status));
  EXPECT_EQ(status, SarCoordinateStatus::kOk);

  EXPECT_NEAR(pulse.position_x_m, 0.0, 1.0e-3);
  EXPECT_NEAR(pulse.position_y_m, 0.0, 1.0e-3);
  EXPECT_NEAR(pulse.position_z_m, 0.0, 1.0e-3);
}

TEST(SarExternalInputAdapterTest, EastwardLlaProducesPositiveEastComponent) {
  const auto mission = MakeMissionAtOrigin();
  const auto reference = BuildSceneCenterReference(mission);

  // 在赤道，正东偏移 0.001° 经度 ≈ 111.32 m。
  SarExternalPulseInput input = MakeLlaPulse(0.0, 0.001, 0.0);
  SarRawIqFrame::PulseState pulse;
  ASSERT_TRUE(TryMakePulseFromExternalKinematics(input, reference, &pulse, nullptr));

  EXPECT_GT(pulse.position_x_m, 100.0);  // East 分量显著为正
  EXPECT_NEAR(pulse.position_y_m, 0.0, 1.0);  // North 接近零
}

TEST(SarExternalInputAdapterTest, NorthwardLlaProducesPositiveNorthComponent) {
  const auto mission = MakeMissionAtOrigin();
  const auto reference = BuildSceneCenterReference(mission);

  // 正北偏移 0.001° 纬度 ≈ 111.32 m。
  SarExternalPulseInput input = MakeLlaPulse(0.001, 0.0, 0.0);
  SarRawIqFrame::PulseState pulse;
  ASSERT_TRUE(TryMakePulseFromExternalKinematics(input, reference, &pulse, nullptr));

  EXPECT_GT(pulse.position_y_m, 100.0);  // North 分量显著为正
  EXPECT_NEAR(pulse.position_x_m, 0.0, 1.0);  // East 接近零
}

TEST(SarExternalInputAdapterTest, EcefInputMatchesEquivalentLla) {
  const auto mission = MakeMissionAtOrigin();
  const auto reference = BuildSceneCenterReference(mission);

  // 同一个 LLA 点，分别用 kEcef 和 kLla 两种方式输入，结果应一致。
  const double lat = 0.0005;
  const double lon = 0.0005;
  const double alt = 0.0;

  oneq::coordinate::LlaPositionDegM lla;
  lla.latitude_deg = lat;
  lla.longitude_deg = lon;
  lla.altitude_m = alt;
  oneq::coordinate::EcefPositionM ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(lla, &ecef));

  SarExternalPulseInput lla_input = MakeLlaPulse(lat, lon, alt);
  SarExternalPulseInput ecef_input;
  ecef_input.pulse_id = 1U;
  ecef_input.time_s = 0.0;
  ecef_input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  ecef_input.kinematics.position_ecef_m = ecef;

  SarRawIqFrame::PulseState lla_pulse;
  SarRawIqFrame::PulseState ecef_pulse;
  ASSERT_TRUE(TryMakePulseFromExternalKinematics(lla_input, reference, &lla_pulse, nullptr));
  ASSERT_TRUE(TryMakePulseFromExternalKinematics(ecef_input, reference, &ecef_pulse, nullptr));

  EXPECT_NEAR(lla_pulse.position_x_m, ecef_pulse.position_x_m, 1.0e-3);
  EXPECT_NEAR(lla_pulse.position_y_m, ecef_pulse.position_y_m, 1.0e-3);
  EXPECT_NEAR(lla_pulse.position_z_m, ecef_pulse.position_z_m, 1.0e-3);
}

TEST(SarExternalInputAdapterTest, NullOutputYieldsNullStatus) {
  const auto mission = MakeMissionAtOrigin();
  const auto reference = BuildSceneCenterReference(mission);
  SarExternalPulseInput input = MakeLlaPulse(0.0, 0.0, 0.0);

  SarCoordinateStatus status;
  EXPECT_FALSE(TryMakePulseFromExternalKinematics(input, reference, nullptr, &status));
  EXPECT_EQ(status, SarCoordinateStatus::kNullOutput);
}

TEST(SarExternalInputAdapterTest, NonFiniteEcefPositionYieldsTransformFail) {
  const auto mission = MakeMissionAtOrigin();
  const auto reference = BuildSceneCenterReference(mission);

  SarExternalPulseInput input;
  input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  input.kinematics.position_ecef_m.x_m = std::numeric_limits<double>::quiet_NaN();
  input.kinematics.position_ecef_m.y_m = 0.0;
  input.kinematics.position_ecef_m.z_m = 0.0;

  SarRawIqFrame::PulseState pulse;
  SarCoordinateStatus status;
  EXPECT_FALSE(TryMakePulseFromExternalKinematics(input, reference, &pulse, &status));
  EXPECT_EQ(status, SarCoordinateStatus::kCoordinateTransformFail);
}

TEST(SarExternalInputAdapterTest, InvalidLlaLatitudeYieldsTransformFail) {
  const auto mission = MakeMissionAtOrigin();
  const auto reference = BuildSceneCenterReference(mission);

  // 纬度超出 [-90, 90] 范围，TryLlaToEnu 应失败。
  SarExternalPulseInput input = MakeLlaPulse(999.0, 0.0, 0.0);

  SarRawIqFrame::PulseState pulse;
  SarCoordinateStatus status;
  EXPECT_FALSE(TryMakePulseFromExternalKinematics(input, reference, &pulse, &status));
  EXPECT_EQ(status, SarCoordinateStatus::kCoordinateTransformFail);
}

TEST(SarExternalInputAdapterTest, BuildSceneCenterReferenceCarriesOrigin) {
  config::SarMissionConfig mission;
  mission.scene_center_latitude_deg = 35.0;
  mission.scene_center_longitude_deg = 120.0;
  mission.scene_center_altitude_m = 500.0;

  const auto reference = BuildSceneCenterReference(mission);
  EXPECT_DOUBLE_EQ(reference.origin_lla.latitude_deg, 35.0);
  EXPECT_DOUBLE_EQ(reference.origin_lla.longitude_deg, 120.0);
  EXPECT_DOUBLE_EQ(reference.origin_lla.altitude_m, 500.0);
}

}  // namespace
}  // namespace session
}  // namespace sar
