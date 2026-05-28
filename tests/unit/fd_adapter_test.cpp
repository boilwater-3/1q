#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "fd_test_helpers.h"

namespace oneq {
namespace flight_dynamic {
namespace {

class FlightDynamicTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.aircraft_model = "c172x";
    config_.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    config_.dt_sec = kDt;
    config_.do_trim = true;
    config_.silent_mode = true;
    config_.initial_kinematics.position_frame = coordinate::PositionFrame::kLla;
    config_.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.altitude_m = 500.0;
    config_.initial_kinematics.velocity_mps.x_mps = 50.0;
    config_.initial_kinematics.velocity_mps.y_mps = 0.0;
    config_.initial_kinematics.velocity_mps.z_mps = 0.0;
    config_.initial_kinematics.attitude_deg.roll_deg = 0.0;
    config_.initial_kinematics.attitude_deg.pitch_deg = 0.0;
    config_.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  }

  config::FlightDynamicConfig config_;
};

TEST_F(FlightDynamicTest, AdapterCreatesAndRuns) {
  FlightManager fm(config_);
  EXPECT_EQ(fm.GetState(), FlightManagerState::kReady);

  bool result = fm.Step(kDt);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(fm.GetVehicleState().sim_time_sec, kDt);
}

TEST_F(FlightDynamicTest, VehicleStatePopulated) {
  FlightManager fm(config_);
  RunSteps(fm, 100);

  const auto& state = fm.GetVehicleState();
  EXPECT_GT(state.sim_time_sec, 0.0);
  EXPECT_GT(state.mass_kg, 0.0);
  ExpectNoNaN(state);
}

TEST_F(FlightDynamicTest, AutopilotDetectsOwnApProfile) {
  FlightManager fm(config_);
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  EXPECT_EQ(profile.lateral_interface, autopilot::LateralControlInterface::kOwnAutopilot);
  EXPECT_TRUE(profile.has_own_autopilot);
  EXPECT_TRUE(profile.has_aileron_command);
  EXPECT_EQ(profile.engine_count, 1);
}

TEST_F(FlightDynamicTest, AutopilotDetectsFbwProfile) {
  config_.aircraft_model = "f16";
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 3000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 200.0;

  FlightManager fm(config_);
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  EXPECT_EQ(profile.lateral_interface, autopilot::LateralControlInterface::kFbwRateCommand);
  EXPECT_FALSE(profile.has_own_autopilot);
  EXPECT_TRUE(profile.has_generic_autopilot);
  EXPECT_TRUE(profile.has_fbw_override);
  EXPECT_TRUE(profile.has_roll_rate_command);
  EXPECT_TRUE(profile.has_aileron_command);
  EXPECT_EQ(profile.engine_count, 1);
}

TEST_F(FlightDynamicTest, AutopilotSetsHeading) {
  FlightManager fm(config_);
  auto& ap = fm.GetAutopilot();

  ap.SetHeadingTargetRad(1.57);
  ap.SetHeadingHold(true);
  ap.SetRollAttitudeMode(1);
  ap.SetRollAutopilotOn(true);

  RunSteps(fm, 200);
}

TEST_F(FlightDynamicTest, AutopilotSetsAltitude) {
  FlightManager fm(config_);
  auto& ap = fm.GetAutopilot();

  ap.SetAltitudeTargetM(12000.0);
  ap.SetAltitudeHold(true);

  RunSteps(fm, 200);
}

TEST_F(FlightDynamicTest, WaypointManagerAddsWaypoints) {
  FlightManager fm(config_);
  auto& wpm = fm.GetWaypointManager();

  guidance::Waypoint wp;
  wp.latitude_rad = 0.1;
  wp.longitude_rad = 0.2;
  wp.altitude_m = 10000.0;
  wpm.AddWaypoint(wp);

  wpm.Start();
  EXPECT_EQ(wpm.GetActiveIndex(), 0U);
  EXPECT_EQ(wpm.GetWaypointCount(), 1U);
}

TEST_F(FlightDynamicTest, FlyToWaypointManeuver) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = 0.01;
  cmd.target.longitude_rad = 0.01;
  cmd.target.altitude_m = 10000.0;

  fm.PushManeuver(cmd);

  fm.Step(kDt);
  double lat_before = fm.GetVehicleState().latitude_rad;
  double lon_before = fm.GetVehicleState().longitude_rad;

  RunSteps(fm, 999);

  double lat_after = fm.GetVehicleState().latitude_rad;
  double lon_after = fm.GetVehicleState().longitude_rad;

  EXPECT_TRUE(fm.GetState() == FlightManagerState::kExecuting ||
              fm.GetState() == FlightManagerState::kCompleted);

  double pos_change = std::hypot(lat_after - lat_before, lon_after - lon_before);
  EXPECT_GT(pos_change, 0.0) << "Aircraft position should change during FlyToWaypoint";
}

TEST_F(FlightDynamicTest, FlyToWaypointNearbyCompletesManeuver) {
  FlightManager fm(config_);

  constexpr double kNearbyLat = 0.0 + (100.0 / 6.371e6);
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = kNearbyLat;
  cmd.target.longitude_rad = 0.0;
  cmd.target.altitude_m = 500.0;

  fm.PushManeuver(cmd);

  RunUntilDone(fm, 8000);

  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted)
      << "FlyToWaypoint to a nearby target should reach kCompleted";
}

TEST_F(FlightDynamicTest, MultipleManeuvers) {
  FlightManager fm(config_);

  ManeuverCommand fly_cmd;
  fly_cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  fly_cmd.target.latitude_rad = 0.01;
  fly_cmd.target.longitude_rad = 0.01;

  ManeuverCommand heading_cmd;
  heading_cmd.type = guidance::ManeuverType::kSetHeading;
  heading_cmd.value = 3.14;

  fm.PushManeuver(fly_cmd);
  fm.PushManeuver(heading_cmd);

  RunSteps(fm, 500);
}

TEST_F(FlightDynamicTest, ResetAndReuse) {
  FlightManager fm(config_);

  RunSteps(fm, 100);
  EXPECT_GT(fm.GetVehicleState().sim_time_sec, 0.0);

  config_.initial_kinematics.position_lla_deg_m.latitude_deg = 10.0;
  fm.Reset(config_);
  EXPECT_EQ(fm.GetState(), FlightManagerState::kReady);

  RunSteps(fm, 100);
}

TEST_F(FlightDynamicTest, OrbitManeuver) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = 0.01;
  cmd.target.longitude_rad = 0.01;
  cmd.target.altitude_m = 1000.0;
  cmd.value = 500.0;

  fm.PushManeuver(cmd);

  fm.Step(kDt);
  double lat_before = fm.GetVehicleState().latitude_rad;
  double lon_before = fm.GetVehicleState().longitude_rad;

  RunSteps(fm, 499);

  EXPECT_TRUE(fm.GetState() == FlightManagerState::kExecuting ||
              fm.GetState() == FlightManagerState::kCompleted);

  double lat_after = fm.GetVehicleState().latitude_rad;
  double lon_after = fm.GetVehicleState().longitude_rad;
  double lateral = std::hypot(lat_after - lat_before, lon_after - lon_before);
  EXPECT_GT(lateral, 0.0) << "Aircraft should move during orbit";
}

TEST_F(FlightDynamicTest, OrbitRunsWithoutError) {
  FlightManager fm(config_);

  constexpr double kNearbyLat = 0.0 + (100.0 / 6.371e6);
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = kNearbyLat;
  cmd.target.longitude_rad = 0.0;
  cmd.target.altitude_m = 500.0;
  cmd.value = 50.0;

  fm.PushManeuver(cmd);
  RunSteps(fm, 2000);

  auto state = fm.GetState();
  EXPECT_EQ(state, FlightManagerState::kExecuting)
      << "Orbit should keep executing until another command or abort";
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_F(FlightDynamicTest, SetPitchManeuver) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetPitch;
  cmd.value = 5.0;
  cmd.duration_sec = 2.0;

  fm.PushManeuver(cmd);

  RunUntilDone(fm, 500);

  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted);

  constexpr double kTargetPitchRad = 5.0 * kPi / 180.0;
  double pitch_rad = fm.GetVehicleState().theta_rad;
  EXPECT_NEAR(pitch_rad, kTargetPitchRad, 0.35)
      << "Pitch should be near 5 deg target, got " << pitch_rad * 180.0 / kPi << " deg";
}

TEST_F(FlightDynamicTest, SetRollManeuver) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetRoll;
  cmd.value = 1.0;

  fm.PushManeuver(cmd);

  fm.Step(kDt);

  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted);
}

TEST_F(FlightDynamicTest, SetAltitudeManeuver) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetAltitude;
  cmd.value = 600.0;

  fm.PushManeuver(cmd);

  fm.Step(kDt);
  double alt_before = fm.GetVehicleState().altitude_geod_m;

  RunSteps(fm, 999);

  double alt_after = fm.GetVehicleState().altitude_geod_m;
  EXPECT_TRUE(fm.GetState() == FlightManagerState::kExecuting ||
              fm.GetState() == FlightManagerState::kCompleted);
  EXPECT_GT(alt_after, alt_before) << "Altitude should increase when targeting 600m from ~500m";
}

TEST_F(FlightDynamicTest, AbortManeuver) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = 0.5;

  fm.PushManeuver(cmd);
  fm.Step(kDt);

  fm.Abort();
  EXPECT_EQ(fm.GetState(), FlightManagerState::kAborted);

  bool result = fm.Step(kDt);
  EXPECT_FALSE(result);
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
