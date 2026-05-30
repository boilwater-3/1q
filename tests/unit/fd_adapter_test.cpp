#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "fd_test_helpers.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"

namespace oneq {
namespace flight_dynamic {
namespace {

constexpr double kMToFt = 1.0 / 0.3048;

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

TEST_F(FlightDynamicTest, InitialVelocityDefaultsToBodyFrame) {
  config_.do_trim = false;
  config_.initial_kinematics.velocity_mps.x_mps = 50.0;
  config_.initial_kinematics.velocity_mps.y_mps = 3.0;
  config_.initial_kinematics.velocity_mps.z_mps = -2.0;

  adapter::JsbsimAdapter adapter(config_);

  EXPECT_NEAR(adapter.GetProperty("velocities/u-fps"), 50.0 * kMToFt, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("velocities/v-fps"), 3.0 * kMToFt, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("velocities/w-fps"), -2.0 * kMToFt, 1.0e-6);
}

TEST_F(FlightDynamicTest, InitialVelocityCanUseEcefFrame) {
  config_.do_trim = false;
  config_.initial_velocity_frame = config::InitialVelocityFrame::kEcef;
  config_.initial_kinematics.velocity_mps.x_mps = 3.0;  // ECEF X -> ENU Up -> NED Down -3
  config_.initial_kinematics.velocity_mps.y_mps = 1.0;  // ECEF Y -> ENU East
  config_.initial_kinematics.velocity_mps.z_mps = 2.0;  // ECEF Z -> ENU North

  adapter::JsbsimAdapter adapter(config_);

  EXPECT_NEAR(adapter.GetProperty("velocities/v-north-fps"), 2.0 * kMToFt, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("velocities/v-east-fps"), 1.0 * kMToFt, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("velocities/v-down-fps"), -3.0 * kMToFt, 1.0e-6);
}

TEST_F(FlightDynamicTest, InitialConditionsAcceptEcefPosition) {
  config_.do_trim = false;
  config_.initial_velocity_frame = config::InitialVelocityFrame::kEcef;
  config_.initial_kinematics.position_frame = coordinate::PositionFrame::kEcef;
  ASSERT_TRUE(coordinate::TryLlaToEcef(config_.initial_kinematics.position_lla_deg_m,
                                       &config_.initial_kinematics.position_ecef_m));
  config_.initial_kinematics.velocity_mps.x_mps = 3.0;
  config_.initial_kinematics.velocity_mps.y_mps = 1.0;
  config_.initial_kinematics.velocity_mps.z_mps = 2.0;

  adapter::JsbsimAdapter adapter(config_);

  EXPECT_NEAR(adapter.GetProperty("position/lat-gc-deg"), 0.0, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("position/long-gc-deg"), 0.0, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("position/h-sl-ft"), 500.0 * kMToFt, 1.0e-3);
  EXPECT_NEAR(adapter.GetProperty("velocities/v-north-fps"), 2.0 * kMToFt, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("velocities/v-east-fps"), 1.0 * kMToFt, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty("velocities/v-down-fps"), -3.0 * kMToFt, 1.0e-6);
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
  EXPECT_EQ(profile.fbw_subtype, autopilot::FbwSubtype::kRollRatePid);
}

TEST_F(FlightDynamicTest, AutopilotDetectsF22Profile) {
  config_.aircraft_model = "f22";
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 3000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 200.0;

  FlightManager fm(config_);
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  // f22 has project-injected Autopilot system which provides ap/heading_hold,
  // so it's detected as kOwnAutopilot, but FBW subtype is integrator+actuator.
  EXPECT_EQ(profile.fbw_subtype, autopilot::FbwSubtype::kRateIntegratorActuator);
  EXPECT_EQ(profile.engine_count, 2);
  EXPECT_TRUE(profile.indexed_throttle);
}

TEST_F(FlightDynamicTest, AutopilotDetectsC310Profile) {
  config_.aircraft_model = "c310";
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 500.0;
  config_.initial_kinematics.velocity_mps.x_mps = 65.0;

  FlightManager fm(config_);
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  // c310 has project-injected Autopilot system providing ap/heading_hold.
  EXPECT_EQ(profile.engine_count, 2);
  EXPECT_FALSE(profile.indexed_throttle);
  EXPECT_FALSE(profile.yaw_input_property.empty());
}

TEST_F(FlightDynamicTest, AutopilotDetectsConcordeProfile) {
  config_.aircraft_model = "Concorde";
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 5000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 150.0;

  FlightManager fm(config_);
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  // Concorde has project-injected AP system, detected as generic AP bridge.
  EXPECT_EQ(profile.engine_count, 4);
  EXPECT_FALSE(profile.indexed_throttle);
  EXPECT_EQ(profile.yaw_input_property, "fcs/rudder-cmd-norm");
}

TEST_F(FlightDynamicTest, AutopilotWritesIndexedThrottleForMultiEngineProfiles) {
  config_.aircraft_model = "f22";
  config_.do_trim = false;
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 3000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 200.0;

  adapter::JsbsimAdapter adapter(config_);
  autopilot::Autopilot ap(adapter);
  ASSERT_TRUE(ap.GetControlProfile().indexed_throttle);
  ASSERT_EQ(ap.GetControlProfile().engine_count, 2);

  ap.SetThrottleCmdNorm(0.61);

  EXPECT_NEAR(adapter.GetProperty("fcs/throttle-cmd-norm"), 0.61, 1.0e-9);
  EXPECT_NEAR(adapter.GetProperty("fcs/throttle-cmd-norm[0]"), 0.61, 1.0e-9);
  EXPECT_NEAR(adapter.GetProperty("fcs/throttle-cmd-norm[1]"), 0.61, 1.0e-9);
}

TEST_F(FlightDynamicTest, AutopilotWritesDetectedYawInputProperty) {
  config_.aircraft_model = "Concorde";
  config_.do_trim = false;
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 5000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 150.0;

  adapter::JsbsimAdapter adapter(config_);
  autopilot::Autopilot ap(adapter);
  ASSERT_EQ(ap.GetControlProfile().yaw_input_property, "fcs/rudder-cmd-norm");

  adapter.SetProperty("fcs/rudder-pedal-norm", 0.42);
  adapter.SetProperty("fcs/rudder-cmd-norm", 0.42);

  ap.Update(kDt);

  EXPECT_NEAR(adapter.GetProperty("fcs/rudder-cmd-norm"), 0.0, 1.0e-9);
  EXPECT_NEAR(adapter.GetProperty("fcs/rudder-pedal-norm"), 0.42, 1.0e-9);
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
