#include <gtest/gtest.h>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"

namespace oneq {
namespace flight_dynamic {
namespace {

class FlightDynamicTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.aircraft_model = "ball";
    config_.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    config_.dt_sec = 0.005;
    config_.do_trim = false;
    config_.silent_mode = true;
    config_.initial_kinematics.position_frame =
        coordinate::PositionFrame::kLla;
    config_.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.altitude_m = 10000.0;
    config_.initial_kinematics.velocity_mps.x_mps = 100.0;
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

  bool result = fm.Step(0.005);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(fm.GetVehicleState().sim_time_sec, 0.005);
}

TEST_F(FlightDynamicTest, VehicleStatePopulated) {
  FlightManager fm(config_);
  for (int i = 0; i < 100; ++i) {
    fm.Step(0.005);
  }

  const auto& state = fm.GetVehicleState();
  EXPECT_GT(state.sim_time_sec, 0.0);
  EXPECT_TRUE(state.mass_kg > 0.0);
}

TEST_F(FlightDynamicTest, AutopilotSetsHeading) {
  FlightManager fm(config_);
  auto& ap = fm.GetAutopilot();

  ap.SetHeadingTargetRad(1.57);
  ap.SetHeadingHold(true);
  ap.SetRollAttitudeMode(1);
  ap.SetRollAutopilotOn(true);

  for (int i = 0; i < 200; ++i) {
    fm.Step(0.005);
  }
}

TEST_F(FlightDynamicTest, AutopilotSetsAltitude) {
  FlightManager fm(config_);
  auto& ap = fm.GetAutopilot();

  ap.SetAltitudeTargetM(12000.0);
  ap.SetAltitudeHold(true);

  for (int i = 0; i < 200; ++i) {
    fm.Step(0.005);
  }
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

  for (int i = 0; i < 1000; ++i) {
    fm.Step(0.005);
  }

  EXPECT_TRUE(fm.GetState() == FlightManagerState::kExecuting ||
              fm.GetState() == FlightManagerState::kCompleted);
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

  for (int i = 0; i < 500; ++i) {
    fm.Step(0.005);
  }
}

TEST_F(FlightDynamicTest, ResetAndReuse) {
  FlightManager fm(config_);

  for (int i = 0; i < 100; ++i) {
    fm.Step(0.005);
  }

  EXPECT_GT(fm.GetVehicleState().sim_time_sec, 0.0);

  // Reset
  config_.initial_kinematics.position_lla_deg_m.latitude_deg = 10.0;
  fm.Reset(config_);
  EXPECT_EQ(fm.GetState(), FlightManagerState::kReady);

  for (int i = 0; i < 100; ++i) {
    fm.Step(0.005);
  }
}

TEST_F(FlightDynamicTest, AbortManeuver) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = 0.5;

  fm.PushManeuver(cmd);
  fm.Step(0.005);

  fm.Abort();
  EXPECT_EQ(fm.GetState(), FlightManagerState::kAborted);

  bool result = fm.Step(0.005);
  EXPECT_FALSE(result);
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
