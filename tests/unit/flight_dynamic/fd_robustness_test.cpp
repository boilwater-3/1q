#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "fd_test_helpers.h"

namespace oneq {
namespace flight_dynamic {
namespace {

class FlightDynamicRobustnessTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.aircraft_model = "c172x";
    config_.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    config_.dt_sec = kDt;
    config_.do_trim = true;
    config_.silent_mode = true;
    config_.initial_kinematics.position_frame =
        coordinate::PositionFrame::kLla;
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

// ── Descent ─────────────────────────────────────────────────────────────────

TEST_F(FlightDynamicRobustnessTest, SetAltitudeDescent) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetAltitude;
  cmd.value = 300.0;

  fm.PushManeuver(cmd);

  RunSteps(fm, 1000);

  double alt_after = fm.GetVehicleState().altitude_geod_m;
  EXPECT_GT(alt_after, 0.0) << "Altitude should remain positive";
  EXPECT_LT(alt_after, 5000.0) << "Altitude should remain bounded";
}

// ── Heading boundary ────────────────────────────────────────────────────────

TEST_F(FlightDynamicRobustnessTest, SetHeadingNearNorthBoundary) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetHeading;
  cmd.value = 6.2;

  fm.PushManeuver(cmd);

  RunSteps(fm, 200);
  double err_early =
      std::abs(fm.GetAutopilot().GetAngleToHeadingRad());

  RunSteps(fm, 800);
  double err_late =
      std::abs(fm.GetAutopilot().GetAngleToHeadingRad());

  EXPECT_LT(err_late, err_early * 1.05)
      << "Heading error should not increase near north boundary (early="
      << err_early << " late=" << err_late << ")";
}

// ── Extreme pitch boundary ──────────────────────────────────────────────────

TEST_F(FlightDynamicRobustnessTest, SetPitchExtremeBoundary) {
  FlightManager fm(config_);

  ManeuverCommand up_cmd;
  up_cmd.type = guidance::ManeuverType::kSetPitch;
  up_cmd.value = 15.0;
  up_cmd.duration_sec = 1.0;
  fm.PushManeuver(up_cmd);

  ManeuverCommand down_cmd;
  down_cmd.type = guidance::ManeuverType::kSetPitch;
  down_cmd.value = -15.0;
  down_cmd.duration_sec = 1.0;
  fm.PushManeuver(down_cmd);

  RunSteps(fm, 4000);

  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
      << "Aircraft should not crash during extreme pitch sequence";
}

// ── Long-run numerical stability ────────────────────────────────────────────

TEST_F(FlightDynamicRobustnessTest, LongRunNumericalStability) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetHeading;
  cmd.value = 1.57;
  fm.PushManeuver(cmd);

  RunSteps(fm, 2000);

  const auto& state = fm.GetVehicleState();
  ExpectNoNaN(state);
  EXPECT_GT(state.altitude_geod_m, 0.0) << "Aircraft should not crash";
  EXPECT_LT(state.altitude_geod_m, 20000.0)
      << "Aircraft should not fly to space";
}

// ── Invalid parameters ──────────────────────────────────────────────────────

TEST_F(FlightDynamicRobustnessTest, OrbitNegativeRadiusDoesNotCrash) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = 0.001;
  cmd.target.longitude_rad = 0.0;
  cmd.target.altitude_m = 500.0;
  cmd.value = -100.0;

  fm.PushManeuver(cmd);
  RunSteps(fm, 200);

  ExpectNoNaN(fm.GetVehicleState());
}

TEST_F(FlightDynamicRobustnessTest, SetHeadingLargeValueDoesNotCrash) {
  FlightManager fm(config_);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetHeading;
  cmd.value = 100.0;

  fm.PushManeuver(cmd);
  RunSteps(fm, 200);

  ExpectNoNaN(fm.GetVehicleState());
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
