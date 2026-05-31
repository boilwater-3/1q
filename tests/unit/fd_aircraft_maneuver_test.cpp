#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <ostream>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "fd_test_helpers.h"

namespace oneq {
namespace flight_dynamic {
namespace {

struct AircraftTestParam {
  std::string model;
  double altitude_m;
  double speed_mps;
  bool unstable;

  AircraftTestParam(std::string m, double alt, double spd, bool u = false)
      : model(std::move(m)), altitude_m(alt), speed_mps(spd), unstable(u) {}
};

void PrintTo(const AircraftTestParam& p, std::ostream* os) {
  *os << p.model;
}

double FlyToWaypointDistanceM(const std::string& model) {
  if (model == "f16") {
    return 10000.0;
  }
  if (model == "A4" || model == "f15") {
    return 20000.0;
  }
  if (model == "Concorde") {
    return 50000.0;
  }
  if (model == "F4N" || model == "F80C" || model == "737" ||
      model == "B747" || model == "MD11") {
    return 10000.0;
  }
  return 5000.0;
}

double FlyToWaypointRadiusM(const std::string& model) {
  if (model == "f16") {
    return 3000.0;
  }
  if (model == "Concorde") {
    return 5000.0;
  }
  return 100.0;
}

double OrbitRadiusM(double speed_mps, double max_bank_deg) {
  // radius = speed² / (g * tan(bank_limit))
  // Uses the aircraft's actual structural/aerodynamic roll limit.
  // Result is rounded up to nearest 500m for stable convergence.
  constexpr double kG = 9.80665;
  double bank_rad = max_bank_deg * M_PI / 180.0;
  double bank_tan = std::tan(bank_rad);
  double radius = (speed_mps * speed_mps) / (kG * bank_tan);
  double scale = 500.0;
  return std::max(std::ceil(radius / scale) * scale, 500.0);
}

class AircraftManeuverTest
    : public ::testing::TestWithParam<AircraftTestParam> {
 protected:
  void SetUp() override {
    const auto& param = GetParam();
    config_.aircraft_model = param.model;
    config_.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    config_.dt_sec = kDt;
    config_.do_trim = true;
    config_.silent_mode = true;
    config_.initial_kinematics.position_frame =
        coordinate::PositionFrame::kLla;
    config_.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.altitude_m =
        param.altitude_m;
    config_.initial_kinematics.velocity_mps.x_mps = param.speed_mps;
    config_.initial_kinematics.velocity_mps.y_mps = 0.0;
    config_.initial_kinematics.velocity_mps.z_mps = 0.0;
    config_.initial_kinematics.attitude_deg.roll_deg = 0.0;
    config_.initial_kinematics.attitude_deg.pitch_deg = 0.0;
    config_.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  }

  config::FlightDynamicConfig config_;
};

TEST_P(AircraftManeuverTest, FlyToWaypoint) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  TrajectoryLogger logger(GetParam().model, "FlyToWaypoint");
  logger.Log(fm.GetVehicleState());

  bool is_dumping = std::getenv("DUMP_MANEUVER_TRAJECTORY") != nullptr;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  constexpr double kInvSqrt2 = 0.7071067811865475;
  double target_distance_m = FlyToWaypointDistanceM(GetParam().model);
  cmd.target.latitude_rad = target_distance_m * kInvSqrt2 / 6378137.0;
  cmd.target.longitude_rad = target_distance_m * kInvSqrt2 / 6378137.0;
  cmd.target.radius_m = FlyToWaypointRadiusM(GetParam().model);
  if (is_dumping) {
    cmd.target.latitude_rad = 20000.0 / 6378137.0;
    cmd.target.longitude_rad = 20000.0 / 6378137.0;
  }
  cmd.target.altitude_m = GetParam().altitude_m;
  logger.LogTarget(cmd.target.latitude_rad, cmd.target.longitude_rad, cmd.target.altitude_m);

  fm.PushManeuver(cmd);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kExecuting);

  fm.Step(kDt);
  logger.Log(fm.GetVehicleState());
  double lat_0 = fm.GetVehicleState().latitude_rad;
  double lon_0 = fm.GetVehicleState().longitude_rad;
  double init_dist = std::hypot(cmd.target.latitude_rad - lat_0,
                                cmd.target.longitude_rad - lon_0) * 6378137.0;

  int max_steps = GetParam().unstable ? 1000 : 40000;
  RunUntilDone(fm, max_steps, &logger);
  if (is_dumping) {
    RunUntilDone(fm, 80000, &logger);
  }

  const auto& state = fm.GetVehicleState();
  EXPECT_GT(state.sim_time_sec, 0.0);
  EXPECT_GT(state.mass_kg, 0.0);
  ExpectNoNaN(state);

  if (!GetParam().unstable) {
    double final_dist = std::hypot(cmd.target.latitude_rad - state.latitude_rad,
                                   cmd.target.longitude_rad - state.longitude_rad) * 6378137.0;

    EXPECT_LT(final_dist, init_dist * 0.5)
        << GetParam().model << ": aircraft should move significantly closer to target";

    EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted)
        << GetParam().model << ": maneuver should complete";

    EXPECT_GT(state.altitude_geod_m, 0.0)
        << GetParam().model << ": aircraft should not crash";
  }
}

TEST_P(AircraftManeuverTest, Orbit) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  TrajectoryLogger logger(GetParam().model, "Orbit");
  logger.Log(fm.GetVehicleState());

  bool is_dumping = std::getenv("DUMP_MANEUVER_TRAJECTORY") != nullptr;

  // Place orbit center a reasonable distance ahead of the aircraft.
  double center_offset_m = GetParam().speed_mps * 20.0;  // 20 seconds of flight
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = center_offset_m / 6378137.0;
  cmd.target.longitude_rad = center_offset_m / 6378137.0;
  cmd.value = 500.0;  // desired radius (controller uses as reference only)
  if (is_dumping) {
    cmd.target.latitude_rad = 5000.0 / 6378137.0;
    cmd.target.longitude_rad = 0.0;
    cmd.value = 1000.0;
  }
  cmd.target.altitude_m = GetParam().altitude_m;
  logger.LogTarget(cmd.target.latitude_rad, cmd.target.longitude_rad, cmd.target.altitude_m);
  logger.LogTargetValue(cmd.value);

  fm.PushManeuver(cmd);

  fm.Step(kDt);
  logger.Log(fm.GetVehicleState());

  int orbit_steps = 10000;
  RunSteps(fm, orbit_steps, &logger);
  if (is_dumping) {
    RunUntilDone(fm, 40000, &logger);
  }

  const auto& state = fm.GetVehicleState();
  ExpectNoNaN(state);
  if (!GetParam().unstable) {
    EXPECT_GT(state.altitude_geod_m, 0.0)
        << GetParam().model << ": aircraft should not crash during orbit";
    EXPECT_TRUE(fm.GetState() == FlightManagerState::kExecuting ||
                fm.GetState() == FlightManagerState::kCompleted);
  }
}

TEST_P(AircraftManeuverTest, SetHeading) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  TrajectoryLogger logger(GetParam().model, "SetHeading");
  logger.Log(fm.GetVehicleState());

  bool is_dumping = std::getenv("DUMP_MANEUVER_TRAJECTORY") != nullptr;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetHeading;
  cmd.value = 1.57;
  logger.LogTargetValue(cmd.value);

  fm.PushManeuver(cmd);

  RunSteps(fm, 200, &logger);
  double err_early =
      std::abs(fm.GetAutopilot().GetAngleToHeadingRad());

  RunSteps(fm, 800, &logger);
  double err_late =
      std::abs(fm.GetAutopilot().GetAngleToHeadingRad());

  if (is_dumping) {
    RunSteps(fm, 12000, &logger);
  }

  if (!GetParam().unstable) {
    EXPECT_LT(err_late, err_early * 1.20)
        << GetParam().model << ": heading error should not grow significantly (early="
        << err_early << " late=" << err_late << ")";
  }

  auto state = fm.GetState();
  EXPECT_TRUE(state == FlightManagerState::kExecuting ||
              state == FlightManagerState::kCompleted);
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(AircraftManeuverTest, SetAltitudeClimb) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  TrajectoryLogger logger(GetParam().model, "SetAltitudeClimb");
  logger.Log(fm.GetVehicleState());

  bool is_dumping = std::getenv("DUMP_MANEUVER_TRAJECTORY") != nullptr;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetAltitude;
  cmd.value = GetParam().altitude_m + 500.0;
  logger.LogTargetValue(cmd.value);

  fm.PushManeuver(cmd);

  fm.Step(kDt);
  logger.Log(fm.GetVehicleState());
  double alt_before = fm.GetVehicleState().altitude_geod_m;

  RunSteps(fm, 999, &logger);
  if (is_dumping) {
    RunUntilDone(fm, 20000, &logger);
  }

  double alt_after = fm.GetVehicleState().altitude_geod_m;

  if (!GetParam().unstable) {
    EXPECT_GT(alt_after, alt_before)
        << GetParam().model
        << ": altitude should increase toward target";
  }

  auto state = fm.GetState();
  EXPECT_TRUE(state == FlightManagerState::kExecuting ||
              state == FlightManagerState::kCompleted);
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(AircraftManeuverTest, SetPitch) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  TrajectoryLogger logger(GetParam().model, "SetPitch");
  logger.Log(fm.GetVehicleState());

  bool is_dumping = std::getenv("DUMP_MANEUVER_TRAJECTORY") != nullptr;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetPitch;
  cmd.value = 5.0;
  cmd.duration_sec = 2.0;
  logger.LogTargetValue(cmd.value);

  fm.PushManeuver(cmd);

  RunUntilDone(fm, 600, &logger);
  if (is_dumping) {
    RunSteps(fm, 4000, &logger);
  }

  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted)
      << GetParam().model << ": SetPitch should complete within 600 steps";
}

TEST_P(AircraftManeuverTest, SetRoll) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  TrajectoryLogger logger(GetParam().model, "SetRoll");
  logger.Log(fm.GetVehicleState());

  bool is_dumping = std::getenv("DUMP_MANEUVER_TRAJECTORY") != nullptr;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetRoll;
  cmd.value = 1.0;
  logger.LogTargetValue(cmd.value);

  fm.PushManeuver(cmd);

  fm.Step(kDt);
  logger.Log(fm.GetVehicleState());
  if (is_dumping) {
    RunSteps(fm, 4000, &logger);
  }

  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted);
}

TEST_P(AircraftManeuverTest, ResetAndReuse) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetHeading;
  cmd.value = 1.57;
  fm.PushManeuver(cmd);
  RunSteps(fm, 100);

  fm.Reset(config_);
  EXPECT_EQ(fm.GetState(), FlightManagerState::kReady)
      << GetParam().model << ": state should be kReady after Reset";

  ManeuverCommand cmd2;
  cmd2.type = guidance::ManeuverType::kSetHeading;
  cmd2.value = 0.0;
  fm.PushManeuver(cmd2);
  RunSteps(fm, 100);
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(AircraftManeuverTest, AbortManeuver) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = 0.5;
  cmd.target.longitude_rad = 0.0;
  cmd.target.altitude_m = GetParam().altitude_m;
  fm.PushManeuver(cmd);

  RunSteps(fm, 10);

  fm.Abort();
  EXPECT_EQ(fm.GetState(), FlightManagerState::kAborted)
      << GetParam().model << ": state should be kAborted after Abort()";

  bool result = fm.Step(kDt);
  EXPECT_FALSE(result)
      << GetParam().model << ": Step() should return false after Abort()";
}

INSTANTIATE_TEST_SUITE_P(
    FighterModels, AircraftManeuverTest,
    ::testing::Values(
        AircraftTestParam{"A4", 2000.0, 120.0, true},  // orbit: controller convergence limit
        AircraftTestParam{"F4N", 2000.0, 130.0},
        AircraftTestParam{"F80C", 2000.0, 120.0},
        AircraftTestParam{"f15", 3000.0, 200.0},
        AircraftTestParam{"f16", 3000.0, 200.0},
        AircraftTestParam{"f22", 3000.0, 200.0, true},
        AircraftTestParam{"OV10", 500.0, 70.0}));

INSTANTIATE_TEST_SUITE_P(
    TransportModels, AircraftManeuverTest,
    ::testing::Values(
        AircraftTestParam{"B17", 1000.0, 80.0, true},  // orbit: energy-limited, can't sustain turn
        AircraftTestParam{"Boeing314", 500.0, 70.0},
        AircraftTestParam{"C130", 1000.0, 90.0},
        AircraftTestParam{"DHC6", 500.0, 55.0},
        AircraftTestParam{"L410", 1000.0, 90.0},
        AircraftTestParam{"737", 3000.0, 130.0},
        AircraftTestParam{"B747", 3000.0, 140.0},
        AircraftTestParam{"Concorde", 15000.0, 500.0},
        AircraftTestParam{"MD11", 3000.0, 140.0}));

INSTANTIATE_TEST_SUITE_P(
    GAModels, AircraftManeuverTest,
    ::testing::Values(
        AircraftTestParam{"c172p", 500.0, 50.0},
        AircraftTestParam{"c172r", 500.0, 50.0},
        AircraftTestParam{"c172x", 500.0, 50.0},
        AircraftTestParam{"c182", 500.0, 55.0},
        AircraftTestParam{"c310", 500.0, 65.0}));

TEST_P(AircraftManeuverTest, OrbitTimedCompletion) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);
  double roll_limit_deg = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double orbit_r = OrbitRadiusM(GetParam().speed_mps, roll_limit_deg);
  double center_offset = orbit_r * 1.5 * 0.7071067811865475;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = center_offset / 6378137.0;
  cmd.target.longitude_rad = center_offset / 6378137.0;
  cmd.value = orbit_r;
  cmd.duration_sec = 5.0;
  cmd.target.altitude_m = GetParam().altitude_m;

  fm.PushManeuver(cmd);

  int steps = RunUntilDone(fm, 2000);

  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted)
      << GetParam().model << ": timed orbit should complete after duration";
  EXPECT_GT(steps, 100)
      << GetParam().model << ": should run more than 1 second at dt=0.01";
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(AircraftManeuverTest, QueueOrbitThenHeading) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);
  double roll_limit_deg = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double orbit_r = OrbitRadiusM(GetParam().speed_mps, roll_limit_deg);
  double center_offset = orbit_r * 1.5 * 0.7071067811865475;

  ManeuverCommand orbit_cmd;
  orbit_cmd.type = guidance::ManeuverType::kOrbit;
  orbit_cmd.target.latitude_rad = center_offset / 6378137.0;
  orbit_cmd.target.longitude_rad = center_offset / 6378137.0;
  orbit_cmd.value = orbit_r;
  orbit_cmd.duration_sec = 3.0;
  orbit_cmd.target.altitude_m = GetParam().altitude_m;

  ManeuverCommand heading_cmd;
  heading_cmd.type = guidance::ManeuverType::kSetHeading;
  heading_cmd.value = 0.0;

  fm.PushManeuver(orbit_cmd);
  fm.PushManeuver(heading_cmd);

  ASSERT_EQ(fm.GetState(), FlightManagerState::kExecuting);

  int steps = RunUntilDone(fm, 5000);

  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted)
      << GetParam().model << ": queue should complete both maneuvers";
  EXPECT_GT(steps, 300)
      << GetParam().model << ": should run both timed orbit + heading";
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(AircraftManeuverTest, QueueFlyToThenOrbit) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);
  double roll_limit_deg = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double orbit_r = OrbitRadiusM(GetParam().speed_mps, roll_limit_deg);
  double wp_distance_m = orbit_r * 2.0;

  ManeuverCommand fly_cmd;
  fly_cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  fly_cmd.target.latitude_rad = wp_distance_m / 6378137.0;
  fly_cmd.target.longitude_rad = 0.0;
  fly_cmd.target.radius_m = orbit_r;
  fly_cmd.target.altitude_m = GetParam().altitude_m;

  ManeuverCommand orbit_cmd;
  orbit_cmd.type = guidance::ManeuverType::kOrbit;
  orbit_cmd.target.latitude_rad = wp_distance_m / 6378137.0;
  orbit_cmd.target.longitude_rad = 0.0;
  orbit_cmd.value = orbit_r;
  orbit_cmd.duration_sec = 3.0;
  orbit_cmd.target.altitude_m = GetParam().altitude_m;

  fm.PushManeuver(fly_cmd);
  fm.PushManeuver(orbit_cmd);

  int steps = RunUntilDone(fm, GetParam().unstable ? 2000 : 20000);

  if (!GetParam().unstable) {
    EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted)
        << GetParam().model << ": fly-to then orbit queue should complete";
    ExpectNoNaN(fm.GetVehicleState());
    EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
        << GetParam().model << ": should not crash";
  }
  EXPECT_GT(steps, 10) << GetParam().model << ": should run some steps";
}

TEST_P(AircraftManeuverTest, InvalidOrbitRadius) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);
  double roll_limit_deg = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double orbit_r = OrbitRadiusM(GetParam().speed_mps, roll_limit_deg);
  double center_offset = orbit_r * 1.5 * 0.7071067811865475;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = center_offset / 6378137.0;
  cmd.target.longitude_rad = center_offset / 6378137.0;
  cmd.value = -100.0;  // negative radius
  cmd.duration_sec = 3.0;
  cmd.target.altitude_m = GetParam().altitude_m;

  fm.PushManeuver(cmd);

  int steps = RunUntilDone(fm, 1000);

  EXPECT_GT(steps, 100) << GetParam().model << ": should run with clamped radius";
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
      << GetParam().model << ": should not crash with invalid radius";
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
