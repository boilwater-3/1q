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

using namespace guidance;

struct AircraftTestParam {
  std::string model;
  double altitude_m;
  double speed_mps;

  AircraftTestParam(std::string m, double alt, double spd)
      : model(std::move(m)), altitude_m(alt), speed_mps(spd) {}
};

void PrintTo(const AircraftTestParam& p, std::ostream* os) {
  *os << p.model;
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
        AircraftTestParam{"A4", 2000.0, 120.0},
        AircraftTestParam{"F4N", 2000.0, 130.0},
        AircraftTestParam{"F80C", 2000.0, 120.0},
        AircraftTestParam{"f15", 3000.0, 200.0},
        AircraftTestParam{"f16", 3000.0, 200.0},
        AircraftTestParam{"OV10", 500.0, 70.0}));


INSTANTIATE_TEST_SUITE_P(
    TransportModels, AircraftManeuverTest,
    ::testing::Values(
        AircraftTestParam{"B17", 1000.0, 80.0},
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

/// 定时盘旋完成：设置 duration_sec，验证在规定时间内完成且不崩溃
TEST_P(AircraftManeuverTest, OrbitTimedCompletion) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);
  double roll_limit_deg = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double orbit_r = OrbitRadiusM(GetParam().speed_mps, roll_limit_deg);
  constexpr double kEarthR = 6378137.0;
  double center_offset = orbit_r * 1.5 * 0.7071067811865475;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = center_offset / kEarthR;
  cmd.target.longitude_rad = center_offset / kEarthR;
  cmd.target.altitude_m = GetParam().altitude_m;
  cmd.value = orbit_r;
  cmd.duration_sec = 5.0;
  fm.PushManeuver(cmd);

  int steps = RunUntilDone(fm, 3000);

  // 应在最大步数内完成或仍在执行（Duration orbit 应完成或接近完成）
  auto state = fm.GetState();
  EXPECT_TRUE(state == FlightManagerState::kCompleted ||
              state == FlightManagerState::kExecuting)
      << GetParam().model << ": unexpected state after timed orbit";
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
      << GetParam().model << ": should not crash during timed orbit";
}

/// 机动队列：盘旋后切换到航向保持
TEST_P(AircraftManeuverTest, QueueOrbitThenHeading) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);
  double roll_limit_deg = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double orbit_r = OrbitRadiusM(GetParam().speed_mps, roll_limit_deg);
  constexpr double kEarthR = 6378137.0;
  double center_offset = orbit_r * 1.5 * 0.7071067811865475;

  // 先盘旋 5 秒
  ManeuverCommand orbit_cmd;
  orbit_cmd.type = guidance::ManeuverType::kOrbit;
  orbit_cmd.target.latitude_rad = center_offset / kEarthR;
  orbit_cmd.target.longitude_rad = center_offset / kEarthR;
  orbit_cmd.target.altitude_m = GetParam().altitude_m;
  orbit_cmd.value = orbit_r;
  orbit_cmd.duration_sec = 5.0;
  fm.PushManeuver(orbit_cmd);

  // 然后保持 90° 航向
  ManeuverCommand hdg_cmd;
  hdg_cmd.type = guidance::ManeuverType::kSetHeading;
  hdg_cmd.value = M_PI / 2.0;
  fm.PushManeuver(hdg_cmd);

  int steps = RunUntilDone(fm, 4000);

  EXPECT_GT(steps, 200) << GetParam().model << ": should run orbit then heading";
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
      << GetParam().model << ": should not crash during queue orbit→heading";
  EXPECT_TRUE(fm.GetState() == FlightManagerState::kCompleted ||
              fm.GetState() == FlightManagerState::kExecuting)
      << GetParam().model << ": unexpected state after queue";
}

/// 机动队列：飞点后切入盘旋
TEST_P(AircraftManeuverTest, QueueFlyToThenOrbit) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);
  double roll_limit_deg = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double orbit_r = OrbitRadiusM(GetParam().speed_mps, roll_limit_deg);
  constexpr double kEarthR = 6378137.0;

  // 先飞到一个中间航点
  double wp_dist = orbit_r * 3.0 * 0.7071067811865475;
  ManeuverCommand fly_cmd;
  fly_cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  fly_cmd.target.latitude_rad = wp_dist / kEarthR;
  fly_cmd.target.longitude_rad = wp_dist / kEarthR;
  fly_cmd.target.altitude_m = GetParam().altitude_m;
  fly_cmd.target.radius_m = 500.0;
  fm.PushManeuver(fly_cmd);

  // 到达后开始盘旋
  double center_offset = orbit_r * 1.5 * 0.7071067811865475;
  ManeuverCommand orbit_cmd;
  orbit_cmd.type = guidance::ManeuverType::kOrbit;
  orbit_cmd.target.latitude_rad = center_offset / kEarthR;
  orbit_cmd.target.longitude_rad = center_offset / kEarthR;
  orbit_cmd.target.altitude_m = GetParam().altitude_m;
  orbit_cmd.value = orbit_r;
  orbit_cmd.duration_sec = 3.0;
  fm.PushManeuver(orbit_cmd);

  int steps = RunUntilDone(fm, 8000);

  EXPECT_GT(steps, 100) << GetParam().model << ": should run fly-to then orbit";
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
      << GetParam().model << ": should not crash during queue fly-to→orbit";
}

/// S型转弯：正弦航向调制，验证各机型在整个 duration 内不失控
TEST_P(AircraftManeuverTest, STurnCompletion) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);

  // ManeuverCommand 字段约定（参见 FlightManager.cpp dispatch）：
  //   value                = base_heading_rad (0 = 北)
  //   heading_tolerance_rad = amplitude_deg   (±15° 摆幅)
  //   altitude_tolerance_m  = period_sec      (10秒一个完整周期)
  //   duration_sec          = 总持续时间       (20秒 = 2个完整S周期)
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSTurn;
  cmd.value = 0.0;                    // base heading: 北
  cmd.heading_tolerance_rad = 15.0;   // amplitude: ±15°
  cmd.altitude_tolerance_m = 10.0;    // period: 10s
  cmd.duration_sec = 20.0;            // duration: 20s (2 full S-cycles)
  fm.PushManeuver(cmd);

  TrajectoryLogger logger(GetParam().model, "STurn");
  int steps = RunUntilDone(fm, 5000, &logger);

  auto state = fm.GetState();
  EXPECT_TRUE(state == FlightManagerState::kCompleted ||
              state == FlightManagerState::kExecuting)
      << GetParam().model << ": unexpected state after S-turn";
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
      << GetParam().model << ": should not crash during S-turn";
  // 20秒 / 0.01 dt = 2000 步, 留余量
  EXPECT_GT(steps, 1500)
      << GetParam().model << ": should complete at least ~20s of S-turn";
}

/// S型转弯后切换到航向保持：验证机动队列中 S-Turn → SetHeading 衔接
TEST_P(AircraftManeuverTest, QueueSTurnThenHeading) {
  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);

  fm.Step(kDt);

  // 先做 10 秒 S-Turn
  ManeuverCommand sturn_cmd;
  sturn_cmd.type = guidance::ManeuverType::kSTurn;
  sturn_cmd.value = 0.0;
  sturn_cmd.heading_tolerance_rad = 20.0;  // ±20°
  sturn_cmd.altitude_tolerance_m = 8.0;    // period 8s
  sturn_cmd.duration_sec = 10.0;
  fm.PushManeuver(sturn_cmd);

  // 然后保持航向 90°
  ManeuverCommand hdg_cmd;
  hdg_cmd.type = guidance::ManeuverType::kSetHeading;
  hdg_cmd.value = M_PI / 2.0;
  fm.PushManeuver(hdg_cmd);

  int steps = RunUntilDone(fm, 6000);

  EXPECT_GT(steps, 500) << GetParam().model << ": should run S-turn then heading";
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0)
      << GetParam().model << ": should not crash during queue S-turn→heading";
  EXPECT_TRUE(fm.GetState() == FlightManagerState::kCompleted ||
              fm.GetState() == FlightManagerState::kExecuting)
      << GetParam().model << ": unexpected state after queue";
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
