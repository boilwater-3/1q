/// @file
/// @brief 盘旋（Orbit）单元断言测试 — 快速、确定性，适合 CI 每次运行
///
/// 覆盖内容：
///   - 不崩溃 / 无 NaN / 基本存活
///   - 高度/速度不剧烈漂移（简单阈值）
///   - 航向单步不跳跃 / 边界穿越无抽搐
///   - 切出过渡无突变
///   - 滚转不灾难性超限
///   - 所有边界/极端参数不崩溃
///
/// 质量分析（半径精度、角速度一致性等）请使用独立工具：
///   orbit_quality_csv <model|ALL> [output.csv]
/// 位于 examples/flight_dynamic/orbit_quality_csv.cpp

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "fd_test_helpers.h"

namespace oneq {
namespace flight_dynamic {
namespace {

using namespace guidance;

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;

double NormalizeRad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double DistanceToCenterM(const model::VehicleState& s, const Waypoint& c) {
  double cos_lat = std::cos(c.latitude_rad);
  double dn = (s.latitude_rad - c.latitude_rad) * kEarthRadiusM;
  double de = (s.longitude_rad - c.longitude_rad) * kEarthRadiusM * cos_lat;
  return std::hypot(dn, de);
}

double MinTurnRadiusM(double speed_mps, double max_bank_deg) {
  double tan_bank = std::tan(max_bank_deg * kDegToRad);
  if (tan_bank < 0.01) tan_bank = 0.01;
  return (speed_mps * speed_mps) / (9.80665 * tan_bank);
}

double FeasibleOrbitRadiusM(double speed_mps, double max_bank_deg, double nominal_r) {
  double min_r = MinTurnRadiusM(speed_mps, max_bank_deg);
  return std::max(nominal_r, min_r * 1.2);
}

/// 中心在南侧，使飞行器初始航向（北）= 顺时针切线方向
Waypoint CenterSouthOfAircraft(double radius_m, double altitude_m) {
  Waypoint c;
  c.latitude_rad = -radius_m / kEarthRadiusM;
  c.longitude_rad = 0.0;
  c.altitude_m = altitude_m;
  return c;
}

// ── 参数 ───────────────────────────────────────────────────────────────────

struct OrbitTestParam {
  std::string model;
  double altitude_m;
  double speed_mps;
  double radius_m;

  OrbitTestParam(std::string m, double a, double s, double r)
      : model(std::move(m)), altitude_m(a), speed_mps(s), radius_m(r) {}
};

void PrintTo(const OrbitTestParam& p, std::ostream* os) { *os << p.model; }

std::vector<OrbitTestParam> AllOrbitAircraft() {
  return {
      {"f16", 3000.0, 200.0, 5000.0},
      {"737", 3000.0, 130.0, 3000.0},
      {"c172p", 500.0, 50.0, 1000.0},
      {"Concorde", 10000.0, 250.0, 10000.0},
  };
}

// ── 夹具 ───────────────────────────────────────────────────────────────────

class OrbitUnitTest : public ::testing::TestWithParam<OrbitTestParam> {
 protected:
  void SetUp() override {
    const auto& p = GetParam();
    cfg_.aircraft_model = p.model;
    cfg_.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    cfg_.dt_sec = kDt;
    cfg_.do_trim = true;
    cfg_.silent_mode = true;
    cfg_.initial_kinematics.position_frame = coordinate::PositionFrame::kLla;
    cfg_.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
    cfg_.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
    cfg_.initial_kinematics.position_lla_deg_m.altitude_m = p.altitude_m;
    cfg_.initial_kinematics.velocity_mps.x_mps = p.speed_mps;
    cfg_.initial_kinematics.velocity_mps.y_mps = 0.0;
    cfg_.initial_kinematics.velocity_mps.z_mps = 0.0;
    cfg_.initial_kinematics.attitude_deg.roll_deg = 0.0;
    cfg_.initial_kinematics.attitude_deg.pitch_deg = 0.0;
    cfg_.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  }

  double FeasibleR(FlightManager& fm) const {
    double max_bank = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
    return FeasibleOrbitRadiusM(GetParam().speed_mps, max_bank, GetParam().radius_m);
  }

  config::FlightDynamicConfig cfg_;
};

// ═══════════════════════════════════════════════════════════════════════════
// A1. 基本存活
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, RunsWithoutCrashOrNaN) {
  FlightManager fm(cfg_);
  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, GetParam().altitude_m);

  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = r;
  fm.PushManeuver(cmd);

  RunSteps(fm, 3000);
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0);
  EXPECT_GT(fm.GetVehicleState().vtrue_mps, 1.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// A2. 高度保持
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, AltitudeDoesNotDriftExcessively) {
  FlightManager fm(cfg_);
  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, GetParam().altitude_m);

  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = r;
  fm.PushManeuver(cmd);

  double min_alt = GetParam().altitude_m, max_alt = min_alt;
  for (int i = 0; i < 2000; ++i) {
    fm.Step(kDt);
    double a = fm.GetVehicleState().altitude_geod_m;
    min_alt = std::min(min_alt, a);
    max_alt = std::max(max_alt, a);
  }

  double drift = max_alt - min_alt;
  EXPECT_TRUE(drift < GetParam().altitude_m * 0.25 || drift < 500.0)
      << GetParam().model << ": altitude drift=" << drift
      << " min=" << min_alt << " max=" << max_alt;

  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// A3. 速度不剧烈波动
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, SpeedDoesNotOscillateWildly) {
  FlightManager fm(cfg_);
  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, GetParam().altitude_m);

  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = r;
  fm.PushManeuver(cmd);

  RunSteps(fm, 500);

  double min_spd = std::numeric_limits<double>::max(), max_spd = 0.0, sum_spd = 0.0;
  int n = 0;
  for (int i = 0; i < 500; ++i) {
    fm.Step(kDt);
    double s = fm.GetVehicleState().vtrue_mps;
    min_spd = std::min(min_spd, s);
    max_spd = std::max(max_spd, s);
    sum_spd += s;
    ++n;
  }

  double avg_spd = sum_spd / n;
  double spread = (max_spd - min_spd) / avg_spd;
  EXPECT_LT(spread, 0.40)
      << GetParam().model << ": speed spread=" << (max_spd - min_spd) << " avg=" << avg_spd;

  ExpectNoNaN(fm.GetVehicleState());
}

// ═══════════════════════════════════════════════════════════════════════════
// A4. 航向单步不跳跃
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, HeadingDoesNotJumpSingleStep) {
  FlightManager fm(cfg_);
  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, GetParam().altitude_m);

  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = r;
  fm.PushManeuver(cmd);

  RunSteps(fm, 500);

  double max_step = 0.0;
  double prev = fm.GetVehicleState().psi_rad;
  for (int i = 0; i < 1000; ++i) {
    fm.Step(kDt);
    double cur = fm.GetVehicleState().psi_rad;
    max_step = std::max(max_step, std::abs(NormalizeRad(cur - prev)));
    prev = cur;
  }

  EXPECT_LT(max_step, 20.0 * kDegToRad)
      << GetParam().model << ": max heading step=" << max_step * 180.0 / M_PI << " deg";

  ExpectNoNaN(fm.GetVehicleState());
}

// ═══════════════════════════════════════════════════════════════════════════
// A5. 航向跨越 ±π 边界无 2π 跳变
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, HeadingNoTwoPiJumpAtWrapBoundary) {
  FlightManager fm(cfg_);
  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, GetParam().altitude_m);

  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = r;
  fm.PushManeuver(cmd);

  RunSteps(fm, 5000);

  double max_step = 0.0;
  double prev = fm.GetVehicleState().psi_rad;
  for (int i = 0; i < 500; ++i) {
    fm.Step(kDt);
    double cur = fm.GetVehicleState().psi_rad;
    max_step = std::max(max_step, std::abs(NormalizeRad(cur - prev)));
    prev = cur;
  }

  EXPECT_LT(max_step, 1.0)
      << GetParam().model << ": 2pi jump at wrap boundary="
      << max_step * 180.0 / M_PI << " deg";

  ExpectNoNaN(fm.GetVehicleState());
}

// ═══════════════════════════════════════════════════════════════════════════
// A6. 切出过渡不剧烈
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, ExitToHeadingDoesNotGlitch) {
  FlightManager fm(cfg_);
  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, GetParam().altitude_m);

  ManeuverCommand orbit_cmd;
  orbit_cmd.type = ManeuverType::kOrbit; orbit_cmd.target = c; orbit_cmd.value = r;
  fm.PushManeuver(orbit_cmd);
  RunSteps(fm, 1500);

  ManeuverCommand hdg_cmd;
  hdg_cmd.type = ManeuverType::kSetHeading;
  hdg_cmd.value = NormalizeRad(fm.GetVehicleState().psi_rad + M_PI / 2.0);
  fm.PushManeuver(hdg_cmd);

  double max_roll_step = 0.0, max_alt_jump = 0.0;
  double prev_roll = fm.GetVehicleState().phi_rad;
  double prev_alt = fm.GetVehicleState().altitude_geod_m;

  for (int i = 0; i < 500; ++i) {
    fm.Step(kDt);
    double roll = fm.GetVehicleState().phi_rad;
    double alt = fm.GetVehicleState().altitude_geod_m;
    max_roll_step = std::max(max_roll_step, std::abs(roll - prev_roll));
    max_alt_jump = std::max(max_alt_jump, std::abs(alt - prev_alt));
    prev_roll = roll;
    prev_alt = alt;
  }

  EXPECT_LT(max_roll_step, 10.0 * kDegToRad)
      << GetParam().model << ": exit roll step=" << max_roll_step * 180.0 / M_PI << " deg";
  EXPECT_LT(max_alt_jump, 5.0)
      << GetParam().model << ": exit alt jump=" << max_alt_jump << " m";

  ExpectNoNaN(fm.GetVehicleState());
}

// ═══════════════════════════════════════════════════════════════════════════
// A7. 滚转不灾难性超限（>2×限制的出现次数应为0）
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, RollNotCatastrophicallyOverLimit) {
  FlightManager fm(cfg_);
  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, GetParam().altitude_m);
  double roll_lim = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;

  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = r;
  fm.PushManeuver(cmd);

  RunSteps(fm, 500);

  double max_roll = 0.0;
  int severe_count = 0;  // > 2× limit
  for (int i = 0; i < 1000; ++i) {
    fm.Step(kDt);
    double roll = std::abs(fm.GetVehicleState().phi_rad) * 180.0 / M_PI;
    max_roll = std::max(max_roll, roll);
    if (roll > roll_lim * 2.0) ++severe_count;
  }

  EXPECT_EQ(severe_count, 0)
      << GetParam().model << ": severe roll events=" << severe_count
      << " max_roll=" << max_roll << " lim=" << roll_lim;

  ExpectNoNaN(fm.GetVehicleState());
}

// ═══════════════════════════════════════════════════════════════════════════
// A8–A14. 边界与极端情况 — 什么都不应崩溃
// ═══════════════════════════════════════════════════════════════════════════

TEST_P(OrbitUnitTest, ZeroRadiusDoesNotCrash) {
  FlightManager fm(cfg_);
  Waypoint c = CenterSouthOfAircraft(500.0, GetParam().altitude_m);
  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = 0.0; cmd.duration_sec = 3.0;
  fm.PushManeuver(cmd);
  int steps = RunUntilDone(fm, 1000);
  EXPECT_GT(steps, 50);
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(OrbitUnitTest, NegativeRadiusDoesNotCrash) {
  FlightManager fm(cfg_);
  Waypoint c = CenterSouthOfAircraft(GetParam().radius_m, GetParam().altitude_m);
  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c;
  cmd.value = -GetParam().radius_m; cmd.duration_sec = 3.0;
  fm.PushManeuver(cmd);
  int steps = RunUntilDone(fm, 1000);
  EXPECT_GT(steps, 100);
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(OrbitUnitTest, VerySmallRadiusDoesNotCrash) {
  FlightManager fm(cfg_);
  Waypoint c = CenterSouthOfAircraft(500.0, GetParam().altitude_m);
  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = 0.1; cmd.duration_sec = 3.0;
  fm.PushManeuver(cmd);
  int steps = RunUntilDone(fm, 1000);
  EXPECT_GT(steps, 50);
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(OrbitUnitTest, VeryLargeRadiusDoesNotCrash) {
  FlightManager fm(cfg_);
  constexpr double kLR = 50000.0;
  Waypoint c; c.latitude_rad = -kLR / kEarthRadiusM; c.longitude_rad = 0.0;
  c.altitude_m = GetParam().altitude_m;
  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = kLR;
  fm.PushManeuver(cmd);
  RunSteps(fm, 200);
  ExpectNoNaN(fm.GetVehicleState());
}

TEST_P(OrbitUnitTest, AtOrbitCenterDoesNotCrash) {
  FlightManager fm(cfg_);
  Waypoint c; c.latitude_rad = 0.0; c.longitude_rad = 0.0;
  c.altitude_m = GetParam().altitude_m;
  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = GetParam().radius_m;
  fm.PushManeuver(cmd);
  RunSteps(fm, 500);
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().altitude_geod_m, 0.0);
  double dist = DistanceToCenterM(fm.GetVehicleState(), c);
  EXPECT_GT(dist, 1.0) << GetParam().model << ": stuck at center";
}

TEST_P(OrbitUnitTest, HighAltitudeDoesNotCrash) {
  FlightManager fm(cfg_);
  double ceiling = fm.GetAutopilot().GetControlProfile().ceiling_m;
  if (ceiling <= 0.0) ceiling = 15000.0;
  double test_alt = ceiling * 0.80;

  config::FlightDynamicConfig acfg = cfg_;
  acfg.initial_kinematics.position_lla_deg_m.altitude_m = test_alt;
  fm.Reset(acfg);

  double r = FeasibleR(fm);
  Waypoint c = CenterSouthOfAircraft(r, test_alt);
  ManeuverCommand cmd;
  cmd.type = ManeuverType::kOrbit; cmd.target = c; cmd.value = r;
  fm.PushManeuver(cmd);
  RunSteps(fm, 500);
  ExpectNoNaN(fm.GetVehicleState());
}

// ── 参数化 ─────────────────────────────────────────────────────────────────

INSTANTIATE_TEST_SUITE_P(
    OrbitUnit, OrbitUnitTest,
    ::testing::ValuesIn(AllOrbitAircraft()),
    [](const ::testing::TestParamInfo<OrbitTestParam>& i) { return i.param.model; });

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
