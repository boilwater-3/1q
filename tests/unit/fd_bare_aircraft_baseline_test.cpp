#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/model/VehicleStateMapper.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"
#include "simgear/misc/sg_path.hxx"

namespace oneq {
namespace flight_dynamic {
namespace {

constexpr double kDt = 0.01;
constexpr double kMToFt = 1.0 / 0.3048;
constexpr double kFtToM = 0.3048;
constexpr double kFreeFlightSec = 10.0;

struct BareAircraftParam {
  std::string model;
  double altitude_m;
  double speed_mps;
};

struct BareAircraftResult {
  // L0: 可加载
  bool model_loaded = false;
  // L1: 可初始化
  bool ic_applied = false;
  bool run_ic_ok = false;
  bool engines_started = false;
  bool gear_retracted = false;
  // L3: 可配平
  bool trim_attempted = false;
  bool trim_succeeded = false;
  bool trim_recovery_applied = false;
  // L2: 可短时运行
  bool run_ok = false;
  bool stable = false;
  double sim_time_sec = 0.0;
  double alt_start_m = 0.0;
  double alt_end_m = 0.0;
  double speed_start_mps = 0.0;
  double speed_end_mps = 0.0;
};

std::vector<BareAircraftParam> FocusAircraft() {
  return {
      {"f22", 3000.0, 200.0}, {"Concorde", 5000.0, 150.0},
      {"B17", 1000.0, 80.0},  {"C130", 1000.0, 90.0},
      {"L410", 1000.0, 90.0}, {"c310", 500.0, 65.0},
  };
}

config::FlightDynamicConfig MakeBaselineConfig(const BareAircraftParam& param,
                                                bool do_trim) {
  config::FlightDynamicConfig config;
  config.aircraft_model = param.model;
  config.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  config.dt_sec = kDt;
  config.do_trim = do_trim;
  config.silent_mode = true;
  config.initial_kinematics.position_frame = coordinate::PositionFrame::kLla;
  config.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
  config.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
  config.initial_kinematics.position_lla_deg_m.altitude_m = param.altitude_m;
  config.initial_kinematics.velocity_mps.x_mps = param.speed_mps;
  config.initial_kinematics.velocity_mps.y_mps = 0.0;
  config.initial_kinematics.velocity_mps.z_mps = 0.0;
  config.initial_kinematics.attitude_deg.roll_deg = 0.0;
  config.initial_kinematics.attitude_deg.pitch_deg = 0.0;
  config.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  return config;
}

bool IsStable(const model::VehicleState& state) {
  return std::isfinite(state.altitude_geod_m) &&
         std::isfinite(state.vtrue_mps) &&
         std::isfinite(state.phi_rad) &&
         std::isfinite(state.theta_rad) &&
         std::isfinite(state.psi_rad) &&
         state.mass_kg > 0.0 &&
         state.altitude_geod_m > 0.0 &&
         state.vtrue_mps > 1.0;
}

BareAircraftResult RunBaseline(const BareAircraftParam& param, bool do_trim) {
  BareAircraftResult result;
  const auto config = MakeBaselineConfig(param, do_trim);

  std::unique_ptr<adapter::JsbsimAdapter> adapter;
  try {
    adapter.reset(new adapter::JsbsimAdapter(config));
  } catch (...) {
    return result;
  }

  const auto& diag = adapter->GetInitDiagnostics();
  result.model_loaded = diag.model_loaded;
  result.ic_applied = diag.ic_applied;
  result.run_ic_ok = diag.run_ic_ok;
  result.engines_started = diag.engines_started;
  result.gear_retracted = diag.gear_retracted;
  result.trim_attempted = diag.trim_attempted;
  result.trim_succeeded = diag.trim_succeeded;
  result.trim_recovery_applied = diag.trim_recovery_applied;

  const auto start = model::VehicleStateMapper::Map(
      adapter->GetPropagate(), adapter->GetAccelerations(),
      adapter->GetFdmExec(), 0.0);
  result.alt_start_m = start.altitude_geod_m;
  result.speed_start_mps = start.vtrue_mps;

  const int steps = static_cast<int>(kFreeFlightSec / kDt);
  for (int i = 0; i < steps; ++i) {
    result.run_ok = adapter->Run();
    result.sim_time_sec += kDt;
    if (!result.run_ok) break;

    const auto state = model::VehicleStateMapper::Map(
        adapter->GetPropagate(), adapter->GetAccelerations(),
        adapter->GetFdmExec(), result.sim_time_sec);
    if (state.altitude_geod_m <= 0.0 || !std::isfinite(state.vtrue_mps)) {
      break;
    }
  }

  const auto end = model::VehicleStateMapper::Map(
      adapter->GetPropagate(), adapter->GetAccelerations(),
      adapter->GetFdmExec(), result.sim_time_sec);
  result.alt_end_m = end.altitude_geod_m;
  result.speed_end_mps = end.vtrue_mps;
  result.stable = result.run_ok && IsStable(end);

  return result;
}

// ── Reset XML baseline (JSBSim native IC) ─────────────────────────────────

BareAircraftResult RunResetBaseline(const BareAircraftParam& param) {
  BareAircraftResult result;

  JSBSim::FGFDMExec fdm;
  fdm.SetDebugLevel(0);
  fdm.SetRootDir(SGPath(FD_JSBSIM_ROOT_DIR));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetEnginePath(SGPath("engine"));
  fdm.SetSystemsPath(SGPath("systems"));
  fdm.Setdt(kDt);

  result.model_loaded = fdm.LoadModel(param.model, true);
  if (!result.model_loaded) return result;

  auto ic = fdm.GetIC();
  result.ic_applied = ic->Load(SGPath("reset00.xml"), true);
  if (!result.ic_applied) return result;

  result.run_ic_ok = fdm.RunIC();
  if (!result.run_ic_ok) return result;

  fdm.GetPropulsion()->InitRunning(-1);
  result.engines_started = true;

  const auto start = model::VehicleStateMapper::Map(
      *fdm.GetPropagate(), *fdm.GetAccelerations(), fdm, 0.0);
  result.alt_start_m = start.altitude_geod_m;
  result.speed_start_mps = start.vtrue_mps;

  const int steps = static_cast<int>(kFreeFlightSec / kDt);
  for (int i = 0; i < steps; ++i) {
    result.run_ok = fdm.Run();
    result.sim_time_sec += kDt;
    if (!result.run_ok) break;

    const auto state = model::VehicleStateMapper::Map(
        *fdm.GetPropagate(), *fdm.GetAccelerations(), fdm, result.sim_time_sec);
    if (state.altitude_geod_m <= 0.0 || !std::isfinite(state.vtrue_mps)) break;
  }

  const auto end = model::VehicleStateMapper::Map(
      *fdm.GetPropagate(), *fdm.GetAccelerations(), fdm, result.sim_time_sec);
  result.alt_end_m = end.altitude_geod_m;
  result.speed_end_mps = end.vtrue_mps;
  result.stable = result.run_ok &&
                  std::isfinite(end.altitude_geod_m) &&
                  std::isfinite(end.vtrue_mps) &&
                  end.mass_kg > 0.0;

  return result;
}

// ── L0: 所有 6 个重点机型必须可加载 ──────────────────────────────────────

class BareAircraftTest : public ::testing::TestWithParam<BareAircraftParam> {};

TEST_P(BareAircraftTest, ModelLoadsSuccessfully) {
  const auto& param = GetParam();
  JSBSim::FGFDMExec fdm;
  fdm.SetDebugLevel(0);
  fdm.SetRootDir(SGPath(FD_JSBSIM_ROOT_DIR));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetEnginePath(SGPath("engine"));
  fdm.SetSystemsPath(SGPath("systems"));
  ASSERT_TRUE(fdm.LoadModel(param.model, true))
      << param.model << " failed to load";
}

TEST_P(BareAircraftTest, ProjectIcTrimOnPassesL1) {
  const auto& param = GetParam();
  const auto result = RunBaseline(param, true);
  EXPECT_TRUE(result.model_loaded) << param.model << " L0 failed";
  EXPECT_TRUE(result.run_ic_ok) << param.model << " L1 failed (RunIC)";
  EXPECT_TRUE(result.engines_started) << param.model << " engines not started";
}

TEST_P(BareAircraftTest, ProjectIcTrimOffPassesL1) {
  const auto& param = GetParam();
  const auto result = RunBaseline(param, false);
  EXPECT_TRUE(result.model_loaded) << param.model << " L0 failed";
  EXPECT_TRUE(result.run_ic_ok) << param.model << " L1 failed (RunIC)";
  EXPECT_TRUE(result.engines_started) << param.model << " engines not started";
}

TEST_P(BareAircraftTest, ProjectIcTrimOnFreeFlightStable) {
  const auto& param = GetParam();
  const auto result = RunBaseline(param, true);
  ASSERT_TRUE(result.run_ic_ok) << param.model << " skipped: L1 failed";

  EXPECT_TRUE(result.run_ok) << param.model
                             << " Run() returned false during 10s free flight";
  EXPECT_TRUE(result.stable)
      << param.model << " unstable after 10s: alt=" << result.alt_end_m
      << "m, speed=" << result.speed_end_mps << "m/s";
}

TEST_P(BareAircraftTest, ProjectIcTrimOffFreeFlightStable) {
  const auto& param = GetParam();
  const auto result = RunBaseline(param, false);
  ASSERT_TRUE(result.run_ic_ok) << param.model << " skipped: L1 failed";

  EXPECT_TRUE(result.run_ok) << param.model
                             << " Run() returned false during 10s free flight";
  EXPECT_TRUE(result.stable)
      << param.model << " unstable after 10s: alt=" << result.alt_end_m
      << "m, speed=" << result.speed_end_mps << "m/s";
}

TEST_P(BareAircraftTest, TrimBehaviorRecorded) {
  const auto& param = GetParam();
  const auto result = RunBaseline(param, true);
  ASSERT_TRUE(result.run_ic_ok);

  EXPECT_TRUE(result.trim_attempted);
}

INSTANTIATE_TEST_SUITE_P(
    FocusAircraft, BareAircraftTest,
    ::testing::ValuesIn(FocusAircraft()),
    [](const ::testing::TestParamInfo<BareAircraftParam>& info) {
      return info.param.model;
    });

// ── Reset XML baseline (JSBSim native IC for aircraft with reset00.xml) ──

class ResetXmlBaselineTest : public ::testing::TestWithParam<BareAircraftParam> {};

TEST_P(ResetXmlBaselineTest, ResetXmlFreeFlightStable) {
  const auto& param = GetParam();

  // Check if reset00.xml exists before attempting load
  std::ifstream reset_file(std::string(FD_JSBSIM_ROOT_DIR) + "/aircraft/" +
                            param.model + "/reset00.xml");
  if (!reset_file.good()) {
    GTEST_SKIP() << param.model << " has no reset00.xml";
  }

  const auto result = RunResetBaseline(param);
  ASSERT_TRUE(result.model_loaded) << param.model << " failed to load";
  ASSERT_TRUE(result.ic_applied) << param.model << " reset00.xml load failed";

  EXPECT_TRUE(result.run_ic_ok) << param.model << " RunIC() failed with reset00.xml";
  EXPECT_TRUE(result.run_ok) << param.model
                             << " Run() returned false during reset free flight";
  EXPECT_TRUE(result.stable)
      << param.model << " unstable in reset free flight: alt=" << result.alt_end_m
      << "m, speed=" << result.speed_end_mps << "m/s";
}

INSTANTIATE_TEST_SUITE_P(
    ResetXml, ResetXmlBaselineTest,
    ::testing::ValuesIn(FocusAircraft()),
    [](const ::testing::TestParamInfo<BareAircraftParam>& info) {
      return info.param.model;
    });

// ── Trim diagnostics detail tests ───────────────────────────────────────

TEST(BareAircraftDiagnosticTest, F22TrimOnAppliesRecovery) {
  const BareAircraftResult result = RunBaseline({"f22", 3000.0, 200.0}, true);
  ASSERT_TRUE(result.model_loaded);
  ASSERT_TRUE(result.run_ic_ok);
  EXPECT_TRUE(result.trim_attempted);
  EXPECT_TRUE(result.trim_recovery_applied)
      << "f22 DoTrim(0) should trigger recovery; recovery_applied records this";
}

TEST(BareAircraftDiagnosticTest, F22TrimOffSkipsTrim) {
  const BareAircraftResult result = RunBaseline({"f22", 3000.0, 200.0}, false);
  ASSERT_TRUE(result.model_loaded);
  ASSERT_TRUE(result.run_ic_ok);
  EXPECT_FALSE(result.trim_attempted);
  EXPECT_FALSE(result.trim_succeeded);
}

TEST(BareAircraftDiagnosticTest, ConcordeTrimOnCompletes) {
  const BareAircraftResult result = RunBaseline({"Concorde", 5000.0, 150.0}, true);
  ASSERT_TRUE(result.run_ic_ok);
  EXPECT_TRUE(result.trim_succeeded)
      << "Concorde at 5000m/150mps should trim successfully";
}

TEST(BareAircraftDiagnosticTest, C310TrimOnAppliesRecovery) {
  const BareAircraftResult result = RunBaseline({"c310", 500.0, 65.0}, true);
  ASSERT_TRUE(result.run_ic_ok);
  EXPECT_TRUE(result.trim_attempted);
  // c310 at 500m/65mps: JSBSim reports "udot doesn't appear to be trimmable".
  // Recovery resets FCS state and continues with free flight.
  EXPECT_TRUE(result.trim_recovery_applied)
      << "c310 at 500m/65mps should trigger trim recovery";
}

TEST(BareAircraftDiagnosticTest, AltitudeReasonableAfterTrim) {
  const auto result = RunBaseline({"c310", 500.0, 65.0}, true);
  ASSERT_TRUE(result.stable);

  double alt_delta = result.alt_end_m - result.alt_start_m;
  EXPECT_GT(alt_delta, -200.0) << "c310 altitude dropped too much in 10s free flight";
  EXPECT_LT(alt_delta, 200.0) << "c310 altitude gained too much in 10s free flight";
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
