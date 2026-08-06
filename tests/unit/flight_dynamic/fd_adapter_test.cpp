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
#include "flight_dynamic/adapter/PropertyNames.h"
#include "flight_dynamic/guidance/GreatCircleTrackGeometry.h"
#include "flight_dynamic/propulsion/EngineManager.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"

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

TEST_F(FlightDynamicTest, InvalidAircraftModelDoesNotThrow) {
  config_.aircraft_model = "missing_aircraft_model";

  adapter::JsbsimAdapter adapter(config_);
  EXPECT_FALSE(adapter.IsValid());
  EXPECT_TRUE(adapter.GetInitDiagnostics().initialization_failed);
  EXPECT_NE(adapter.GetInitDiagnostics().failure_reason.find("failed to load aircraft"),
            std::string::npos);
  EXPECT_FALSE(adapter.Run());

  FlightManager fm(config_);
  EXPECT_EQ(fm.GetState(), FlightManagerState::kAborted);
  EXPECT_EQ(fm.GetDiagnostics().outcome, ManeuverOutcome::kAborted);
  EXPECT_NE(fm.GetDiagnostics().last_failure_reason.find("failed to load aircraft"),
            std::string::npos);
  EXPECT_FALSE(fm.Step(kDt));
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

TEST_F(FlightDynamicTest, InitialLatitudeIsGeodeticNotGeocentric) {
  // 回归：JSBSim 1.3.1 的 SetLatitudeDegIC/GetLatitude 均为地心纬度语义，
  // 1Q 的 ExternalKinematics LLA 是 WGS84 大地纬度——曾导致非赤道纬度下
  // IC 位置与 VehicleState 纬度整体北移（30°N 处 ≈ 18.5 km，见
  // docs/flight_dynamic/algorithms.md 状态映射节）。赤道处两约定重合，
  // 既有测试无法暴露该问题，这里在 30°N 显式断言。
  config_.do_trim = false;
  config_.initial_kinematics.position_lla_deg_m.latitude_deg = 30.0;
  config_.initial_kinematics.position_lla_deg_m.longitude_deg = 120.0;

  adapter::JsbsimAdapter adapter(config_);
  const double kLat30Rad = 30.0 * M_PI / 180.0;
  // 飞机真实位置（JSBSim ECEF 反算的大地纬度）必须等于配置的大地纬度。
  EXPECT_NEAR(adapter.GetProperty("position/lat-geod-rad"), kLat30Rad, 1.0e-6);

  FlightManager fm(config_);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kReady);
  ASSERT_TRUE(fm.Step(kDt));
  // VehicleState 纬度必须是大地纬度（修复前为地心纬度 30.1666°）。
  EXPECT_NEAR(fm.GetVehicleState().latitude_rad, kLat30Rad, 1.0e-6);
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

TEST_F(FlightDynamicTest, AutopilotDetectsC310Profile) {
  config_.aircraft_model = "c310";
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 500.0;
  config_.initial_kinematics.velocity_mps.x_mps = 65.0;

  FlightManager fm(config_);
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  // c310 has project-injected Autopilot system providing ap/heading_hold.
  EXPECT_EQ(profile.engine_count, 2);
  EXPECT_TRUE(profile.indexed_throttle);   // multi-engine → per-engine throttle
  EXPECT_FALSE(profile.yaw_input_property.empty());
}

TEST_F(FlightDynamicTest, AutopilotDetectsConcordeProfile) {
  config_.aircraft_model = "Concorde";
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 5000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 150.0;

  FlightManager fm(config_);
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  // Tier 2 dynamic detection without model-name hardcoding.
  EXPECT_EQ(profile.engine_count, 4);
  EXPECT_TRUE(profile.indexed_throttle);   // multi-engine → per-engine throttle
  EXPECT_EQ(profile.yaw_input_property, "fcs/rudder-cmd-norm");
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

TEST_F(FlightDynamicTest, EnergyManagementRaisesThrottleForPositiveSpeedError) {
  config_.aircraft_model = "Concorde";
  config_.do_trim = false;
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 5000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 150.0;

  adapter::JsbsimAdapter adapter(config_);
  autopilot::Autopilot ap(adapter);
  ASSERT_TRUE(ap.GetControlProfile().speed_energy_priority);

  ap.SetAltitudeTargetM(5000.0);
  ap.SetAltitudeHold(true);
  ap.SetSpeedTargetMps(ap.GetTrueSpeedMps() + 100.0);
  ap.SetSpeedHold(true);
  ap.Update(kDt);

  EXPECT_GT(adapter.GetProperty("fcs/throttle-cmd-norm"), 0.70)
      << "Positive speed error should add throttle, not reduce it";
}

TEST_F(FlightDynamicTest, AutopilotReadsB747LandingGuidanceOverrides) {
  config_.aircraft_model = "B747";
  config_.do_trim = false;
  config_.initial_kinematics.position_lla_deg_m.altitude_m = 5000.0;
  config_.initial_kinematics.velocity_mps.x_mps = 150.0;

  FlightManager fm(config_);
  auto& adapter = fm.GetAdapter();
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  EXPECT_DOUBLE_EQ(adapter.GetProperty("guidance/landing-heavy-flare"), 1.0);
  EXPECT_DOUBLE_EQ(profile.landing_approach_speed_mps, 68.0);
  EXPECT_DOUBLE_EQ(profile.landing_high_descent_agl_m, 3000.0);
  EXPECT_DOUBLE_EQ(profile.landing_staging_agl_m, 2500.0);
  EXPECT_DOUBLE_EQ(profile.landing_pattern_agl_m, 450.0);
  EXPECT_TRUE(profile.landing_high_descent_orbit);
  EXPECT_DOUBLE_EQ(profile.landing_descent_throttle, 0.15);
  EXPECT_DOUBLE_EQ(profile.landing_approach_flaps_norm, 0.25);
  EXPECT_DOUBLE_EQ(profile.landing_final_flaps_norm, 0.75);
  EXPECT_DOUBLE_EQ(profile.landing_final_throttle_cap, 0.05);
  EXPECT_DOUBLE_EQ(profile.landing_flare_initial_elevator, -0.08);
  EXPECT_TRUE(profile.landing_heavy_flare);
  EXPECT_DOUBLE_EQ(profile.landing_touchdown_agl_m, 5.5);
}

TEST_F(FlightDynamicTest, AutopilotPreservesC172xXmlRollGuidanceOverrides) {
  config_.aircraft_model = "c172x";
  config_.do_trim = false;

  FlightManager fm(config_);
  auto& adapter = fm.GetAdapter();
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  EXPECT_NEAR(adapter.GetProperty(adapter::property::kGuidanceRollAngleLimit), 0.523, 1.0e-6);
  EXPECT_NEAR(adapter.GetProperty(adapter::property::kGuidanceRollRateLimit), 0.174, 1.0e-6);
  EXPECT_NEAR(profile.max_roll_angle_deg, 0.523 * 180.0 / kPi * 0.7, 1.0e-3);
}

// ─── EngineManager contract tests ───────────────────────────────────────────
// Verify that EngineManager-derived aircraft capability parameters
// (CLmax, Vr factor, climb pitch, approach speed fallback) match
// expected values for representative aircraft types.

struct EngineManagerContractParam {
  std::string model_name;
  double altitude_m;
  double speed_mps;
  bool do_trim;
};

class EngineManagerContractTest
    : public ::testing::TestWithParam<EngineManagerContractParam> {
 protected:
  void SetUp() override {
    const auto& param = GetParam();
    config_.aircraft_model = param.model_name;
    config_.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    config_.dt_sec = kDt;
    config_.do_trim = param.do_trim;
    config_.silent_mode = true;
    config_.initial_kinematics.position_frame = coordinate::PositionFrame::kLla;
    config_.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.altitude_m = param.altitude_m;
    config_.initial_kinematics.velocity_mps.x_mps = param.speed_mps;
    config_.initial_kinematics.velocity_mps.y_mps = 0.0;
    config_.initial_kinematics.velocity_mps.z_mps = 0.0;
    config_.initial_kinematics.attitude_deg.roll_deg = 0.0;
    config_.initial_kinematics.attitude_deg.pitch_deg = 0.0;
    config_.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  }

  config::FlightDynamicConfig config_;
};

TEST_P(EngineManagerContractTest, RotationSpeedReasonable) {
  // Verify that Vr falls within a reasonable range for the aircraft type.
  // Exact Vr depends on runtime weight (fuel + payload), so we test
  // range bounds and classification logic, not a single magic number.
  FlightManager fm(config_);
  const auto& engines = fm.GetEngineManager();
  double vr_kts = engines.GetRotationSpeedKts();

  EXPECT_GT(vr_kts, 30.0) << "Vr unreasonably low for " << GetParam().model_name;
  EXPECT_LT(vr_kts, 400.0) << "Vr unreasonably high for " << GetParam().model_name;

  const std::string& model = GetParam().model_name;

  if (model == "c172x") {
    // GA piston: CLmax=1.6, Vr factor=1.10, Iyy=1346 (light)
    EXPECT_EQ(engines.GetType(), propulsion::EngineType::kPiston);
    // C172 Vr is typically 50-60 kts at MTOW
    EXPECT_GT(vr_kts, 40.0);
    EXPECT_LT(vr_kts, 80.0);
  } else if (model == "DHC6") {
    // Turboprop: CLmax=2.0, Vr factor=1.15, Iyy=24679
    EXPECT_EQ(engines.GetType(), propulsion::EngineType::kTurboprop);
    EXPECT_GT(vr_kts, 35.0);
    EXPECT_LT(vr_kts, 90.0);
  } else if (model == "B747") {
    // Heavy turbine: CLmax=1.6, Vr factor=1.08, Iyy=3.31e7
    EXPECT_EQ(engines.GetType(), propulsion::EngineType::kTurbine);
    EXPECT_GT(vr_kts, 100.0);
    EXPECT_LT(vr_kts, 200.0);
  } else if (model == "XB-70") {
    // Delta wing: CLmax=2.5 (AR=1.75), Vr factor=1.20 (light turbine)
    EXPECT_GT(vr_kts, 60.0);
    EXPECT_LT(vr_kts, 160.0);
  }
}

TEST_P(EngineManagerContractTest, ClimbPitchMatchesEngineType) {
  // Verify climb pitch is consistent with engine type classification.
  FlightManager fm(config_);
  const auto& engines = fm.GetEngineManager();
  double pitch_deg = engines.GetClimbPitchDeg();

  EXPECT_GT(pitch_deg, 5.0) << "Climb pitch unreasonably low";
  EXPECT_LT(pitch_deg, 25.0) << "Climb pitch unreasonably high";

  switch (engines.GetType()) {
    case propulsion::EngineType::kPiston:
      EXPECT_DOUBLE_EQ(pitch_deg, 10.0);
      break;
    case propulsion::EngineType::kTurbine:
      EXPECT_DOUBLE_EQ(pitch_deg, 15.0);
      break;
    case propulsion::EngineType::kTurboprop:
      EXPECT_DOUBLE_EQ(pitch_deg, 10.0);
      break;
    default:
      EXPECT_DOUBLE_EQ(pitch_deg, 10.0);  // fallback
      break;
  }
}

TEST_P(EngineManagerContractTest, ApproachSpeedFallbackReasonable) {
  // Verify approach speed fallback is physically plausible.
  // Computed as V_stall × 1.3 — derived from weight, wing area, and CLmax.
  // These are last-resort defaults when Vr calculation is unavailable.
  // Exact values vary with aircraft weight and CLmax; we verify physical bounds.
  FlightManager fm(config_);
  const auto& engines = fm.GetEngineManager();
  double spd_mps = engines.GetDefaultApproachSpeedMps();

  EXPECT_GT(spd_mps, 20.0) << "Approach speed unreasonably low";
  EXPECT_LT(spd_mps, 100.0) << "Approach speed unreasonably high";

  const std::string& model = GetParam().model_name;
  if (model == "c172x") {
    // GA piston: V_stall ≈ 26 m/s × 1.3 ≈ 34 m/s
    EXPECT_GT(spd_mps, 25.0);
    EXPECT_LT(spd_mps, 45.0);
  } else if (model == "DHC6") {
    // Turboprop, CLmax=2.0: V_stall ≈ 25 m/s × 1.3 ≈ 32 m/s
    EXPECT_GT(spd_mps, 22.0);
    EXPECT_LT(spd_mps, 50.0);
  } else if (model == "B747") {
    // Heavy turbine: V_stall ≈ 69 m/s × 1.3 ≈ 90 m/s (no-trim, near-empty weight)
    EXPECT_GT(spd_mps, 60.0);
    EXPECT_LT(spd_mps, 110.0);
  } else if (model == "XB-70") {
    // Delta wing, CLmax=2.5: V_stall ≈ 39 m/s × 1.3 ≈ 50 m/s
    EXPECT_GT(spd_mps, 30.0);
    EXPECT_LT(spd_mps, 70.0);
  }
}

TEST_F(FlightDynamicTest, DynamicTasEnvelopeTracksCommittedAtmosphereDensity) {
  FlightManager fm(config_);
  const auto initial = fm.GetAutopilot().GetControlProfile();
  const double initial_rho =
      fm.GetAdapter().GetProperty("atmosphere/rho-slugs_ft3");
  ASSERT_GT(initial_rho, 0.0);

  fm.GetAdapter().SetProperty("position/h-sl-ft", 3000.0 * kMToFt);
  ASSERT_TRUE(fm.GetAdapter().Run());
  const double lower_rho =
      fm.GetAdapter().GetProperty("atmosphere/rho-slugs_ft3");
  ASSERT_GT(lower_rho, 0.0);
  ASSERT_LT(lower_rho, initial_rho);
  fm.GetAutopilot().Update(kDt);
  const auto lower_density = fm.GetAutopilot().GetControlProfile();
  const double expected_ratio = std::sqrt(initial_rho / lower_rho);
  EXPECT_NEAR(lower_density.v_stall_mps / initial.v_stall_mps, expected_ratio, 1.0e-12);
  EXPECT_NEAR(lower_density.min_speed_mps / initial.min_speed_mps, expected_ratio,
              1.0e-12);
  EXPECT_NEAR(lower_density.cruise_speed_mps / initial.cruise_speed_mps,
              expected_ratio, 1.0e-12);
}

TEST_F(FlightDynamicTest, XmlSpeedOverrideRemainsFixedWhileOtherTasSpeedsRefresh) {
  FlightManager fm(config_);
  const auto initial = fm.GetAutopilot().GetControlProfile();
  const double initial_rho =
      fm.GetAdapter().GetProperty("atmosphere/rho-slugs_ft3");
  ASSERT_GT(initial_rho, 0.0);

  fm.GetAdapter().SetProperty("guidance/min-speed-mps", 42.0);
  fm.GetAdapter().SetProperty("position/h-sl-ft", 3000.0 * kMToFt);
  ASSERT_TRUE(fm.GetAdapter().Run());
  const double lower_rho =
      fm.GetAdapter().GetProperty("atmosphere/rho-slugs_ft3");
  ASSERT_GT(lower_rho, 0.0);
  ASSERT_LT(lower_rho, initial_rho);
  fm.GetAutopilot().Update(kDt);
  const auto refreshed = fm.GetAutopilot().GetControlProfile();
  EXPECT_DOUBLE_EQ(refreshed.min_speed_mps, 42.0);
  const double expected_ratio = std::sqrt(initial_rho / lower_rho);
  EXPECT_NEAR(refreshed.v_stall_mps / initial.v_stall_mps, expected_ratio, 1.0e-12);
  EXPECT_NEAR(refreshed.cruise_speed_mps / initial.cruise_speed_mps,
              expected_ratio, 1.0e-12);
}

TEST_F(FlightDynamicTest, InitialAltitudeChangesTasEnvelopeButNotCasTakeoffSpeeds) {
  config::FlightDynamicConfig high_config = config_;
  high_config.initial_kinematics.position_lla_deg_m.altitude_m = 3000.0;
  FlightManager low(config_);
  FlightManager high(high_config);

  const auto& low_profile = low.GetAutopilot().GetControlProfile();
  const auto& high_profile = high.GetAutopilot().GetControlProfile();
  EXPECT_GT(high_profile.v_stall_mps, low_profile.v_stall_mps);
  EXPECT_GT(high_profile.min_speed_mps, low_profile.min_speed_mps);
  EXPECT_DOUBLE_EQ(high.GetEngineManager().GetRotationSpeedKts(),
                   low.GetEngineManager().GetRotationSpeedKts());
  EXPECT_DOUBLE_EQ(high.GetEngineManager().GetDefaultApproachSpeedMps(),
                   low.GetEngineManager().GetDefaultApproachSpeedMps());
}

INSTANTIATE_TEST_SUITE_P(
    EngineManagerContracts, EngineManagerContractTest,
    ::testing::Values(
        EngineManagerContractParam{"c172x", 500.0, 50.0, true},
        EngineManagerContractParam{"DHC6", 500.0, 50.0, true},
        EngineManagerContractParam{"B747", 500.0, 100.0, false},
        EngineManagerContractParam{"XB-70", 500.0, 100.0, true}));

// ─── Thrust-to-Weight contract tests ─────────────────────────────────────
// Verify that TWR is physically plausible for each aircraft category.

TEST_P(EngineManagerContractTest, ThrustToWeightReasonable) {
  FlightManager fm(config_);
  const auto& engines = fm.GetEngineManager();
  double twr = engines.GetThrustToWeight();

  EXPECT_GE(twr, 0.0) << "TWR must be non-negative for " << GetParam().model_name;

  // TWR is estimated from current thrust/HP scaled by throttle position.
  // The estimate is approximate (thrust is not perfectly linear with throttle)
  // so bounds are generous.  The key property is that different aircraft
  // categories produce different TWR values (discrimination, not precision).
  const std::string& model = GetParam().model_name;
  if (model == "c172x") {
    EXPECT_GT(twr, 0.10) << "Piston TWR too low";
    EXPECT_LT(twr, 0.60) << "Piston TWR too high";
  } else if (model == "DHC6") {
    EXPECT_GT(twr, 0.08);
    EXPECT_LT(twr, 0.70);
  } else if (model == "B747") {
    EXPECT_GT(twr, 0.10);
    EXPECT_LT(twr, 0.50);
  } else if (model == "XB-70") {
    EXPECT_GT(twr, 0.08);
    EXPECT_LT(twr, 0.70);
  }

  // Also verify the profile's TWR matches EngineManager's.
  const auto& profile = fm.GetAutopilot().GetControlProfile();
  EXPECT_NEAR(profile.thrust_to_weight, twr, 0.01)
      << "Profile TWR should match EngineManager TWR";
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

TEST_F(FlightDynamicTest, WaypointManagerCompletesAfterPassingTargetPlane) {
  config_.do_trim = false;
  FlightManager fm(config_);
  auto& wpm = fm.GetWaypointManager();

  guidance::Waypoint wp;
  wp.latitude_rad = 100.0 / 6378137.0;
  wp.longitude_rad = 0.0;
  wp.altitude_m = 500.0;
  wp.radius_m = 10.0;
  wpm.AddWaypoint(wp);
  wpm.Start();

  const auto& current = fm.GetAdapter().GetPropagate().GetLocation();
  JSBSim::FGLocation passed_location = current;
  passed_location.SetPositionGeodetic(0.0, 150.0 / 6378137.0, current.GetGeodAltitude());
  fm.GetAdapter().GetPropagate().SetLocation(passed_location);

  EXPECT_GT(wpm.GetDistanceToActiveM(), wp.radius_m);
  EXPECT_TRUE(wpm.IsAtOrPastTarget())
      << "Waypoint should complete after crossing the normal plane at the target";
}

TEST(GreatCircleTrackGeometryTest, ResolvesHighLatitudeCrossTrackAgainstSphericalReference) {
  constexpr double kDegToRad = kPi / 180.0;
  const auto track = guidance::great_circle_track::ResolveTrackMetricsM(
      75.0 * kDegToRad, 0.0, 76.0 * kDegToRad, 0.1 * kDegToRad,
      76.05 * kDegToRad, 0.0);

  ASSERT_TRUE(track.valid);
  EXPECT_NEAR(std::fabs(track.cross_track_m), 2826.814, 1.0);
  EXPECT_NEAR(track.along_track_m - track.leg_length_m, 5496.944, 1.0);
}

TEST(GreatCircleTrackGeometryTest, UsesShortArcAcrossDateLine) {
  constexpr double kDegToRad = kPi / 180.0;
  const auto track = guidance::great_circle_track::ResolveTrackMetricsM(
      10.0 * kDegToRad, 179.8 * kDegToRad,
      10.0 * kDegToRad, -179.8 * kDegToRad,
      10.0 * kDegToRad, -179.7 * kDegToRad);

  ASSERT_TRUE(track.valid);
  EXPECT_GT(track.along_track_m, track.leg_length_m);
  EXPECT_LT(std::fabs(track.cross_track_m), 100.0);
}

TEST(GreatCircleTrackGeometryTest, RejectsDegenerateAndAntipodalLegs) {
  constexpr double kDegToRad = kPi / 180.0;
  EXPECT_FALSE(guidance::great_circle_track::ResolveTrackMetricsM(
                   0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
                   .valid);
  EXPECT_FALSE(guidance::great_circle_track::ResolveTrackMetricsM(
                   0.0, 0.0, 0.0, 180.0 * kDegToRad, 0.1, 0.0)
                   .valid);
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

TEST_F(FlightDynamicTest, FlyToMultipleWaypointsThenOrbit) {
  FlightManager fm(config_);

  struct WpCheck { double lat; double lon; double alt; double radius; };
  WpCheck waypoints[] = {
    {500.0 / 6.371e6,  500.0 / 6.371e6,  500.0, 100.0},
    {1000.0 / 6.371e6, 0.0,              500.0, 100.0},
    {0.0,              1000.0 / 6.371e6,  500.0, 100.0},
  };

  for (const auto& wp : waypoints) {
    ManeuverCommand fly;
    fly.type = guidance::ManeuverType::kFlyToWaypoint;
    fly.target.latitude_rad = wp.lat;
    fly.target.longitude_rad = wp.lon;
    fly.target.altitude_m = wp.alt;
    fly.target.radius_m = wp.radius;
    fm.PushManeuver(fly);
  }

  ManeuverCommand orbit;
  orbit.type = guidance::ManeuverType::kOrbit;
  orbit.target.latitude_rad = 1000.0 / 6.371e6;
  orbit.target.longitude_rad = 1000.0 / 6.371e6;
  orbit.target.altitude_m = 500.0;
  orbit.value = 500.0;
  fm.PushManeuver(orbit);

  // Run long enough to fly through all 3 waypoints + enter orbit
  RunSteps(fm, 10000);

  auto s = fm.GetState();
  EXPECT_TRUE(s == FlightManagerState::kExecuting)
      << "Should still be in orbit (kExecuting), got " << static_cast<int>(s);
  ExpectNoNaN(fm.GetVehicleState());
  EXPECT_GT(fm.GetVehicleState().sim_time_sec, 0.0);
}

TEST_F(FlightDynamicTest, TightSpacedWaypointRouteFlowsSequentially) {
  // 回归：航点间距小于转弯量级捕获圈时，航路不得在起步时被整条吞掉。
  // 旧语义下 c172x 捕获圈 = max(radius_m, 1.5×v²/(g·tan(max_bank))) ≈ 1.7-2.3 km，
  // 间距 566/707 m 的航点会在第 1 步全部"到达"（飞机未动即整条航路完成）。
  // 中间航点现按法平面穿越 / 到达半径（max(radius_m, 100 m)）完成，飞机必须真实
  // 飞过每个航点；最终航点保留转弯量级到达容差（机型相关，不同型号不可一概而论）。
  FlightManager fm(config_);

  // 起点 (0,0)：wp0 (400,400) 间距 566 m、wp1 (1000,0) 间距 707 m（旧捕获圈内，
  // 坍缩区间）；wp2 (9000,0) 末段 8 km（> 捕获圈，最终航点按捕获圈提前完成）。
  constexpr double kDegM = 1.0 / 6.371e6;
  struct Wp { double lat_rad; double lon_rad; };
  const Wp wps[] = {
      {400.0 * kDegM, 400.0 * kDegM},
      {1000.0 * kDegM, 0.0},
      {9000.0 * kDegM, 0.0},
  };
  for (const auto& wp : wps) {
    ManeuverCommand fly;
    fly.type = guidance::ManeuverType::kFlyToWaypoint;
    fly.target.latitude_rad = wp.lat_rad;
    fly.target.longitude_rad = wp.lon_rad;
    fly.target.altitude_m = 500.0;
    fm.PushManeuver(fly);
  }

  auto haversine_m = [](double lat1, double lon1, double lat2, double lon2) {
    constexpr double kEarthRadiusM = 6.371e6;
    const double dlat = lat2 - lat1;
    const double dlon = lon2 - lon1;
    const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                     std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    return 2.0 * kEarthRadiusM * std::asin(std::sqrt(a));
  };

  // 名义航路长度（起点 → 各航点折线）。
  double nominal_route_m = 0.0;
  {
    double prev_lat = 0.0;
    double prev_lon = 0.0;
    for (const auto& wp : wps) {
      nominal_route_m += haversine_m(prev_lat, prev_lon, wp.lat_rad, wp.lon_rad);
      prev_lat = wp.lat_rad;
      prev_lon = wp.lon_rad;
    }
  }

  // 沿轨迹累计飞行距离，通过航点完成事件记录（GetWaypointEvents，先于
  // ExecuteNextManeuver 追加）检测每次完成的时刻，记录完成第 i 个航点时已
  // 飞行的距离——坍缩时全部 ≈ 0（完成发生在起步瞬间），真实飞行时逐段增长。
  double path_at_completion[3] = {0.0, 0.0, 0.0};
  int completions = 0;
  double path_length_m = 0.0;
  double prev_lat = 0.0;
  double prev_lon = 0.0;
  bool have_prev = false;
  for (int i = 0; i < 20000; ++i) {
    if (!fm.Step(kDt)) break;
    const auto& s = fm.GetVehicleState();
    if (have_prev) {
      path_length_m += haversine_m(prev_lat, prev_lon, s.latitude_rad, s.longitude_rad);
    }
    prev_lat = s.latitude_rad;
    prev_lon = s.longitude_rad;
    have_prev = true;

    if (fm.GetWaypointEvents().size() > static_cast<std::size_t>(completions)) {
      path_at_completion[completions] = path_length_m;
      ++completions;
    }
    if (fm.GetState() == FlightManagerState::kCompleted) break;
  }

  EXPECT_EQ(completions, 3) << "三个航点应逐个完成";
  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted) << "紧间距航路应最终完成";
  // 每个航点完成前必须真实飞向它：坍缩时三个"完成"都在起步瞬间（飞行距离 ≈ 0）。
  EXPECT_GT(path_at_completion[0], 300.0) << "wp0 完成前必须飞行（566 m 航段；坍缩时为 0）";
  EXPECT_GT(path_at_completion[1] - path_at_completion[0], 500.0)
      << "wp1 完成前必须在 wp0 之后继续飞行";
  EXPECT_GT(path_at_completion[2] - path_at_completion[1], 1000.0)
      << "最终航点完成前必须沿长末段飞行（末段 8 km，捕获圈提前 ~2 km 完成）";
  EXPECT_GT(path_length_m, 0.6 * nominal_route_m) << "总飞行距离不得低于名义航路的 60%";
  EXPECT_GT(fm.GetVehicleState().sim_time_sec, 30.0) << "完成不应发生在起步瞬间";
}

TEST_F(FlightDynamicTest, WaypointSequencingEventsRecorded) {
  // 航点完成事件记录：中间航点（到达半径 100 m）与最终航点（转弯量级捕获圈）
  // 的事件应携带正确的门/阈值/索引/时间/中间语义。两个航点均取正北对头接近，
  // 保证 kWithinRadius 门确定性命中（无 S 形收敛导致的法平面穿越）。
  FlightManager fm(config_);

  constexpr double kDegM = 1.0 / 6.371e6;
  struct Wp { double lat_rad; double lon_rad; };
  const Wp wps[] = {
      {400.0 * kDegM, 0.0},   // 中间航点：400 m，到达半径 100 m
      {9000.0 * kDegM, 0.0},  // 最终航点：8.6 km，转弯量级捕获圈
  };
  for (const auto& wp : wps) {
    ManeuverCommand fly;
    fly.type = guidance::ManeuverType::kFlyToWaypoint;
    fly.target.latitude_rad = wp.lat_rad;
    fly.target.longitude_rad = wp.lon_rad;
    fly.target.altitude_m = 500.0;
    fm.PushManeuver(fly);
  }

  RunUntilDone(fm, 20000);
  ASSERT_EQ(fm.GetState(), FlightManagerState::kCompleted);

  const auto& events = fm.GetWaypointEvents();
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].waypoint_index, 0U);
  EXPECT_TRUE(events[0].intermediate) << "后继仍是 kFlyToWaypoint，应为中间航点语义";
  EXPECT_EQ(events[0].gate, guidance::WaypointCompletionGate::kWithinRadius);
  EXPECT_DOUBLE_EQ(events[0].threshold_m, 100.0) << "中间航点阈值为到达半径下限";
  EXPECT_LE(events[0].distance_m, events[0].threshold_m + 1e-6);
  EXPECT_GT(events[0].sim_time_sec, 0.0);

  EXPECT_EQ(events[1].waypoint_index, 1U);
  EXPECT_FALSE(events[1].intermediate) << "队列末尾应为最终航点语义";
  EXPECT_EQ(events[1].gate, guidance::WaypointCompletionGate::kWithinRadius);
  EXPECT_GT(events[1].threshold_m, 900.0) << "最终航点阈值应为转弯量级捕获圈";
  EXPECT_LE(events[1].distance_m, events[1].threshold_m + 1e-6);
  EXPECT_GT(events[1].sim_time_sec, events[0].sim_time_sec) << "事件应按完成顺序排列";

  // Reset 清空事件记录。
  fm.Reset(config_);
  EXPECT_TRUE(fm.GetWaypointEvents().empty());
}

TEST_F(FlightDynamicTest, NonFlyToCompletionsProduceNoWaypointEvents) {
  // 非 kFlyToWaypoint 的机动完成不应产生航点事件。
  FlightManager fm(config_);

  ManeuverCommand heading_cmd;
  heading_cmd.type = guidance::ManeuverType::kSetHeading;
  heading_cmd.value = 3.14;
  fm.PushManeuver(heading_cmd);

  RunUntilDone(fm, 8000);
  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted);
  EXPECT_TRUE(fm.GetWaypointEvents().empty());
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

TEST_F(FlightDynamicTest, ClearManeuversDoesNotInterruptCurrentManeuver) {
  FlightManager fm(config_);

  ManeuverCommand fly_cmd;
  fly_cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  fly_cmd.target.latitude_rad = 0.5;  // 远点：当前机动，短时间内不会完成
  fm.PushManeuver(fly_cmd);           // kReady → kExecuting 并派发（index 0 → 1）

  ManeuverCommand queued_cmd;
  queued_cmd.type = guidance::ManeuverType::kSetHeading;
  queued_cmd.value = 3.14;
  fm.PushManeuver(queued_cmd);  // kExecuting 下仅排队

  fm.ClearManeuvers();

  // 清空队列不影响当前正在执行的机动与诊断（对照 Abort 的 kAborted 语义）。
  EXPECT_EQ(fm.GetState(), FlightManagerState::kExecuting);
  EXPECT_EQ(fm.GetDiagnostics().outcome, ManeuverOutcome::kNone);

  RunSteps(fm, 100);  // 当前机动继续推进，未被清除
  EXPECT_EQ(fm.GetState(), FlightManagerState::kExecuting);
}

TEST_F(FlightDynamicTest, ClearManeuversDropsQueuedManeuversAndResetsExecutionIndex) {
  FlightManager fm(config_);

  // A：近距航点，首个机动，会正常完成（配置同 FlyToWaypointNearbyCompletesManeuver）。
  constexpr double kNearbyLat = 0.0 + (100.0 / 6.371e6);
  ManeuverCommand a_cmd;
  a_cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  a_cmd.target.latitude_rad = kNearbyLat;
  a_cmd.target.longitude_rad = 0.0;
  a_cmd.target.altitude_m = 500.0;
  fm.PushManeuver(a_cmd);  // 派发 A（index 0 → 1）

  // B：远点航点——若未被清空，A 完成后将长期执行，无法到达 kCompleted。
  ManeuverCommand b_cmd;
  b_cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  b_cmd.target.latitude_rad = 0.5;
  fm.PushManeuver(b_cmd);  // 仅排队

  fm.ClearManeuvers();  // 清空队列并复位执行索引

  // C：清空后重新入队——若索引未复位（仍为 1），C 会被跳过直接 kCompleted。
  ManeuverCommand c_cmd;
  c_cmd.type = guidance::ManeuverType::kSetHeading;
  c_cmd.value = 1.57;
  fm.PushManeuver(c_cmd);

  const int steps = RunUntilDone(fm, 20000);

  // A 完成后仅 C 待执行：全部完成且最后执行的是 C（B 从未执行）。
  EXPECT_EQ(fm.GetState(), FlightManagerState::kCompleted);
  EXPECT_EQ(fm.GetDiagnostics().current_type, guidance::ManeuverType::kSetHeading);
  EXPECT_LT(steps, 20000);
}

// ==========================================================================
// Profile snapshot test: captures ALL AircraftControlProfile fields for each
// target aircraft. Any change to the profile detection logic or aircraft XML
// that alters these values will produce a precise field-level diff.
// ==========================================================================

struct ProfileSnapshotParam {
  std::string model_name;
  double altitude_m;
  double speed_mps;
};

class ProfileSnapshotTest
    : public ::testing::TestWithParam<ProfileSnapshotParam> {
 protected:
  void InitConfig() {
    const auto& param = GetParam();
    config_.aircraft_model = param.model_name;
    config_.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
    config_.dt_sec = kDt;
    config_.do_trim = true;
    config_.silent_mode = true;
    config_.initial_kinematics.position_frame = coordinate::PositionFrame::kLla;
    config_.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
    config_.initial_kinematics.position_lla_deg_m.altitude_m = param.altitude_m;
    config_.initial_kinematics.velocity_mps.x_mps = param.speed_mps;
    config_.initial_kinematics.velocity_mps.y_mps = 0.0;
    config_.initial_kinematics.velocity_mps.z_mps = 0.0;
    config_.initial_kinematics.attitude_deg.roll_deg = 0.0;
    config_.initial_kinematics.attitude_deg.pitch_deg = 0.0;
    config_.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  }

  config::FlightDynamicConfig config_;
};

#define SNAPSHOT_CHECK_ENUM(profile, field, expected) \
  EXPECT_EQ(static_cast<int>(profile.field), static_cast<int>(expected))

#define SNAPSHOT_CHECK_BOOL(profile, field, expected) \
  EXPECT_EQ(profile.field, expected)

#define SNAPSHOT_CHECK_INT(profile, field, expected) \
  EXPECT_EQ(profile.field, expected)

#define SNAPSHOT_CHECK_STR(profile, field, expected) \
  EXPECT_EQ(profile.field, expected)

// 性能面派生量来自 JSBSim 气动/大气输入，属数值派生结果：不同构建配置
// （debug vs release+LTO）下末位浮点存在 ~1e-11 级差异（2026-08-05 在
// release 构建实测 f16 ref_speed 差异 2.1e-11），EXPECT_DOUBLE_EQ（4 ULP）
// 无法跨构建配置稳定；改为相对 1e-9 容差——仍能捕获真实回归（量级远大于
// 该噪声），且 debug/release 均稳定。
#define SNAPSHOT_CHECK_DBL(profile, field, expected) \
  EXPECT_NEAR(profile.field, expected, std::fabs(expected) * 1.0e-9)

TEST_P(ProfileSnapshotTest, MatchesExpectedProfile) {
  InitConfig();
  FlightManager fm(config_);
  const auto& p = fm.GetAutopilot().GetControlProfile();

  // Set per-aircraft expectations below.
  const std::string& model = GetParam().model_name;

  if (model == "f16") {
    SNAPSHOT_CHECK_ENUM(p, lateral_interface, autopilot::LateralControlInterface::kFbwRateCommand);
    SNAPSHOT_CHECK_ENUM(p, pitch_interface, autopilot::PitchControlInterface::kNativeAutopilot);
    SNAPSHOT_CHECK_ENUM(p, fbw_subtype, autopilot::FbwSubtype::kRollRatePid);
    SNAPSHOT_CHECK_BOOL(p, has_own_autopilot, false);
    SNAPSHOT_CHECK_BOOL(p, has_generic_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_fbw_override, true);
    SNAPSHOT_CHECK_BOOL(p, has_roll_rate_command, true);
    SNAPSHOT_CHECK_BOOL(p, has_aileron_command, true);
    SNAPSHOT_CHECK_BOOL(p, indexed_throttle, false);  // single engine
    SNAPSHOT_CHECK_INT(p, engine_count, 1);
    SNAPSHOT_CHECK_BOOL(p, has_mixture, false);
    SNAPSHOT_CHECK_STR(p, yaw_input_property, "fcs/rudder-cmd-norm");

  } else if (model == "c172x") {
    SNAPSHOT_CHECK_ENUM(p, lateral_interface, autopilot::LateralControlInterface::kOwnAutopilot);
    SNAPSHOT_CHECK_ENUM(p, pitch_interface, autopilot::PitchControlInterface::kNativeAutopilot);
    SNAPSHOT_CHECK_ENUM(p, fbw_subtype, autopilot::FbwSubtype::kNone);
    SNAPSHOT_CHECK_BOOL(p, has_own_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_generic_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_fbw_override, false);
    SNAPSHOT_CHECK_BOOL(p, has_roll_rate_command, false);
    SNAPSHOT_CHECK_BOOL(p, has_aileron_command, true);
    SNAPSHOT_CHECK_BOOL(p, indexed_throttle, false);  // single engine
    SNAPSHOT_CHECK_INT(p, engine_count, 1);
    SNAPSHOT_CHECK_BOOL(p, has_mixture, true);
    SNAPSHOT_CHECK_STR(p, yaw_input_property, "fcs/rudder-cmd-norm");

  } else if (model == "c310") {
    SNAPSHOT_CHECK_ENUM(p, lateral_interface, autopilot::LateralControlInterface::kOwnAutopilot);
    SNAPSHOT_CHECK_ENUM(p, pitch_interface, autopilot::PitchControlInterface::kNativeAutopilot);
    SNAPSHOT_CHECK_ENUM(p, fbw_subtype, autopilot::FbwSubtype::kNone);
    SNAPSHOT_CHECK_BOOL(p, has_own_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_generic_autopilot, false);
    SNAPSHOT_CHECK_BOOL(p, has_fbw_override, false);
    SNAPSHOT_CHECK_BOOL(p, has_roll_rate_command, false);
    SNAPSHOT_CHECK_BOOL(p, has_aileron_command, true);
    SNAPSHOT_CHECK_BOOL(p, indexed_throttle, true);   // twin-engine
    SNAPSHOT_CHECK_INT(p, engine_count, 2);
    SNAPSHOT_CHECK_BOOL(p, has_mixture, true);
    SNAPSHOT_CHECK_STR(p, yaw_input_property, "fcs/rudder-cmd-norm");

  } else if (model == "f15") {
    // has ap/autopilot-roll-on but guard rule downgrades to kDirectSurface
    // (twin-engine, no own AP, no FBW — single leaked AP property).
    SNAPSHOT_CHECK_ENUM(p, lateral_interface, autopilot::LateralControlInterface::kDirectSurface);
    SNAPSHOT_CHECK_ENUM(p, pitch_interface, autopilot::PitchControlInterface::kNativeAutopilot);
    SNAPSHOT_CHECK_ENUM(p, fbw_subtype, autopilot::FbwSubtype::kNone);
    SNAPSHOT_CHECK_BOOL(p, has_own_autopilot, false);
    SNAPSHOT_CHECK_BOOL(p, has_generic_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_fbw_override, false);
    SNAPSHOT_CHECK_BOOL(p, has_roll_rate_command, false);
    SNAPSHOT_CHECK_BOOL(p, has_aileron_command, true);
    SNAPSHOT_CHECK_BOOL(p, indexed_throttle, true);   // twin-engine
    SNAPSHOT_CHECK_INT(p, engine_count, 2);
    SNAPSHOT_CHECK_BOOL(p, has_mixture, false);
    SNAPSHOT_CHECK_STR(p, yaw_input_property, "fcs/rudder-cmd-norm");

  } else if (model == "Concorde") {
    SNAPSHOT_CHECK_ENUM(p, lateral_interface, autopilot::LateralControlInterface::kGenericAutopilotBridge);
    SNAPSHOT_CHECK_ENUM(p, pitch_interface, autopilot::PitchControlInterface::kNativeAutopilot);
    SNAPSHOT_CHECK_ENUM(p, fbw_subtype, autopilot::FbwSubtype::kNone);
    SNAPSHOT_CHECK_BOOL(p, has_own_autopilot, false);
    SNAPSHOT_CHECK_BOOL(p, has_generic_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_fbw_override, false);
    SNAPSHOT_CHECK_BOOL(p, has_roll_rate_command, false);
    SNAPSHOT_CHECK_BOOL(p, has_aileron_command, true);
    SNAPSHOT_CHECK_BOOL(p, indexed_throttle, true);   // quad-engine
    SNAPSHOT_CHECK_INT(p, engine_count, 4);
    SNAPSHOT_CHECK_BOOL(p, has_mixture, false);
    SNAPSHOT_CHECK_STR(p, yaw_input_property, "fcs/rudder-cmd-norm");

  } else if (model == "B17") {
    // B17 has project-injected Autopilot.xml -> kGenericAutopilotBridge
    SNAPSHOT_CHECK_ENUM(p, lateral_interface, autopilot::LateralControlInterface::kGenericAutopilotBridge);
    SNAPSHOT_CHECK_ENUM(p, pitch_interface, autopilot::PitchControlInterface::kNativeAutopilot);
    SNAPSHOT_CHECK_ENUM(p, fbw_subtype, autopilot::FbwSubtype::kNone);
    SNAPSHOT_CHECK_BOOL(p, has_own_autopilot, false);
    SNAPSHOT_CHECK_BOOL(p, has_generic_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_fbw_override, false);
    SNAPSHOT_CHECK_BOOL(p, has_roll_rate_command, false);
    SNAPSHOT_CHECK_BOOL(p, has_aileron_command, true);
    SNAPSHOT_CHECK_BOOL(p, indexed_throttle, true);   // quad-engine piston
    SNAPSHOT_CHECK_INT(p, engine_count, 4);
    SNAPSHOT_CHECK_BOOL(p, has_mixture, true);
    SNAPSHOT_CHECK_STR(p, yaw_input_property, "fcs/rudder-cmd-norm");

  } else if (model == "C130") {
    // C130 has project-injected Autopilot.xml -> kGenericAutopilotBridge
    SNAPSHOT_CHECK_ENUM(p, lateral_interface, autopilot::LateralControlInterface::kGenericAutopilotBridge);
    SNAPSHOT_CHECK_ENUM(p, pitch_interface, autopilot::PitchControlInterface::kNativeAutopilot);
    SNAPSHOT_CHECK_ENUM(p, fbw_subtype, autopilot::FbwSubtype::kNone);
    SNAPSHOT_CHECK_BOOL(p, has_own_autopilot, false);
    SNAPSHOT_CHECK_BOOL(p, has_generic_autopilot, true);
    SNAPSHOT_CHECK_BOOL(p, has_fbw_override, false);
    SNAPSHOT_CHECK_BOOL(p, has_roll_rate_command, false);
    SNAPSHOT_CHECK_BOOL(p, has_aileron_command, true);
    SNAPSHOT_CHECK_BOOL(p, indexed_throttle, true);   // quad-engine
    SNAPSHOT_CHECK_INT(p, engine_count, 4);
    SNAPSHOT_CHECK_BOOL(p, has_mixture, false);
    SNAPSHOT_CHECK_STR(p, yaw_input_property, "fcs/rudder-cmd-norm");

  } else {
    FAIL() << "Unknown model: " << model;
  }

  if (model == "Concorde") {
    // Classified as heavy jet (4 engines, Iyy=1.9e7, no mixture).
    // cruise_factor continuous: 2.89 + 0.00455 × WL ≈ 3.43, max_factor = CF + 0.7
    SNAPSHOT_CHECK_DBL(p, ref_speed_mps, 302.14528602514599);
    SNAPSHOT_CHECK_DBL(p, cruise_speed_mps, 250.20098363784268);
    SNAPSHOT_CHECK_DBL(p, min_speed_mps, 89.047375521091368);
    SNAPSHOT_CHECK_DBL(p, max_speed_mps, 302.14528602514599);
    SNAPSHOT_CHECK_DBL(p, ceiling_m, 13700.0);
    SNAPSHOT_CHECK_DBL(p, max_pitch_command_deg, 8.0);
    SNAPSHOT_CHECK_DBL(p, max_roll_angle_deg, 35.0);
    SNAPSHOT_CHECK_DBL(p, min_throttle, 0.55);
    SNAPSHOT_CHECK_BOOL(p, speed_energy_priority, true);
  } else if (model == "f16") {
    // FBW fighter: cruise_factor continuous + 0.25 FBW bonus → ~3.48
    SNAPSHOT_CHECK_DBL(p, ref_speed_mps, 366.86358488022273);
    SNAPSHOT_CHECK_DBL(p, cruise_speed_mps, 232.30605776652874);
    SNAPSHOT_CHECK_DBL(p, min_speed_mps, 84.09845444605871);
    SNAPSHOT_CHECK_DBL(p, max_speed_mps, 366.86358488022273);
    SNAPSHOT_CHECK_DBL(p, ceiling_m, 15200.0);
    SNAPSHOT_CHECK_DBL(p, max_pitch_command_deg, 15.0);
    SNAPSHOT_CHECK_DBL(p, max_roll_angle_deg, 45.0);
    SNAPSHOT_CHECK_DBL(p, min_throttle, 0.35);
    SNAPSHOT_CHECK_BOOL(p, speed_energy_priority, true);
  } else if (model == "f15") {
    // Non-FBW, non-piston, not heavy (2 engines, Iyy=1.65e5).
    // cruise_factor continuous: 2.89 + 0.00455 × WL ≈ 3.16, max_factor = CF + 0.8
    SNAPSHOT_CHECK_DBL(p, ref_speed_mps, 236.23972212006282);
    SNAPSHOT_CHECK_DBL(p, min_speed_mps, 77.973270056684228);
    SNAPSHOT_CHECK_DBL(p, max_pitch_command_deg, 20.0);
    EXPECT_NEAR(p.max_roll_angle_deg, 21.0, 0.5);  // from XML roll limit × sustained factor
    SNAPSHOT_CHECK_DBL(p, min_throttle, 0.15);
    // f15 wing_loading ≈ 55 lbs/ft² > 50 → speed_energy_priority = true.
    SNAPSHOT_CHECK_BOOL(p, speed_energy_priority, true);
  } else if (model == "B17") {
    SNAPSHOT_CHECK_DBL(p, ref_speed_mps, 156.86775448231597);
    SNAPSHOT_CHECK_DBL(p, cruise_speed_mps, 125.49420358585277);
    SNAPSHOT_CHECK_DBL(p, min_speed_mps, 58.265165950574506);
    SNAPSHOT_CHECK_DBL(p, max_speed_mps, 156.86775448231597);
    SNAPSHOT_CHECK_DBL(p, ceiling_m, 4300.0);
    SNAPSHOT_CHECK_DBL(p, max_pitch_command_deg, 10.0);
    SNAPSHOT_CHECK_DBL(p, max_roll_angle_deg, 25.0);
    SNAPSHOT_CHECK_DBL(p, min_throttle, 0.40);
    // B17 wing_loading ≈ 25 lbs/ft² < 50 → speed_energy_priority = false.
    SNAPSHOT_CHECK_BOOL(p, speed_energy_priority, false);
  } else if (model == "C130") {
    // Classified as heavy jet (4 engines, no magneto, no mixture).
    // cruise_factor continuous: 2.89 + 0.00455 × WL ≈ 3.20, max_factor = CF + 0.7
    SNAPSHOT_CHECK_DBL(p, ref_speed_mps, 164.71854621451828);
    SNAPSHOT_CHECK_DBL(p, cruise_speed_mps, 133.99413788498572);
    SNAPSHOT_CHECK_DBL(p, min_speed_mps, 52.670414279198674);
    SNAPSHOT_CHECK_DBL(p, max_speed_mps, 164.71854621451828);
    SNAPSHOT_CHECK_DBL(p, ceiling_m, 13700.0);
    SNAPSHOT_CHECK_DBL(p, max_pitch_command_deg, 8.0);
    SNAPSHOT_CHECK_DBL(p, max_roll_angle_deg, 35.0);
    SNAPSHOT_CHECK_DBL(p, min_throttle, 0.55);
    // C130 wing_loading ≈ 30 lbs/ft² < 50 → speed_energy_priority = false.
    SNAPSHOT_CHECK_BOOL(p, speed_energy_priority, false);
  } else if (model == "c172x") {
    SNAPSHOT_CHECK_DBL(p, ref_speed_mps, 94.615984112791907);
    SNAPSHOT_CHECK_DBL(p, cruise_speed_mps, 75.692787290233525);
    SNAPSHOT_CHECK_DBL(p, min_speed_mps, 35.143079813322707);
    SNAPSHOT_CHECK_DBL(p, max_speed_mps, 94.615984112791907);
    SNAPSHOT_CHECK_DBL(p, ceiling_m, 4300.0);
    SNAPSHOT_CHECK_DBL(p, max_pitch_command_deg, 12.0);
    EXPECT_NEAR(p.max_roll_angle_deg, 0.523 * 180.0 / kPi * 0.7, 1.0e-3);
    SNAPSHOT_CHECK_DBL(p, min_throttle, 0.20);
    SNAPSHOT_CHECK_BOOL(p, speed_energy_priority, false);
  } else if (model == "c310") {
    SNAPSHOT_CHECK_DBL(p, ref_speed_mps, 124.95075905640114);
    SNAPSHOT_CHECK_DBL(p, cruise_speed_mps, 99.960607245120897);
    SNAPSHOT_CHECK_DBL(p, min_speed_mps, 46.410281935234707);
    SNAPSHOT_CHECK_DBL(p, max_speed_mps, 124.95075905640114);
    SNAPSHOT_CHECK_DBL(p, ceiling_m, 4300.0);
    SNAPSHOT_CHECK_DBL(p, max_pitch_command_deg, 10.0);
    SNAPSHOT_CHECK_DBL(p, max_roll_angle_deg, 30.0);
    SNAPSHOT_CHECK_DBL(p, min_throttle, 0.30);
    // c310 wing_loading ≈ 22 lbs/ft² < 50 → speed_energy_priority = false.
    SNAPSHOT_CHECK_BOOL(p, speed_energy_priority, false);
  }
  SNAPSHOT_CHECK_DBL(p, max_throttle, 1.0);
  SNAPSHOT_CHECK_BOOL(p, landing_heavy_flare, false);  // only B747 XML enables it
}

INSTANTIATE_TEST_SUITE_P(
    AircraftProfiles, ProfileSnapshotTest,
    ::testing::Values(
        ProfileSnapshotParam{"f16", 3000.0, 200.0},
        ProfileSnapshotParam{"c172x", 500.0, 50.0},
        ProfileSnapshotParam{"c310", 500.0, 65.0},
        ProfileSnapshotParam{"f15", 3000.0, 200.0},
        ProfileSnapshotParam{"Concorde", 5000.0, 150.0},
        ProfileSnapshotParam{"B17", 1000.0, 80.0},
        ProfileSnapshotParam{"C130", 1000.0, 90.0}));

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
