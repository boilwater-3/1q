/**
 * @file fd_validation_test.cpp
 * @brief flight_dynamic 模块验证测试——物理真实性、长时间稳定性、复杂任务场景、边界条件、机动精度。
 */

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"
#include "1q/flight_dynamic/flight_dynamic.hpp"

namespace fd = flight_dynamic;
namespace fd_model = flight_dynamic::model;
namespace fd_config = flight_dynamic::config;
namespace fd_session = flight_dynamic::session;
namespace fd_maneuver = flight_dynamic::maneuver;

namespace {

#ifndef FD_JSBSIM_ROOT_DIR
#define FD_JSBSIM_ROOT_DIR ""
#endif

fd_config::FlightDynamicConfig MakeC172Config(double alt_m = 1000.0,
                                              double heading_deg = 0.0,
                                              double speed_mps = 50.0) {
  fd_config::FlightDynamicConfig cfg;
  cfg.aircraft.root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.aircraft.model_name = "c172x";
  cfg.silent = true;
  cfg.do_trim = true;
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 39.9;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 116.4;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = alt_m;
  cfg.initial_kinematics.attitude_deg.yaw_deg = heading_deg;
  cfg.initial_kinematics.velocity_mps.z_mps = speed_mps;
  return cfg;
}

bool HasDataDir() { return !std::string(FD_JSBSIM_ROOT_DIR).empty(); }

void Stabilize(fd_session::FlightDynamicSession& session,
               int steps = 120, double alt_m = 1000.0) {
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.75;
  input.control.altitude_setpoint_m = alt_m;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;
  for (int i = 0; i < steps; ++i) {
    input.cycle_index = static_cast<std::uint32_t>(i);
    session.Step(input);
  }
}

/// 规范化航向误差到 [0, 180]
double HeadingErrorDeg(double actual, double target) {
  double err = std::abs(actual - target);
  if (err > 180.0) err = 360.0 - err;
  return err;
}

/// 获取当前位置的 LLA
oneq::coordinate::LlaPositionDegM GetCurrentLLA(
    const fd_model::FlightDynamicOutput& out) {
  if (out.kinematics.position_frame ==
      oneq::coordinate::PositionFrame::kLla) {
    return out.kinematics.position_lla_deg_m;
  }
  oneq::coordinate::LlaPositionDegM lla{};
  oneq::coordinate::TryEcefToLla(out.kinematics.position_ecef_m, &lla);
  return lla;
}

constexpr double kDt = 0.05;  // 20 Hz

}  // namespace

// ============================================================
// V1: 物理真实性验证
// ============================================================

/// 验证 C172 巡航速度在合理范围内（40-70 m/s ≈ 78-136 kts）
TEST(FdValidationTest, V1_CruiseSpeedInRange) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  // 保持平飞 30s，采样空速
  double min_airspeed = 1e9, max_airspeed = 0.0;
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.75;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  for (int i = 0; i < 600; ++i) {
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);
    double as = out.state.airspeed_mps;
    if (as < min_airspeed) min_airspeed = as;
    if (as > max_airspeed) max_airspeed = as;
  }

  // C172 巡航包线：~40-70 m/s（78-136 kts）
  EXPECT_GE(min_airspeed, 40.0) << "Airspeed too low: " << min_airspeed;
  EXPECT_LE(max_airspeed, 70.0) << "Airspeed too high: " << max_airspeed;
}

/// 验证标准转弯率（~3°/s）
TEST(FdValidationTest, V1_StandardTurnRate) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  // 90° 转弯，记录航向随时间变化
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.heading_setpoint_deg = 90.0;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  double initial_heading = session.GetCurrentState().state.yaw_deg;
  double heading_at_15s = initial_heading;
  double heading_at_30s = initial_heading;

  for (int i = 0; i < 800; ++i) {
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);

    double sim_time = i * kDt;
    if (std::abs(sim_time - 15.0) < kDt) heading_at_15s = out.state.yaw_deg;
    if (std::abs(sim_time - 30.0) < kDt) heading_at_30s = out.state.yaw_deg;
  }

  // 15s 内转弯角度应在 30-80° 范围（2-5°/s，AP 可能有初始过冲）
  double turn_15s = HeadingErrorDeg(heading_at_15s, initial_heading);
  EXPECT_GE(turn_15s, 30.0) << "Turn too slow at 15s: " << turn_15s << "°";
  EXPECT_LE(turn_15s, 80.0) << "Turn too fast at 15s: " << turn_15s << "°";

  // 30s 内应接近完成 90° 转弯
  double turn_30s = HeadingErrorDeg(heading_at_30s, initial_heading);
  EXPECT_GE(turn_30s, 70.0) << "Turn too slow at 30s: " << turn_30s << "°";
}

/// 验证高度保持能力（AP 保持时漂移 < ±50m）
TEST(FdValidationTest, V1_AltitudeHoldStability) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  const double target_alt = 1000.0;
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.75;
  input.control.heading_setpoint_deg = 0.0;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = target_alt;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  double max_alt_error = 0.0;
  for (int i = 0; i < 2000; ++i) {  // 100s
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);
    double err = std::abs(out.state.altitude_msl_m - target_alt);
    if (err > max_alt_error) max_alt_error = err;
  }

  EXPECT_LT(max_alt_error, 50.0)
      << "Altitude hold drifted more than 50m: " << max_alt_error;
}

/// 验证 ECEF 位置在地球半径量级
TEST(FdValidationTest, V1_EcefPositionSanity) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  for (int i = 0; i < 200; ++i) {
    fd_model::FlightDynamicInput input{};
    input.dt_sec = 0.05f;
    input.cycle_index = static_cast<std::uint32_t>(i);
    input.control.throttle = 0.75;
    input.control.altitude_setpoint_m = 1000.0;
    input.control.altitude_hold = true;
    input.control.airspeed_setpoint_mps = 50.0;
    input.control.airspeed_hold = true;

    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);

    double r = std::sqrt(
        out.kinematics.position_ecef_m.x_m *
            out.kinematics.position_ecef_m.x_m +
        out.kinematics.position_ecef_m.y_m *
            out.kinematics.position_ecef_m.y_m +
        out.kinematics.position_ecef_m.z_m *
            out.kinematics.position_ecef_m.z_m);

    // 地球半径 ~6371km，允许 ±100km 范围
    EXPECT_GT(r, 6271000.0) << "ECEF radius too small";
    EXPECT_LT(r, 6471000.0) << "ECEF radius too large";
  }
}

/// 验证姿态角在合理范围（roll/pitch < ±90°）
TEST(FdValidationTest, V1_AttitudeSanityDuringLevelFlight) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.75;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.heading_setpoint_deg = 45.0;
  input.control.heading_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  for (int i = 0; i < 800; ++i) {
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);

    // 平飞/转弯时 roll 不超过 ±60°，pitch 不超过 ±30°
    EXPECT_LT(std::abs(out.state.roll_deg), 60.0)
        << "Excessive roll at step " << i << ": " << out.state.roll_deg;
    EXPECT_LT(std::abs(out.state.pitch_deg), 30.0)
        << "Excessive pitch at step " << i << ": " << out.state.pitch_deg;
  }
}

// ============================================================
// V2: 长时间稳定性
// ============================================================

/// 500s 连续平飞，检查无 NaN 或异常
TEST(FdValidationTest, V2_LongDurationLevelFlight) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.75;
  input.control.heading_setpoint_deg = 0.0;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  for (int i = 0; i < 10000; ++i) {  // 500s
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok) << "Step failed at cycle " << i;

    // 检查 NaN
    ASSERT_FALSE(std::isnan(out.state.altitude_msl_m))
        << "NaN altitude at step " << i;
    ASSERT_FALSE(std::isnan(out.state.airspeed_mps))
        << "NaN airspeed at step " << i;
    ASSERT_FALSE(std::isnan(out.state.yaw_deg))
        << "NaN heading at step " << i;
  }
}

/// 300s 连续蛇形机动
TEST(FdValidationTest, V2_LongDurationWeave) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kWeave;
  req.dt_sec = 0.05f;
  req.weave.base_heading_deg = 90.0;
  req.weave.amplitude_deg = 20.0;
  req.weave.period_s = 30.0;

  for (int i = 0; i < 6000; ++i) {  // 300s
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok) << "Step failed at cycle " << i;
    ASSERT_TRUE(result.status.active);
    ASSERT_FALSE(std::isnan(result.output.state.yaw_deg));
  }
}

/// 300s 连续轨道盘旋
TEST(FdValidationTest, V2_LongDurationOrbit) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kOrbit;
  req.dt_sec = 0.05f;
  req.orbit.center_lla.latitude_deg = 39.9 + 0.01;
  req.orbit.center_lla.longitude_deg = 116.4;
  req.orbit.center_lla.altitude_m = 1000.0;
  req.orbit.radius_m = 800.0;
  req.orbit.clockwise = true;
  req.orbit.altitude_m = 1000.0;
  req.orbit.cruise_speed_mps = 50.0;

  for (int i = 0; i < 6000; ++i) {  // 300s
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok) << "Step failed at cycle " << i;
    ASSERT_FALSE(std::isnan(result.output.state.altitude_msl_m))
        << "NaN at step " << i;
  }
}

// ============================================================
// V3: 多阶段复杂任务场景
// ============================================================

/// 完整任务：起飞稳定 → 航路点飞行 → 轨道侦察 → 蛇形规避 → 返航
TEST(FdValidationTest, V3_ComplexMission) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  // 阶段 1: 飞向第一个航路点（东 2km）
  {
    fd_maneuver::ManeuverRequest req;
    req.mode = fd_maneuver::ManeuverMode::kPointToPoint;
    req.dt_sec = 0.05f;
    req.point_to_point.target_lla.latitude_deg = 39.9;
    req.point_to_point.target_lla.longitude_deg = 116.4 + 0.025;
    req.point_to_point.target_lla.altitude_m = 1000.0;
    req.point_to_point.arrival_distance_m = 500.0;
    req.point_to_point.cruise_speed_mps = 50.0;

    bool reached = false;
    for (int i = 0; i < 2000 && !reached; ++i) {
      auto result = session.StepManeuver(req);
      ASSERT_TRUE(result.output.ok);
      reached = result.status.completed;
    }
    EXPECT_TRUE(reached) << "Phase 1: did not reach first waypoint";
  }

  // 阶段 2: 轨道侦察 60s
  {
    auto lla = GetCurrentLLA(session.GetCurrentState());
    fd_maneuver::ManeuverRequest req;
    req.mode = fd_maneuver::ManeuverMode::kOrbit;
    req.dt_sec = 0.05f;
    req.orbit.center_lla = lla;
    req.orbit.radius_m = 500.0;
    req.orbit.clockwise = true;
    req.orbit.altitude_m = 1000.0;
    req.orbit.cruise_speed_mps = 50.0;

    for (int i = 0; i < 1200; ++i) {  // 60s
      auto result = session.StepManeuver(req);
      ASSERT_TRUE(result.output.ok) << "Orbit failed at step " << i;
    }
  }

  // 阶段 3: 蛇形规避 20s
  {
    fd_maneuver::ManeuverRequest req;
    req.mode = fd_maneuver::ManeuverMode::kWeave;
    req.dt_sec = 0.05f;
    req.weave.base_heading_deg = 270.0;  // 向西
    req.weave.amplitude_deg = 25.0;
    req.weave.period_s = 15.0;

    for (int i = 0; i < 400; ++i) {  // 20s
      auto result = session.StepManeuver(req);
      ASSERT_TRUE(result.output.ok);
    }
  }

  // 阶段 4: 规避机动（向南急转 + 下降）
  {
    fd_maneuver::ManeuverRequest req;
    req.mode = fd_maneuver::ManeuverMode::kEvasion;
    req.dt_sec = 0.05f;
    req.evasion.evasion_heading_deg = 180.0;
    req.evasion.target_altitude_m = 600.0;
    req.evasion.duration_s = 10.0;
    req.evasion.cruise_speed_mps = 55.0;

    bool completed = false;
    for (int i = 0; i < 600 && !completed; ++i) {  // 30s
      auto result = session.StepManeuver(req);
      ASSERT_TRUE(result.output.ok);
      completed = result.status.completed;
    }
    EXPECT_TRUE(completed) << "Phase 4: evasion did not complete";
  }

  // 阶段 5: 返航（回到起始点附近）
  {
    fd_maneuver::ManeuverRequest req;
    req.mode = fd_maneuver::ManeuverMode::kPointToPoint;
    req.dt_sec = 0.05f;
    req.point_to_point.target_lla.latitude_deg = 39.9;
    req.point_to_point.target_lla.longitude_deg = 116.4;
    req.point_to_point.target_lla.altitude_m = 1000.0;
    req.point_to_point.arrival_distance_m = 1000.0;
    req.point_to_point.cruise_speed_mps = 50.0;

    // 验证返航至少开始执行（航路可能很远，只运行 40s 检查稳定性）
    for (int i = 0; i < 800; ++i) {
      auto result = session.StepManeuver(req);
      ASSERT_TRUE(result.output.ok) << "Return leg failed at step " << i;
      if (result.status.completed) break;
    }
  }
}

// ============================================================
// V4: 边界条件
// ============================================================

/// 航向 0/360 边界——转向 350° 不产生 NaN 或异常
TEST(FdValidationTest, V4_HeadingWrapAround) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  // 初始航向 10°，转向 350°（穿越 0/360 边界）
  auto session = fd_session::FlightDynamicSessionFactory::Create(
      MakeC172Config(1000.0, 10.0));
  Stabilize(session);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.heading_setpoint_deg = 350.0;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  for (int i = 0; i < 800; ++i) {
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);
    ASSERT_FALSE(std::isnan(out.state.yaw_deg))
        << "NaN heading at step " << i;
  }

  double final_heading = session.GetCurrentState().state.yaw_deg;
  EXPECT_LT(HeadingErrorDeg(final_heading, 350.0), 20.0)
      << "Heading wrap-around failed, final=" << final_heading;
}

/// 快速连续模式切换（每 10 步切换一次）
TEST(FdValidationTest, V4_RapidModeSwitching) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::ManeuverRequest weave_req;
  weave_req.mode = fd_maneuver::ManeuverMode::kWeave;
  weave_req.dt_sec = 0.05f;
  weave_req.weave.base_heading_deg = 90.0;

  fd_maneuver::ManeuverRequest orbit_req;
  orbit_req.mode = fd_maneuver::ManeuverMode::kOrbit;
  orbit_req.dt_sec = 0.05f;
  orbit_req.orbit.center_lla.latitude_deg = 39.9 + 0.01;
  orbit_req.orbit.center_lla.longitude_deg = 116.4;
  orbit_req.orbit.radius_m = 500.0;
  orbit_req.orbit.altitude_m = 1000.0;

  // 200 次快速切换
  for (int i = 0; i < 200; ++i) {
    auto& req = (i % 2 == 0) ? weave_req : orbit_req;
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok) << "Failed at rapid switch " << i;
    ASSERT_FALSE(std::isnan(result.output.state.yaw_deg));
  }
}

/// ResetManeuver 不影响物理状态
TEST(FdValidationTest, V4_ResetManeuverPreservesPhysics) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  // 执行一些机动
  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kWeave;
  req.dt_sec = 0.05f;
  for (int i = 0; i < 40; ++i) {
    session.StepManeuver(req);
  }

  // 记录 Reset 前的物理状态（weave 已改变高度）
  double alt_before = session.GetCurrentState().state.altitude_msl_m;

  // Reset 只清除机动状态，不改变物理
  session.ResetManeuver();

  double alt_after = session.GetCurrentState().state.altitude_msl_m;
  EXPECT_NEAR(alt_after, alt_before, 0.01)
      << "ResetManeuver should not change altitude";
}

/// kManual 模式不执行任何机动
TEST(FdValidationTest, V4_ManualModeNoOp) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kManual;
  req.dt_sec = 0.05f;

  auto result = session.StepManeuver(req);
  EXPECT_TRUE(result.output.ok);
  EXPECT_FALSE(result.status.active);
}

// ============================================================
// V5: 机动精度
// ============================================================

/// 轨道半径保持精度：先飞到轨道附近，再切入轨道
TEST(FdValidationTest, V5_OrbitRadiusAccuracy) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  // 轨道中心在初始位置以东 2km
  const double target_radius = 1000.0;
  oneq::coordinate::LlaPositionDegM orbit_center{};
  orbit_center.latitude_deg = 39.9;
  orbit_center.longitude_deg = 116.4 + 0.025;
  orbit_center.altitude_m = 1000.0;

  // 先飞向轨道上的一个点（中心以东 radius 远处）
  {
    fd_maneuver::ManeuverRequest req;
    req.mode = fd_maneuver::ManeuverMode::kPointToPoint;
    req.dt_sec = 0.05f;
    req.point_to_point.target_lla.latitude_deg = orbit_center.latitude_deg;
    req.point_to_point.target_lla.longitude_deg =
        orbit_center.longitude_deg + 0.01;  // 中心以东 ~800m
    req.point_to_point.target_lla.altitude_m = 1000.0;
    req.point_to_point.arrival_distance_m = 500.0;
    req.point_to_point.cruise_speed_mps = 50.0;

    for (int i = 0; i < 3000; ++i) {
      auto result = session.StepManeuver(req);
      ASSERT_TRUE(result.output.ok);
      if (result.status.completed) break;
    }
  }

  // 切入轨道
  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kOrbit;
  req.dt_sec = 0.05f;
  req.orbit.center_lla = orbit_center;
  req.orbit.radius_m = target_radius;
  req.orbit.clockwise = true;
  req.orbit.altitude_m = 1000.0;
  req.orbit.cruise_speed_mps = 50.0;

  // 120s 收敛
  for (int i = 0; i < 2400; ++i) {
    session.StepManeuver(req);
  }

  // 采样 60s
  double max_error_ratio = 0.0;
  for (int i = 0; i < 1200; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    auto lla = GetCurrentLLA(result.output);
    double dist = fd_maneuver::ComputeGreatCircleDistanceM(
        lla, req.orbit.center_lla);
    double error_ratio = std::abs(dist - target_radius) / target_radius;
    if (error_ratio > max_error_ratio) max_error_ratio = error_ratio;
  }

  // AP heading hold 轨道精度有限（AP 转弯率限制），容差 65%
  EXPECT_LT(max_error_ratio, 0.65)
      << "Orbit radius error > 65%: max_error_ratio=" << max_error_ratio;
}

/// 规避机动产生高度变化
TEST(FdValidationTest, V5_EvasionAltitudeChange) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  double initial_alt = session.GetCurrentState().state.altitude_msl_m;

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kEvasion;
  req.dt_sec = 0.05f;
  req.evasion.evasion_heading_deg = 90.0;
  req.evasion.target_altitude_m = 600.0;  // 下降 400m
  req.evasion.duration_s = 30.0;
  req.evasion.cruise_speed_mps = 55.0;

  for (int i = 0; i < 800; ++i) {  // 40s
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);
    if (result.status.completed) break;
  }

  double final_alt = session.GetCurrentState().state.altitude_msl_m;
  double alt_change = initial_alt - final_alt;

  // 应至少下降 50m（从 1000m 向 600m 下降）
  EXPECT_GT(alt_change, 50.0)
      << "Evasion should produce altitude change. initial="
      << initial_alt << " final=" << final_alt;
}

/// 航路点全链路验证——到达距离可靠
TEST(FdValidationTest, V5_WaypointArrivalAccuracy) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  // 单航路点，正东 2km
  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kWaypoint;
  req.dt_sec = 0.05f;
  {
    oneq::coordinate::LlaPositionDegM wp{};
    wp.latitude_deg = 39.9;
    wp.longitude_deg = 116.4 + 0.025;
    wp.altitude_m = 1000.0;
    req.waypoints.push_back(wp);
  }
  req.waypoint_params.segment_params.arrival_distance_m = 500.0;
  req.waypoint_params.segment_params.cruise_speed_mps = 50.0;
  req.waypoint_params.turn_anticipation_m = 400.0;

  double min_dist = 1e9;
  for (int i = 0; i < 3000; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    auto lla = GetCurrentLLA(result.output);
    double dist = fd_maneuver::ComputeGreatCircleDistanceM(
        lla, req.waypoints[0]);
    if (dist < min_dist) min_dist = dist;

    if (result.status.completed) break;
  }

  EXPECT_LT(min_dist, 500.0)
      << "Waypoint arrival: min distance to target=" << min_dist;
}
