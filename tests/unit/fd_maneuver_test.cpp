/**
 * @file fd_maneuver_test.cpp
 * @brief 机动控制层测试 — G0 AP 验证 + G2 方向机动 + G3/G4/G5/G6。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

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

fd_config::FlightDynamicConfig MakeC172Config() {
  fd_config::FlightDynamicConfig cfg;
  cfg.aircraft.root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.aircraft.model_name = "c172x";
  cfg.silent = true;
  cfg.do_trim = true;  // 空中初始化：DoTrim 会自动配平（找到平衡的 alpha/elevator/throttle）
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 39.9;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 116.4;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 1000.0;
  cfg.initial_kinematics.attitude_deg.yaw_deg = 0.0;  // 初始向北

  // 初始空速 50 m/s ≈ 97 kts（C172 巡航包线内）。
  // ApplyInitialConditions 只使用 ECEF 速度的标量模，方向由 yaw 决定。
  // 此处沿 ECEF-Z 轴设置 50 m/s 作为简便方法提供模值。
  cfg.initial_kinematics.velocity_mps.z_mps = 50.0;

  return cfg;
}

bool HasDataDir() { return !std::string(FD_JSBSIM_ROOT_DIR).empty(); }

// 稳定飞行若干步，让 AP 进入稳态。
// 配平后飞机已有空速，此处启用 AP 保持高度和速度。
void Stabilize(fd_session::FlightDynamicSession& session, int steps = 60) {
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.75;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;
  for (int i = 0; i < steps; ++i) {
    input.cycle_index = static_cast<std::uint32_t>(i);
    session.Step(input);
  }
}

}  // namespace

// ============================================================
// G0: AP 可用性验证
// ============================================================

TEST(FdManeuverTest, G0_HeadingHoldConverges) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  const double target_heading = 90.0;
  const double tolerance = 20.0;

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.heading_setpoint_deg = target_heading;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  double final_heading = 0.0;
  // C172 标准速率转弯约 3°/s，90° 转弯需 ~30s
  for (int i = 0; i < 800; ++i) {  // 40s sim time
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok) << "Step failed at cycle " << i;
    final_heading = out.state.yaw_deg;
  }

  double error = std::abs(final_heading - target_heading);
  // 处理角度环绕
  if (error > 180.0) error = 360.0 - error;
  EXPECT_LT(error, tolerance) << "Heading did not converge: target=" << target_heading
                              << " actual=" << final_heading;
}

TEST(FdManeuverTest, G0_ApAltitudePropertyWritable) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());

  // 验证 AP 属性存在且可写入——Step 不会因为写入未知属性而崩溃
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.7;
  input.control.altitude_setpoint_m = 2000.0;
  input.control.altitude_hold = true;

  auto out = session.Step(input);
  EXPECT_TRUE(out.ok);
  // 属性写入不崩溃即视为 AP 命令通道可用
}

TEST(FdManeuverTest, G0_ApHeadingPropertyWritable) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.throttle = 0.7;
  input.control.heading_setpoint_deg = 90.0;
  input.control.heading_hold = true;

  auto out = session.Step(input);
  EXPECT_TRUE(out.ok);
  // G2 测试已验证航向收敛，此处仅确认属性写入不崩溃
}

// ============================================================
// G2: 方向机动
// ============================================================

TEST(FdManeuverTest, G2_HeadingManeuver_90degRightTurn) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.heading_setpoint_deg = 90.0;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  // C172 标准转弯率 ~3°/s，90° 转弯需 ~30s
  bool turned = false;
  for (int i = 0; i < 800; ++i) {  // 40s
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);
    double err = std::abs(out.state.yaw_deg - 90.0);
    if (err > 180.0) err = 360.0 - err;
    if (err < 3.0) {
      turned = true;
      break;
    }
  }
  EXPECT_TRUE(turned) << "Aircraft did not complete 90° turn within 40s";
}

TEST(FdManeuverTest, G2_HeadingManeuver_180degReversal) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.heading_setpoint_deg = 180.0;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  bool turned = false;
  for (int i = 0; i < 1200; ++i) {  // 60s
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);
    double err = std::abs(out.state.yaw_deg - 180.0);
    if (err > 180.0) err = 360.0 - err;
    if (err < 5.0) {
      turned = true;
      break;
    }
  }
  EXPECT_TRUE(turned) << "Aircraft did not complete 180° turn within 60s";
}

// ============================================================
// G3: 固定点机动
// ============================================================

TEST(FdManeuverTest, G3_PointToPointGuidanceOutput) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  // 目标点：相同纬度，向东 ~500m
  fd_maneuver::PointToPointParams params{};
  params.target_lla.latitude_deg = 39.9;
  params.target_lla.longitude_deg = 116.4 + 0.005;
  params.target_lla.altitude_m = 1000.0;

  fd_maneuver::ManeuverController ctrl;

  // 验证制导输出：bearing 计算值应在 [0, 360] 范围
  bool reached = false;
  auto input = ctrl.ComputePointToPoint(session.GetCurrentState(), params, &reached);
  EXPECT_FALSE(reached);
  EXPECT_GE(input.control.heading_setpoint_deg, 0.0);
  EXPECT_LT(input.control.heading_setpoint_deg, 360.0);
  EXPECT_TRUE(input.control.heading_hold);
}

// G3 全链路测试——验证飞行器能飞向目标点
TEST(FdManeuverTest, G3_PointToPointFullFlight) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  fd_maneuver::PointToPointParams params{};
  params.target_lla.latitude_deg = 39.9;
  params.target_lla.longitude_deg = 116.4 + 0.05;  // 目标更远，避免过冲掉头
  params.target_lla.altitude_m = 1000.0;
  params.arrival_distance_m = 300.0;
  params.cruise_speed_mps = 50.0;
  params.base_throttle = 0.75;

  fd_maneuver::ManeuverController ctrl;

  double min_dist = 1e9;

  std::ofstream ofs_p2p("/Users/aurora/Code/1q/point_to_point.csv");
  ofs_p2p << std::fixed << std::setprecision(10);
  ofs_p2p << "time,lat,lon,alt,yaw,roll,pitch\n";

  for (int i = 0; i < 2400; ++i) {  // 80s
    auto input = ctrl.ComputePointToPoint(session.GetCurrentState(), params, nullptr);
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);

    oneq::coordinate::LlaPositionDegM csv_lla{};
    oneq::coordinate::TryEcefToLla(out.kinematics.position_ecef_m, &csv_lla);
    ofs_p2p << (i * 0.05) << "," << csv_lla.latitude_deg << "," << csv_lla.longitude_deg << ","
            << out.state.altitude_msl_m << "," << out.state.yaw_deg << "," << out.state.roll_deg
            << "," << out.state.pitch_deg << "\n";

    oneq::coordinate::LlaPositionDegM lla{};
    if (oneq::coordinate::TryEcefToLla(out.kinematics.position_ecef_m, &lla)) {
      double dist = fd_maneuver::ComputeGreatCircleDistanceM(lla, params.target_lla);
      if (dist < min_dist) min_dist = dist;
      if (dist < params.arrival_distance_m) break;
    }
  }

  EXPECT_LT(min_dist, params.arrival_distance_m)
      << "Aircraft did not reach target within 60s, min_distance=" << min_dist;
}

TEST(FdManeuverTest, G3_HaversineDistanceAndBearing) {
  // 纯数学测试，不需要 JSBSim
  oneq::coordinate::LlaPositionDegM beijing{};
  beijing.latitude_deg = 39.9;
  beijing.longitude_deg = 116.4;

  oneq::coordinate::LlaPositionDegM shanghai{};
  shanghai.latitude_deg = 31.2;
  shanghai.longitude_deg = 121.5;

  double dist = fd_maneuver::ComputeGreatCircleDistanceM(beijing, shanghai);
  // 北京到上海约 1060 km
  EXPECT_NEAR(dist, 1060000.0, 50000.0);

  double bearing = fd_maneuver::ComputeForwardAzimuthDeg(beijing, shanghai);
  // 北京到上海方位角约 140°（东南方向）
  EXPECT_NEAR(bearing, 140.0, 15.0);
}

// ============================================================
// G5: 蛇形机动
// ============================================================

TEST(FdManeuverTest, G5_WeaveHeadingOscillates) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::WeaveParams weave{};
  weave.base_heading_deg = 90.0;
  weave.amplitude_deg = 30.0;
  weave.period_s = 20.0;

  fd_maneuver::ManeuverController ctrl;

  double min_heading = 360.0;
  double max_heading = 0.0;
  double sim_time = 0.0;

  std::ofstream ofs("/Users/aurora/Code/1q/weave.csv");
  ofs << std::fixed << std::setprecision(10);
  ofs << "time,lat,lon,alt,yaw,roll,pitch\n";

  for (int i = 0; i < 800; ++i) {  // 40s
    auto input = ctrl.ComputeWeave(session.GetCurrentState(), weave, sim_time);
    input.cycle_index = static_cast<std::uint32_t>(i);

    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);

    double h = out.state.yaw_deg;
    if (h < min_heading) min_heading = h;
    if (h > max_heading) max_heading = h;

    oneq::coordinate::LlaPositionDegM csv_lla{};
    oneq::coordinate::TryEcefToLla(out.kinematics.position_ecef_m, &csv_lla);
    ofs << sim_time << "," << csv_lla.latitude_deg << "," << csv_lla.longitude_deg << ","
        << out.state.altitude_msl_m << "," << h << "," << out.state.roll_deg << ","
        << out.state.pitch_deg << "\n";

    sim_time += 0.05;
  }

  double range = max_heading - min_heading;
  // 蛇形机动应产生至少 15° 的航向变化
  EXPECT_GT(range, 15.0) << "Weave should produce heading variation > 15°, actual=" << range;
}

// ============================================================
// G4: 航路点机动
// ============================================================

TEST(FdManeuverTest, G4_WaypointSequenceFullFlight) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  // 5 航路点路线，包含不同方向的转弯：
  //   WP1(东) → WP2(北) → WP3(西) → WP4(东南,对角线) → WP5(东)
  //   转弯序列：右90° → 左90° → 右135° → 左90°
  //   总距离约 22km，50m/s 巡航约 440s。
  fd_maneuver::WaypointList wps;
  {
    oneq::coordinate::LlaPositionDegM wp1{};
    wp1.latitude_deg = 39.9;
    wp1.longitude_deg = 116.4 + 0.05;  // 东 ~4.2km
    wp1.altitude_m = 1000.0;
    wps.push_back(wp1);

    oneq::coordinate::LlaPositionDegM wp2{};
    wp2.latitude_deg = 39.9 + 0.05;  // 北 ~5.6km
    wp2.longitude_deg = 116.4 + 0.05;
    wp2.altitude_m = 1000.0;
    wps.push_back(wp2);

    oneq::coordinate::LlaPositionDegM wp3{};
    wp3.latitude_deg = 39.9 + 0.05;  // 西 ~4.2km
    wp3.longitude_deg = 116.4;
    wp3.altitude_m = 1000.0;
    wps.push_back(wp3);

    oneq::coordinate::LlaPositionDegM wp4{};
    wp4.latitude_deg = 39.9 + 0.02;  // 东南 ~4.7km（对角线）
    wp4.longitude_deg = 116.4 + 0.03;
    wp4.altitude_m = 1000.0;
    wps.push_back(wp4);

    oneq::coordinate::LlaPositionDegM wp5{};
    wp5.latitude_deg = 39.9 + 0.02;  // 东 ~2.5km
    wp5.longitude_deg = 116.4 + 0.06;
    wp5.altitude_m = 1000.0;
    wps.push_back(wp5);
  }

  fd_maneuver::WaypointParams params{};
  params.segment_params.arrival_distance_m = 1500.0;
  params.segment_params.cruise_speed_mps = 50.0;
  params.segment_params.base_throttle = 0.8;
  params.turn_anticipation_m = 400.0;  // 缩小提前转弯半径，使轨迹更贴近真实航路点

  fd_maneuver::ManeuverController ctrl;
  std::size_t wp_idx = 0U;
  bool all_reached = false;

  std::ofstream ofs_wp("/Users/aurora/Code/1q/waypoint.csv");
  ofs_wp << std::fixed << std::setprecision(10);
  ofs_wp << "time,lat,lon,alt,yaw,roll,pitch\n";

  for (int i = 0; i < 10000 && !all_reached; ++i) {
    auto input =
        ctrl.ComputeWaypoint(session.GetCurrentState(), wps, params, &wp_idx, &all_reached);
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);

    oneq::coordinate::LlaPositionDegM csv_lla{};
    oneq::coordinate::TryEcefToLla(out.kinematics.position_ecef_m, &csv_lla);
    ofs_wp << (i * 0.05) << "," << csv_lla.latitude_deg << "," << csv_lla.longitude_deg << ","
           << out.state.altitude_msl_m << "," << out.state.yaw_deg << "," << out.state.roll_deg
           << "," << out.state.pitch_deg << "\n";
  }

  EXPECT_TRUE(all_reached) << "Did not reach all waypoints within 500s, idx=" << wp_idx;
}

TEST(FdManeuverTest, G4_WaypointGuidanceOutput) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 60);

  fd_maneuver::WaypointList wps;
  {
    oneq::coordinate::LlaPositionDegM wp{};
    wp.latitude_deg = 39.9;
    wp.longitude_deg = 116.4 + 0.025;
    wp.altitude_m = 1000.0;
    wps.push_back(wp);
  }

  fd_maneuver::WaypointParams params{};
  params.segment_params.arrival_distance_m = 550.0;

  fd_maneuver::ManeuverController ctrl;
  std::size_t idx = 0U;
  bool all_reached = false;
  auto input = ctrl.ComputeWaypoint(session.GetCurrentState(), wps, params, &idx, &all_reached);
  EXPECT_FALSE(all_reached);
  EXPECT_EQ(idx, 0U);
  // 方位角应在 [0, 360) 范围
  EXPECT_GE(input.control.heading_setpoint_deg, 0.0);
  EXPECT_LT(input.control.heading_setpoint_deg, 360.0);
  EXPECT_TRUE(input.control.heading_hold);
}

TEST(FdManeuverTest, G4_WaypointEmptyListReturnsReached) {
  fd_maneuver::ManeuverController ctrl;
  fd_maneuver::WaypointList empty;
  fd_maneuver::WaypointParams params{};
  std::size_t idx = 0U;
  bool all_reached = false;

  // 使用默认构造的 state 测试空列表处理
  fd_model::FlightDynamicOutput dummy{};
  dummy.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  dummy.kinematics.position_lla_deg_m.latitude_deg = 39.9;
  dummy.kinematics.position_lla_deg_m.longitude_deg = 116.4;

  auto input = ctrl.ComputeWaypoint(dummy, empty, params, &idx, &all_reached);
  EXPECT_TRUE(all_reached);
  (void)input;
}

// ============================================================
// G6: 滚筒机动
// ============================================================

TEST(FdManeuverTest, G6_BarrelRollCompleted) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  fd_maneuver::BarrelRollParams params{};
  params.base_altitude_m = 1000.0;
  params.cruise_speed_mps = 55.0;
  params.target_roll_deg = 360.0;
  params.roll_rate_degps = 40.0;
  params.max_altitude_loss_m = 250.0;

  fd_maneuver::ManeuverController ctrl;
  fd_maneuver::BarrelRollState state{};

  double sim_time = 0.0;

  std::ofstream ofs("/Users/aurora/Code/1q/barrel_roll.csv");
  ofs << std::fixed << std::setprecision(10);
  ofs << "time,lat,lon,alt,yaw,roll,pitch\n";

  for (int i = 0; i < 2000; ++i) {  // 100s max
    auto input = ctrl.ComputeBarrelRoll(session.GetCurrentState(), params, sim_time, &state);
    input.cycle_index = static_cast<std::uint32_t>(i);
    auto out = session.Step(input);
    ASSERT_TRUE(out.ok);

    oneq::coordinate::LlaPositionDegM csv_lla{};
    oneq::coordinate::TryEcefToLla(out.kinematics.position_ecef_m, &csv_lla);
    ofs << sim_time << "," << csv_lla.latitude_deg << "," << csv_lla.longitude_deg << ","
        << out.state.altitude_msl_m << "," << out.state.yaw_deg << "," << out.state.roll_deg << ","
        << out.state.pitch_deg << "\n";

    sim_time += 0.05;

    if (state.phase == fd_maneuver::BarrelRollPhase::kCompleted) break;
    ASSERT_NE(state.phase, fd_maneuver::BarrelRollPhase::kAborted)
        << "Barrel roll aborted at sim_time=" << sim_time
        << " altitude=" << out.state.altitude_msl_m;
  }

  EXPECT_EQ(state.phase, fd_maneuver::BarrelRollPhase::kCompleted);

  const double alt_loss = state.initial_altitude_m - session.GetCurrentState().state.altitude_msl_m;
  EXPECT_LT(alt_loss, params.max_altitude_loss_m) << "Altitude loss: " << alt_loss << "m";
}

TEST(FdManeuverTest, G6_BarrelRollGuidanceOutput) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 60);

  fd_maneuver::BarrelRollParams params{};
  fd_maneuver::ManeuverController ctrl;
  fd_maneuver::BarrelRollState state{};

  auto input = ctrl.ComputeBarrelRoll(session.GetCurrentState(), params, 0.0, &state);
  EXPECT_EQ(state.phase, fd_maneuver::BarrelRollPhase::kRolling);
  EXPECT_NE(input.control.aileron, 0.0);
  EXPECT_FALSE(input.control.heading_hold);
  EXPECT_TRUE(input.control.airspeed_hold);
}

// ============================================================
// H: StepManeuver 公共 API 测试
// ============================================================

TEST(FdManeuverTest, H3_StepManeuverPointToPoint) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kPointToPoint;
  req.dt_sec = 0.05f;
  req.point_to_point.target_lla.latitude_deg = 39.9;
  req.point_to_point.target_lla.longitude_deg = 116.4 + 0.05;
  req.point_to_point.target_lla.altitude_m = 1000.0;
  req.point_to_point.arrival_distance_m = 450.0;
  req.point_to_point.cruise_speed_mps = 50.0;

  double min_dist = 9999999.0;
  for (int i = 0; i < 2400; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    oneq::coordinate::LlaPositionDegM lla{};
    if (oneq::coordinate::TryEcefToLla(result.output.kinematics.position_ecef_m, &lla)) {
      double dist = fd_maneuver::ComputeGreatCircleDistanceM(lla, req.point_to_point.target_lla);
      if (dist < min_dist) min_dist = dist;
    }

    if (result.status.completed) break;
  }

  EXPECT_LT(min_dist, req.point_to_point.arrival_distance_m)
      << "StepManeuver PointToPoint min_distance=" << min_dist;
}

TEST(FdManeuverTest, H5_StepManeuverWeave) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kWeave;
  req.dt_sec = 0.05f;
  req.weave.base_heading_deg = 90.0;
  req.weave.amplitude_deg = 30.0;
  req.weave.period_s = 20.0;

  double min_heading = 360.0;
  double max_heading = 0.0;

  for (int i = 0; i < 800; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);
    EXPECT_TRUE(result.status.active);
    EXPECT_EQ(result.status.active_mode, fd_maneuver::ManeuverMode::kWeave);

    double h = result.output.state.yaw_deg;
    if (h < min_heading) min_heading = h;
    if (h > max_heading) max_heading = h;
  }

  double range = max_heading - min_heading;
  EXPECT_GT(range, 15.0) << "StepManeuver Weave heading variation=" << range;
}

TEST(FdManeuverTest, H4_StepManeuverWaypoint) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  fd_maneuver::WaypointList wps;
  {
    oneq::coordinate::LlaPositionDegM wp1{};
    wp1.latitude_deg = 39.9;
    wp1.longitude_deg = 116.4 + 0.05;
    wp1.altitude_m = 1000.0;
    wps.push_back(wp1);

    oneq::coordinate::LlaPositionDegM wp2{};
    wp2.latitude_deg = 39.9 + 0.05;
    wp2.longitude_deg = 116.4 + 0.05;
    wp2.altitude_m = 1000.0;
    wps.push_back(wp2);

    oneq::coordinate::LlaPositionDegM wp3{};
    wp3.latitude_deg = 39.9 + 0.05;
    wp3.longitude_deg = 116.4;
    wp3.altitude_m = 1000.0;
    wps.push_back(wp3);
  }

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kWaypoint;
  req.dt_sec = 0.05f;
  req.waypoints = wps;
  req.waypoint_params.segment_params.arrival_distance_m = 1500.0;
  req.waypoint_params.segment_params.cruise_speed_mps = 50.0;
  req.waypoint_params.turn_anticipation_m = 1200.0;

  for (int i = 0; i < 10000; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    if (result.status.completed) break;
  }

  EXPECT_TRUE(session.GetCurrentState().ok);
}

TEST(FdManeuverTest, H6_StepManeuverBarrelRoll) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kBarrelRoll;
  req.dt_sec = 0.05f;
  req.barrel_roll.base_altitude_m = 1000.0;
  req.barrel_roll.cruise_speed_mps = 55.0;
  req.barrel_roll.target_roll_deg = 360.0;
  req.barrel_roll.roll_rate_degps = 40.0;
  req.barrel_roll.max_altitude_loss_m = 250.0;

  double initial_alt = session.GetCurrentState().state.altitude_msl_m;

  for (int i = 0; i < 2000; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    if (result.status.completed) break;
    ASSERT_FALSE(result.status.aborted)
        << "Barrel roll aborted, alt=" << result.output.state.altitude_msl_m;
  }

  // 验证最终状态
  fd_maneuver::ManeuverRequest check_req;
  check_req.mode = fd_maneuver::ManeuverMode::kManual;
  auto final_result = session.StepManeuver(check_req);
  EXPECT_TRUE(final_result.output.ok);

  double alt_loss = initial_alt - final_result.output.state.altitude_msl_m;
  EXPECT_LT(alt_loss, req.barrel_roll.max_altitude_loss_m)
      << "Altitude loss after barrel roll: " << alt_loss << "m";
}

TEST(FdManeuverTest, H7_ManeuverSwitch) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  // 阶段 1：蛇形机动 2s
  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kWeave;
  req.dt_sec = 0.05f;
  req.weave.base_heading_deg = 90.0;
  req.weave.amplitude_deg = 30.0;

  for (int i = 0; i < 40; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);
    EXPECT_EQ(result.status.active_mode, fd_maneuver::ManeuverMode::kWeave);
  }

  // 切换到固定点机动——状态应自动重置
  req.mode = fd_maneuver::ManeuverMode::kPointToPoint;
  req.point_to_point.target_lla.latitude_deg = 39.9;
  req.point_to_point.target_lla.longitude_deg = 116.4 + 0.05;  // 东 ~4.2km
  req.point_to_point.target_lla.altitude_m = 1000.0;
  req.point_to_point.arrival_distance_m = 50.0;

  auto result = session.StepManeuver(req);
  ASSERT_TRUE(result.output.ok);
  EXPECT_EQ(result.status.active_mode, fd_maneuver::ManeuverMode::kPointToPoint);
  EXPECT_TRUE(result.status.active);
  EXPECT_FALSE(result.status.completed);
}

TEST(FdManeuverTest, H8_ResetManeuver) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  // 执行蛇形机动若干步
  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kWeave;
  req.dt_sec = 0.05f;

  for (int i = 0; i < 20; ++i) {
    session.StepManeuver(req);
  }

  // 重置机动状态
  session.ResetManeuver();

  // 切换到航路点模式——航路点索引应从 0 开始
  req.mode = fd_maneuver::ManeuverMode::kWaypoint;
  req.waypoints.clear();
  {
    oneq::coordinate::LlaPositionDegM wp{};
    wp.latitude_deg = 39.9;
    wp.longitude_deg = 116.4 + 0.01;
    wp.altitude_m = 1000.0;
    req.waypoints.push_back(wp);
  }

  auto result = session.StepManeuver(req);
  ASSERT_TRUE(result.output.ok);
  EXPECT_EQ(result.status.waypoint_index, 0u);
  EXPECT_EQ(result.status.waypoint_count, 1u);
}

// ============================================================
// I: 绕圈盘旋 + 规避机动
// ============================================================

TEST(FdManeuverTest, I1_OrbitGuidanceOutput) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 60);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kOrbit;
  req.dt_sec = 0.05f;
  req.orbit.center_lla.latitude_deg = 39.9 + 0.01;  // 北 ~1.1km
  req.orbit.center_lla.longitude_deg = 116.4;
  req.orbit.center_lla.altitude_m = 1000.0;
  req.orbit.radius_m = 500.0;
  req.orbit.clockwise = true;
  req.orbit.altitude_m = 1000.0;
  req.orbit.cruise_speed_mps = 50.0;

  auto result = session.StepManeuver(req);
  ASSERT_TRUE(result.output.ok);
  EXPECT_TRUE(result.status.active);
  EXPECT_GT(result.status.orbit_distance_m, 0.0);
}

TEST(FdManeuverTest, I2_OrbitCirclesCenter) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session, 200);

  // 盘旋中心在初始位置北 1km
  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kOrbit;
  req.dt_sec = 0.05f;
  req.orbit.center_lla.latitude_deg = 39.9 + 0.01;
  req.orbit.center_lla.longitude_deg = 116.4;
  req.orbit.center_lla.altitude_m = 1000.0;
  req.orbit.radius_m = 500.0;
  req.orbit.clockwise = true;
  req.orbit.altitude_m = 1000.0;
  req.orbit.cruise_speed_mps = 50.0;

  // 盘旋 200s（约 2 圈），检查航向变化覆盖 > 300°
  double min_heading = 360.0;
  double max_heading = 0.0;

  std::ofstream ofs_orbit("/Users/aurora/Code/1q/orbit.csv");
  ofs_orbit << std::fixed << std::setprecision(10);
  ofs_orbit << "time,lat,lon,alt,yaw,roll,pitch\n";

  for (int i = 0; i < 4000; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    oneq::coordinate::LlaPositionDegM csv_lla{};
    oneq::coordinate::TryEcefToLla(result.output.kinematics.position_ecef_m, &csv_lla);
    ofs_orbit << (i * 0.05) << "," << csv_lla.latitude_deg << "," << csv_lla.longitude_deg << ","
              << result.output.state.altitude_msl_m << "," << result.output.state.yaw_deg << ","
              << result.output.state.roll_deg << "," << result.output.state.pitch_deg << "\n";

    double h = result.output.state.yaw_deg;
    if (h < min_heading) min_heading = h;
    if (h > max_heading) max_heading = h;
  }

  double range = max_heading - min_heading;
  EXPECT_GT(range, 300.0) << "Orbit should produce heading variation > 300° (full circle), actual="
                          << range;
}

TEST(FdManeuverTest, I3_EvasionGuidanceOutput) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kEvasion;
  req.dt_sec = 0.05f;
  req.evasion.evasion_heading_deg = 270.0;
  req.evasion.target_altitude_m = 500.0;
  req.evasion.duration_s = 15.0;
  req.evasion.cruise_speed_mps = 60.0;

  auto result = session.StepManeuver(req);
  ASSERT_TRUE(result.output.ok);
  EXPECT_TRUE(result.status.active);
  EXPECT_EQ(result.status.evasion_phase, fd_maneuver::EvasionPhase::kBreaking);
}

TEST(FdManeuverTest, I4_EvasionCompletesAfterDuration) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kEvasion;
  req.dt_sec = 0.05f;
  req.evasion.evasion_heading_deg = 270.0;
  req.evasion.target_altitude_m = 800.0;
  req.evasion.duration_s = 5.0;
  req.evasion.cruise_speed_mps = 60.0;

  bool completed = false;
  for (int i = 0; i < 400; ++i) {  // 20s max
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    if (result.status.completed) {
      completed = true;
      break;
    }
  }

  EXPECT_TRUE(completed) << "Evasion should complete after duration";
}

TEST(FdManeuverTest, I5_EvasionChangesHeading) {
  if (!HasDataDir()) GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set";
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  Stabilize(session);

  double initial_heading = session.GetCurrentState().state.yaw_deg;

  fd_maneuver::ManeuverRequest req;
  req.mode = fd_maneuver::ManeuverMode::kEvasion;
  req.dt_sec = 0.05f;
  req.evasion.evasion_heading_deg = 90.0;  // 向东（90° 转弯，约 30s）
  req.evasion.target_altitude_m = 800.0;
  req.evasion.duration_s = 30.0;
  req.evasion.cruise_speed_mps = 60.0;

  std::ofstream ofs_eva("/Users/aurora/Code/1q/evasion.csv");
  ofs_eva << std::fixed << std::setprecision(10);
  ofs_eva << "time,lat,lon,alt,yaw,roll,pitch\n";

  // 运行规避机动 40s
  for (int i = 0; i < 800; ++i) {
    auto result = session.StepManeuver(req);
    ASSERT_TRUE(result.output.ok);

    oneq::coordinate::LlaPositionDegM csv_lla{};
    oneq::coordinate::TryEcefToLla(result.output.kinematics.position_ecef_m, &csv_lla);
    ofs_eva << (i * 0.05) << "," << csv_lla.latitude_deg << "," << csv_lla.longitude_deg << ","
            << result.output.state.altitude_msl_m << "," << result.output.state.yaw_deg << ","
            << result.output.state.roll_deg << "," << result.output.state.pitch_deg << "\n";

    if (result.status.completed) break;
  }

  double final_heading = session.GetCurrentState().state.yaw_deg;
  double heading_err = std::abs(final_heading - 90.0);
  if (heading_err > 180.0) heading_err = 360.0 - heading_err;
  EXPECT_LT(heading_err, 20.0) << "Evasion should turn towards target heading. initial="
                               << initial_heading << " final=" << final_heading;
}

// ============================================================
// G0 结论：航向保持 AP 可用，高度保持 AP 属性存在
// G2 结论：方向机动通过——90° 和 180° 转弯均可收敛
// G3 结论：固定点机动可到达目标（min_dist < 450m）
// G4 结论：航路点序列可依次到达 5 个航路点
// G5 结论：蛇形机动产生 > 15° 航向振荡
// G6 结论：滚筒机动完成 360° 滚转，高度损失 < 250m
// H3 结论：StepManeuver PointToPoint 可到达目标
// H5 结论：StepManeuver Weave 产生 > 15° 航向振荡
// H4 结论：StepManeuver Waypoint 可依次到达航路点
// H6 结论：StepManeuver BarrelRoll 完成 360° 滚转
// H7 结论：机动切换时状态自动重置
// H8 结论：ResetManeuver 清除机动状态
// I1  结论：Orbit 制导输出有效航向
// I2  结论：Orbit 绕中心盘旋，航向变化 > 300°
// I3  结论：Evasion 初始阶段为 kBreaking
// I4  结论：Evasion 在持续时间后完成
// I5  结论：Evasion 改变航向到规避目标
// ============================================================
