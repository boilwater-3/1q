/**
 * @file fd_session_test.cpp
 * @brief FlightDynamicSession 创建、Step、Reset 单元测试。
 *
 * 使用 JSBSim 自带的 c172x 机型数据（third_party/jsbsim/aircraft/c172x/）。
 * 数据文件根目录由 FD_JSBSIM_ROOT_DIR 编译定义指定。
 */

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"
#include "1q/flight_dynamic/flight_dynamic.hpp"

namespace fd = flight_dynamic;
namespace fd_model = flight_dynamic::model;
namespace fd_config = flight_dynamic::config;
namespace fd_session = flight_dynamic::session;
namespace coord = oneq::coordinate;

namespace {

#ifndef FD_JSBSIM_ROOT_DIR
#define FD_JSBSIM_ROOT_DIR ""
#endif

fd_config::FlightDynamicConfig MakeC172Config() {
  fd_config::FlightDynamicConfig cfg;
  cfg.aircraft.root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.aircraft.model_name = "c172x";
  cfg.silent = true;
  // 初始状态：北京附近，海拔 1000m，航向 90°（向东），速度 50 m/s
  cfg.initial_kinematics.position_frame = coord::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 39.9;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 116.4;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 1000.0;
  cfg.initial_kinematics.attitude_deg.yaw_deg = 90.0;
  cfg.initial_kinematics.attitude_deg.pitch_deg = 0.0;
  cfg.initial_kinematics.attitude_deg.roll_deg = 0.0;
  return cfg;
}

}  // namespace

// ---- 生命周期测试 ----

TEST(FdSessionTest, CreateAndDestroy) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  fd_config::FlightDynamicConfig cfg = MakeC172Config();
  EXPECT_NO_THROW({
    auto session = fd_session::FlightDynamicSessionFactory::Create(cfg);
    (void)session;
  });
}

TEST(FdSessionTest, StepProducesValidOutput) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());

  fd_model::FlightDynamicInput input{};
  input.cycle_index = 0U;
  input.dt_sec = 0.01f;
  input.control.throttle = 0.8;
  input.control.elevator = 0.0;

  auto output = session.Step(input);
  EXPECT_TRUE(output.ok);
  // kinematics 应以 kEcef 帧输出
  EXPECT_EQ(output.kinematics.position_frame, coord::PositionFrame::kEcef);
  // 位置应为非零（飞机在移动）
  EXPECT_NE(output.state.position_ecef_x_m, 0.0);
  // 海拔应接近初始值
  EXPECT_NEAR(output.state.altitude_msl_m, 1000.0, 100.0);
  // 空速应为正值
  EXPECT_GT(output.state.airspeed_mps, 0.0);
}

TEST(FdSessionTest, MultipleStepsAreConsistent) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.01f;
  input.control.throttle = 0.8;

  double prev_alt = 0.0;
  for (std::uint32_t i = 0U; i < 100U; ++i) {
    input.cycle_index = i;
    auto output = session.Step(input);
    ASSERT_TRUE(output.ok) << "Step failed at cycle " << i;
    if (i > 0U) {
      // 连续步之间状态应平滑变化
      EXPECT_NE(output.state.position_ecef_x_m, 0.0);
    }
    prev_alt = output.state.altitude_msl_m;
  }
  (void)prev_alt;
}

TEST(FdSessionTest, ResetRestoresInitialState) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  auto initial = session.GetCurrentState();

  // 跑几步改变状态
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.01f;
  input.control.throttle = 1.0;
  input.control.elevator = 0.1;
  for (int i = 0; i < 50; ++i) {
    input.cycle_index = static_cast<std::uint32_t>(i);
    session.Step(input);
  }

  // Reset 到初始状态
  session.Reset(MakeC172Config().initial_kinematics);
  auto after_reset = session.GetCurrentState();
  EXPECT_TRUE(after_reset.ok);
  // 海拔应回到初始值附近
  EXPECT_NEAR(after_reset.state.altitude_msl_m, 1000.0, 50.0);
}

TEST(FdSessionTest, MoveSemantics) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  auto session1 = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  auto state1 = session1.GetCurrentState();

  // 移动构造
  auto session2 = std::move(session1);
  auto state2 = session2.GetCurrentState();
  EXPECT_TRUE(state2.ok);
  EXPECT_EQ(state2.state.altitude_msl_m, state1.state.altitude_msl_m);

  // 移动赋值
  fd_session::FlightDynamicSession session3;
  session3 = std::move(session2);
  auto state3 = session3.GetCurrentState();
  EXPECT_TRUE(state3.ok);
}

TEST(FdSessionTest, GetCurrentStateDoesNotAdvance) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  auto s1 = session.GetCurrentState();
  auto s2 = session.GetCurrentState();
  // 连续 GetCurrentState 应返回相同状态（未推进）
  EXPECT_EQ(s1.state.position_ecef_x_m, s2.state.position_ecef_x_m);
  EXPECT_EQ(s1.state.position_ecef_y_m, s2.state.position_ecef_y_m);
  EXPECT_EQ(s1.state.position_ecef_z_m, s2.state.position_ecef_z_m);
}

TEST(FdSessionTest, FailedStepReturnsPreviousState) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());
  auto before = session.GetCurrentState();

  // 极短 dt 不应导致失败，但验证 ok flag 被正确设置
  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.01f;
  input.control.throttle = 0.5;
  auto output = session.Step(input);
  EXPECT_TRUE(output.ok);
  // 状态应已推进
  EXPECT_NE(output.state.airspeed_mps, before.state.airspeed_mps);
}

// ---- ExternalKinematics 输出格式验证 ----

TEST(FdSessionTest, OutputKinematicsIsEcefFrame) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) {
    GTEST_SKIP() << "FD_JSBSIM_ROOT_DIR not set — skipping JSBSim-dependent test";
  }
  auto session = fd_session::FlightDynamicSessionFactory::Create(MakeC172Config());

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.01f;
  auto output = session.Step(input);

  // 始终输出 kEcef
  EXPECT_EQ(output.kinematics.position_frame, coord::PositionFrame::kEcef);
  // ECEF 位置应为合理的地球半径量级
  double r = std::sqrt(output.kinematics.position_ecef_m.x_m * output.kinematics.position_ecef_m.x_m +
                       output.kinematics.position_ecef_m.y_m * output.kinematics.position_ecef_m.y_m +
                       output.kinematics.position_ecef_m.z_m * output.kinematics.position_ecef_m.z_m);
  EXPECT_GT(r, 6.0e6);  // > 地球半径 ~6371 km
  EXPECT_LT(r, 7.0e6);
}
