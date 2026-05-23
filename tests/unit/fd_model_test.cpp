/**
 * @file fd_model_test.cpp
 * @brief VehicleStateMapper 和 model 层结构的单元测试。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/model/FlightDynamicInput.h"
#include "1q/flight_dynamic/model/VehicleState.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/model/VehicleStateMapper.h"

namespace fd_model = flight_dynamic::model;
namespace fd_adapter = flight_dynamic::adapter;
namespace fd_config = flight_dynamic::config;
namespace coord = oneq::coordinate;

namespace {

#ifndef FD_JSBSIM_ROOT_DIR
#define FD_JSBSIM_ROOT_DIR ""
#endif

fd_config::FlightDynamicConfig MakeModelConfig() {
  fd_config::FlightDynamicConfig cfg;
  cfg.aircraft.root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.aircraft.model_name = "c172x";
  cfg.silent = true;
  cfg.do_trim = false;
  cfg.initial_kinematics.position_frame = coord::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 39.9;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 116.4;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 1000.0;
  // Use heading to verify attitude mapping
  cfg.initial_kinematics.attitude_deg.yaw_deg = 90.0;
  cfg.initial_kinematics.velocity_mps.z_mps = 50.0;
  return cfg;
}

}  // namespace

TEST(FdModelTest, VehicleStateMapperProducesCorrectOutput) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) GTEST_SKIP() << "No JSBSim dir";

  fd_config::FlightDynamicConfig cfg = MakeModelConfig();
  fd_adapter::JsbsimAdapter adapter(cfg);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  adapter.Run(input);

  fd_model::VehicleStateMapper mapper;
  auto output =
      mapper.Map(adapter.GetPropagate(), adapter.GetAccelerations(), adapter.GetFdmExec());

  EXPECT_TRUE(output.ok);

  // Verify basic LLA (since we initialized at 39.9, 116.4, 1000)
  EXPECT_NEAR(output.state.altitude_msl_m, 1000.0, 50.0);

  // Verify attitude mapping
  // We started at 90 deg yaw
  EXPECT_NEAR(output.state.yaw_deg, 90.0, 1.0);

  // ECEF kinematics should match output state exactly
  EXPECT_EQ(output.kinematics.position_frame, coord::PositionFrame::kEcef);
  EXPECT_DOUBLE_EQ(output.kinematics.position_ecef_m.x_m, output.state.position_ecef_x_m);
  EXPECT_DOUBLE_EQ(output.kinematics.position_ecef_m.y_m, output.state.position_ecef_y_m);
  EXPECT_DOUBLE_EQ(output.kinematics.position_ecef_m.z_m, output.state.position_ecef_z_m);

  EXPECT_DOUBLE_EQ(output.kinematics.velocity_mps.x_mps, output.state.velocity_ecef_x_mps);
  EXPECT_DOUBLE_EQ(output.kinematics.velocity_mps.y_mps, output.state.velocity_ecef_y_mps);
  EXPECT_DOUBLE_EQ(output.kinematics.velocity_mps.z_mps, output.state.velocity_ecef_z_mps);

  EXPECT_DOUBLE_EQ(output.kinematics.attitude_deg.yaw_deg, output.state.yaw_deg);
  EXPECT_DOUBLE_EQ(output.kinematics.attitude_deg.pitch_deg, output.state.pitch_deg);
  EXPECT_DOUBLE_EQ(output.kinematics.attitude_deg.roll_deg, output.state.roll_deg);

  // Alpha and Beta shouldn't be nan
  EXPECT_FALSE(std::isnan(output.state.alpha_deg));
  EXPECT_FALSE(std::isnan(output.state.beta_deg));
}

TEST(FdModelTest, DefaultFlightDynamicInputValues) {
  fd_model::FlightDynamicInput input;
  EXPECT_EQ(input.cycle_index, 0U);
  EXPECT_FLOAT_EQ(input.dt_sec, 0.01f);

  EXPECT_DOUBLE_EQ(input.control.throttle, 0.0);
  EXPECT_DOUBLE_EQ(input.control.heading_setpoint_deg, -1.0);
  EXPECT_FALSE(input.control.heading_hold);
  EXPECT_DOUBLE_EQ(input.control.altitude_setpoint_m, -1.0);
  EXPECT_FALSE(input.control.altitude_hold);
  EXPECT_DOUBLE_EQ(input.control.airspeed_setpoint_mps, -1.0);
  EXPECT_FALSE(input.control.airspeed_hold);

  EXPECT_DOUBLE_EQ(input.ext_force.force_x_n, 0.0);
  EXPECT_DOUBLE_EQ(input.ext_force.moment_z_nm, 0.0);
}
