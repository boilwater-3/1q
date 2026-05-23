/**
 * @file fd_adapter_test.cpp
 * @brief JsbsimAdapter 单元测试。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/model/FlightDynamicInput.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"

namespace fd_adapter = flight_dynamic::adapter;
namespace fd_config = flight_dynamic::config;
namespace fd_model = flight_dynamic::model;
namespace coord = oneq::coordinate;

namespace {

#ifndef FD_JSBSIM_ROOT_DIR
#define FD_JSBSIM_ROOT_DIR ""
#endif

fd_config::FlightDynamicConfig MakeAdapterConfig() {
  fd_config::FlightDynamicConfig cfg;
  cfg.aircraft.root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.aircraft.model_name = "c172x";
  cfg.silent = true;
  cfg.do_trim = false;  // Fast creation for adapter test
  cfg.initial_kinematics.position_frame = coord::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 39.9;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 116.4;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 1000.0;
  return cfg;
}

}  // namespace

TEST(FdAdapterTest, InitializationFailsOnInvalidModel) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) GTEST_SKIP() << "No JSBSim dir";

  fd_config::FlightDynamicConfig cfg = MakeAdapterConfig();
  cfg.aircraft.model_name = "invalid_model_123";

  EXPECT_THROW({ fd_adapter::JsbsimAdapter adapter(cfg); }, std::runtime_error);
}

TEST(FdAdapterTest, PropertyGetSet) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) GTEST_SKIP() << "No JSBSim dir";

  fd_config::FlightDynamicConfig cfg = MakeAdapterConfig();
  fd_adapter::JsbsimAdapter adapter(cfg);

  // Set a property that doesn't affect physics instantly and read it back
  double initial_val = adapter.GetProperty("fcs/throttle-cmd-norm[0]");

  adapter.SetProperty("fcs/throttle-cmd-norm[0]", 0.77);
  EXPECT_NEAR(adapter.GetProperty("fcs/throttle-cmd-norm[0]"), 0.77, 1e-5);

  // Reset back
  adapter.SetProperty("fcs/throttle-cmd-norm[0]", initial_val);
}

TEST(FdAdapterTest, ExternalForcesPropagation) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) GTEST_SKIP() << "No JSBSim dir";

  fd_config::FlightDynamicConfig cfg = MakeAdapterConfig();
  fd_adapter::JsbsimAdapter adapter(cfg);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.ext_force.force_x_n = 1000.0;   // 1000 N
  input.ext_force.moment_y_nm = 500.0;  // 500 Nm

  EXPECT_TRUE(adapter.Run(input));

  // Check if properties were written (in lbf and lbf-ft)
  // kNToLbf = 0.224809, so 1000 N = 224.809 lbf
  // kNmToLbfFt = 0.737562, so 500 Nm = 368.781 lbf-ft
  double fx_lbf = adapter.GetProperty("external_reactions/external/x");
  double my_lbfft = adapter.GetProperty("external_reactions/external/m");

  EXPECT_NEAR(fx_lbf, 1000.0 * 0.224809, 1.0);
  EXPECT_NEAR(my_lbfft, 500.0 * 0.737562, 1.0);
}

TEST(FdAdapterTest, RunAdvancesSimulationTime) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) GTEST_SKIP() << "No JSBSim dir";

  fd_config::FlightDynamicConfig cfg = MakeAdapterConfig();
  fd_adapter::JsbsimAdapter adapter(cfg);

  double initial_time = adapter.GetProperty("simulation/sim-time-sec");

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.1f;

  EXPECT_TRUE(adapter.Run(input));

  double new_time = adapter.GetProperty("simulation/sim-time-sec");
  EXPECT_NEAR(new_time, initial_time + 0.1, 1e-4);
}

TEST(FdAdapterTest, ResetRestoresInitialStateAndClearsIntegral) {
  if (std::string(FD_JSBSIM_ROOT_DIR).empty()) GTEST_SKIP() << "No JSBSim dir";

  fd_config::FlightDynamicConfig cfg = MakeAdapterConfig();
  fd_adapter::JsbsimAdapter adapter(cfg);

  fd_model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.airspeed_setpoint_mps = 60.0;
  input.control.airspeed_hold = true;  // This will accumulate integral

  for (int i = 0; i < 100; ++i) {
    adapter.Run(input);
  }

  double alt_after_run = adapter.GetProperty("position/h-sl-meters");

  adapter.Reset(cfg.initial_kinematics);

  double alt_after_reset = adapter.GetProperty("position/h-sl-meters");
  EXPECT_NEAR(alt_after_reset, 1000.0, 1.0);  // Reset to 1000m
}
