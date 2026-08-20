#ifndef ONEQ_TESTS_UNIT_FD_TEST_HELPERS_H_
#define ONEQ_TESTS_UNIT_FD_TEST_HELPERS_H_

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"

namespace oneq {
namespace flight_dynamic {
namespace {

constexpr double kDt = 0.01;
constexpr double kPi = 3.14159265358979323846;

class TrajectoryLogger {
 public:
  TrajectoryLogger(const std::string& model, const std::string& maneuver) {
    const char* dump_env = std::getenv("DUMP_MANEUVER_TRAJECTORY");
    if (dump_env && std::string(dump_env) == "1") {
      enabled_ = true;
      std::string path = "/tmp/1q_trajectories/" + maneuver + "_" + model + ".csv";
      out_.open(path);
      if (out_.is_open()) {
        out_ << "time_sec,lat_rad,lon_rad,alt_m,pitch_rad,roll_rad,heading_rad\n";
      } else {
        enabled_ = false;
      }
    }
  }

  void Log(const model::VehicleState& state) {
    if (enabled_ && out_.is_open()) {
      out_ << std::fixed << std::setprecision(6)
           << state.sim_time_sec << ","
           << state.latitude_rad << ","
           << state.longitude_rad << ","
           << state.altitude_geod_m << ","
           << state.theta_rad << ","
           << state.phi_rad << ","
           << state.psi_rad << "\n";
    }
  }

  void LogTarget(double lat_rad, double lon_rad, double alt_m = 0.0) {
    if (enabled_ && out_.is_open()) {
      out_ << "#TARGET," << lat_rad << "," << lon_rad << "," << alt_m << "\n";
    }
  }

  void LogTargetValue(double val) {
    if (enabled_ && out_.is_open()) {
      out_ << "#TARGET_VAL," << val << "\n";
    }
  }

 private:
  bool enabled_ = false;
  std::ofstream out_;
};

bool RunSteps(FlightManager& fm, int steps, TrajectoryLogger* logger = nullptr) {
  for (int i = 0; i < steps; ++i) {
    if (!fm.Step(kDt)) return false;
    const auto& s = fm.GetVehicleState();
    if (logger) logger->Log(s);
  }
  return true;
}

int RunUntilDone(FlightManager& fm, int max_steps, TrajectoryLogger* logger = nullptr) {
  for (int i = 0; i < max_steps; ++i) {
    fm.Step(kDt);
    const auto& s = fm.GetVehicleState();
    if (logger) logger->Log(s);
    auto state = fm.GetState();
    if (state == FlightManagerState::kCompleted ||
        state == FlightManagerState::kAborted) {
      return i + 1;
    }
  }
  return max_steps;
}

void ExpectNoNaN(const model::VehicleState& state) {
  EXPECT_FALSE(std::isnan(state.altitude_geod_m)) << "altitude NaN";
  EXPECT_FALSE(std::isnan(state.psi_rad)) << "heading NaN";
  EXPECT_FALSE(std::isnan(state.theta_rad)) << "pitch NaN";
  EXPECT_FALSE(std::isnan(state.phi_rad)) << "roll NaN";
  EXPECT_FALSE(std::isnan(state.u_mps)) << "u velocity NaN";
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_TESTS_UNIT_FD_TEST_HELPERS_H_
