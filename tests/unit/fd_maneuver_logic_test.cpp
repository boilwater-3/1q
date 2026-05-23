/**
 * @file fd_maneuver_logic_test.cpp
 * @brief ManeuverController 无状态逻辑单元测试
 */

#include <gtest/gtest.h>

#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/maneuver/ManeuverController.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"

namespace fd_maneuver = flight_dynamic::maneuver;
namespace fd_model = flight_dynamic::model;
namespace coord = oneq::coordinate;

namespace {

fd_model::FlightDynamicOutput CreateMockState(double lat, double lon, double alt_m,
                                              double heading_deg) {
  fd_model::FlightDynamicOutput out{};
  out.ok = true;
  out.kinematics.position_frame = coord::PositionFrame::kLla;
  out.kinematics.position_lla_deg_m.latitude_deg = lat;
  out.kinematics.position_lla_deg_m.longitude_deg = lon;
  out.kinematics.position_lla_deg_m.altitude_m = alt_m;
  out.state.altitude_msl_m = alt_m;
  out.state.yaw_deg = heading_deg;
  return out;
}

}  // namespace

TEST(FdManeuverLogicTest, PointToPointLogic) {
  fd_maneuver::ManeuverController ctrl;
  auto current = CreateMockState(39.9, 116.4, 1000.0, 0.0);

  fd_maneuver::PointToPointParams params{};
  params.target_lla.latitude_deg = 39.9;
  params.target_lla.longitude_deg = 116.4 + 0.1;
  params.target_lla.altitude_m = 1500.0;
  params.arrival_distance_m = 100.0;
  params.cruise_speed_mps = 60.0;

  bool reached = false;
  auto input = ctrl.ComputePointToPoint(current, params, &reached);

  EXPECT_FALSE(reached);
  EXPECT_NEAR(input.control.heading_setpoint_deg, 90.0, 1.0);  // Target is East
  EXPECT_TRUE(input.control.heading_hold);
  EXPECT_NEAR(input.control.altitude_setpoint_m, 1500.0, 1e-3);
  EXPECT_TRUE(input.control.altitude_hold);
  EXPECT_NEAR(input.control.airspeed_setpoint_mps, 60.0, 1e-3);
}

TEST(FdManeuverLogicTest, PointToPointReached) {
  fd_maneuver::ManeuverController ctrl;
  auto current = CreateMockState(39.9, 116.4, 1000.0, 0.0);

  fd_maneuver::PointToPointParams params{};
  params.target_lla.latitude_deg = 39.9;
  params.target_lla.longitude_deg = 116.4;
  params.target_lla.altitude_m = 1000.0;
  params.arrival_distance_m = 100.0;

  bool reached = false;
  ctrl.ComputePointToPoint(current, params, &reached);
  EXPECT_TRUE(reached);
}

TEST(FdManeuverLogicTest, WaypointTransitionLogic) {
  fd_maneuver::ManeuverController ctrl;
  auto current = CreateMockState(39.9, 116.4, 1000.0, 90.0);

  fd_maneuver::WaypointList wps;
  {
    coord::LlaPositionDegM wp1{};
    wp1.latitude_deg = 39.9;
    wp1.longitude_deg = 116.4 + 0.0001;
    wp1.altitude_m = 1000.0;
    wps.push_back(wp1);

    coord::LlaPositionDegM wp2{};
    wp2.latitude_deg = 39.9;
    wp2.longitude_deg = 116.4 + 0.1;
    wp2.altitude_m = 1000.0;
    wps.push_back(wp2);
  }

  fd_maneuver::WaypointParams params{};
  params.turn_anticipation_m = 200.0;  // Huge turn anticipation will trigger immediately for wp0

  std::size_t wp_index = 0;
  bool all_reached = false;

  // Call once, since it's very close (dist < 200), it should transition index to 1
  ctrl.ComputeWaypoint(current, wps, params, &wp_index, &all_reached);

  EXPECT_FALSE(all_reached);
  EXPECT_EQ(wp_index, 1U);
}

TEST(FdManeuverLogicTest, WeaveLogic) {
  fd_maneuver::ManeuverController ctrl;
  auto current = CreateMockState(39.9, 116.4, 1000.0, 0.0);

  fd_maneuver::WeaveParams params{};
  params.base_heading_deg = 0.0;
  params.amplitude_deg = 30.0;
  params.period_s = 20.0;

  // At t=0, sine(0) = 0, so heading is base_heading = 0.0
  auto in_0 = ctrl.ComputeWeave(current, params, 0.0);
  EXPECT_NEAR(in_0.control.heading_setpoint_deg, 0.0, 1e-3);

  // At t=5, sine(pi/2) = 1, so heading = 30.0
  auto in_5 = ctrl.ComputeWeave(current, params, 5.0);
  EXPECT_NEAR(in_5.control.heading_setpoint_deg, 30.0, 1e-3);

  // At t=15, sine(3pi/2) = -1, so heading = -30.0 -> normalized to 330.0
  auto in_15 = ctrl.ComputeWeave(current, params, 15.0);
  EXPECT_NEAR(in_15.control.heading_setpoint_deg, 330.0, 1e-3);
}

TEST(FdManeuverLogicTest, BarrelRollStateSafeguard) {
  fd_maneuver::ManeuverController ctrl;
  // Initialize slightly rolled, but dropped in altitude
  auto current = CreateMockState(39.9, 116.4, 750.0, 0.0);  // Now at 750m
  current.state.roll_deg = 180.0;                           // Inverted

  fd_maneuver::BarrelRollParams params{};
  params.base_altitude_m = 1000.0;
  params.max_altitude_loss_m = 200.0;  // Threshold is 800m

  fd_maneuver::BarrelRollState state{};
  state.phase = fd_maneuver::BarrelRollPhase::kRolling;
  state.initial_altitude_m = 1000.0;
  state.initialized = true;

  auto input = ctrl.ComputeBarrelRoll(current, params, 1.0, &state);

  // Should abort because altitude loss (250m) > max (200m)
  EXPECT_EQ(state.phase, fd_maneuver::BarrelRollPhase::kAborted);

  // When aborted, aileron should aim to recover roll to 0, so heading_hold is false
  EXPECT_FALSE(input.control.heading_hold);
  EXPECT_TRUE(input.control.altitude_hold);
  EXPECT_LT(input.control.aileron, 0.0);  // Recover from 180 deg roll with negative aileron
}
