/**
 * @file ManeuverController.cpp
 * @brief 机动制导/控制律实现。
 */

#include "flight_dynamic/maneuver/ManeuverController.h"

#include <algorithm>

#include "1q/coordinate/position_transform.h"

namespace flight_dynamic {
namespace maneuver {

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kEarthRadiusM = 6371000.0;

}  // namespace

// ---- 几何工具 ----

double ComputeGreatCircleDistanceM(const oneq::coordinate::LlaPositionDegM& from,
                                   const oneq::coordinate::LlaPositionDegM& to) {
  const double lat1 = from.latitude_deg * kDegToRad;
  const double lon1 = from.longitude_deg * kDegToRad;
  const double lat2 = to.latitude_deg * kDegToRad;
  const double lon2 = to.longitude_deg * kDegToRad;

  const double dlat = lat2 - lat1;
  const double dlon = lon2 - lon1;

  const double a = std::sin(dlat * 0.5) * std::sin(dlat * 0.5) +
                   std::cos(lat1) * std::cos(lat2) *
                       std::sin(dlon * 0.5) * std::sin(dlon * 0.5);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  return kEarthRadiusM * c;
}

double ComputeForwardAzimuthDeg(const oneq::coordinate::LlaPositionDegM& from,
                                const oneq::coordinate::LlaPositionDegM& to) {
  const double lat1 = from.latitude_deg * kDegToRad;
  const double lon1 = from.longitude_deg * kDegToRad;
  const double lat2 = to.latitude_deg * kDegToRad;
  const double lon2 = to.longitude_deg * kDegToRad;

  const double dlon = lon2 - lon1;

  const double y = std::sin(dlon) * std::cos(lat2);
  const double x = std::cos(lat1) * std::sin(lat2) -
                   std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

  double azimuth_rad = std::atan2(y, x);
  double azimuth_deg = azimuth_rad * kRadToDeg;
  if (azimuth_deg < 0.0) azimuth_deg += 360.0;
  return azimuth_deg;
}

// ---- 机动控制 ----

model::FlightDynamicInput ManeuverController::ComputePointToPoint(
    const model::FlightDynamicOutput& current,
    const PointToPointParams& params,
    bool* reached) const {
  model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;

  // 从 ECEF 位置提取当前 LLA
  oneq::coordinate::LlaPositionDegM current_lla{};
  if (current.kinematics.position_frame == oneq::coordinate::PositionFrame::kLla) {
    current_lla = current.kinematics.position_lla_deg_m;
  } else {
    if (!oneq::coordinate::TryEcefToLla(current.kinematics.position_ecef_m,
                                         &current_lla)) {
      if (reached) *reached = false;
      return input;
    }
  }

  // 计算方位角和距离
  const double dist_m =
      ComputeGreatCircleDistanceM(current_lla, params.target_lla);
  const double bearing_deg =
      ComputeForwardAzimuthDeg(current_lla, params.target_lla);

  // 到达判定：距离 + 航向收敛
  if (dist_m < params.arrival_distance_m) {
    if (reached) *reached = true;
    input.control.heading_hold = false;
    input.control.altitude_hold = false;
  } else {
    if (reached) *reached = false;
    input.control.heading_setpoint_deg = bearing_deg;
    input.control.heading_hold = true;
    input.control.altitude_setpoint_m = params.target_lla.altitude_m;
    input.control.altitude_hold = false;
  }

  // 速度控制：简单 P 控制油门
  const double speed_error = params.cruise_speed_mps - current.state.ground_speed_mps;
  double throttle = params.base_throttle + speed_error * 0.01;
  throttle = std::max(0.1, std::min(1.0, throttle));
  input.control.throttle = throttle;

  return input;
}

model::FlightDynamicInput ManeuverController::ComputeWeave(
    const model::FlightDynamicOutput& /*current*/,
    const WeaveParams& params,
    double sim_time_s) const {
  model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;

  const double omega = 2.0 * M_PI / params.period_s;
  const double heading =
      params.base_heading_deg + params.amplitude_deg * std::sin(omega * sim_time_s);

  // 规范化到 [0, 360)
  double normalized = heading;
  while (normalized < 0.0) normalized += 360.0;
  while (normalized >= 360.0) normalized -= 360.0;

  input.control.heading_setpoint_deg = normalized;
  input.control.heading_hold = true;
  input.control.throttle = 0.75;

  return input;
}

}  // namespace maneuver
}  // namespace flight_dynamic
