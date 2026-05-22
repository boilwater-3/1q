/**
 * @file ManeuverController.cpp
 * @brief 机动制导/控制律实现。
 */

#include "1q/flight_dynamic/maneuver/ManeuverController.h"

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
    input.control.altitude_hold = true;
  }

  // 速度控制：由 JSBSim Auto-Throttle 接管
  input.control.airspeed_setpoint_mps = params.cruise_speed_mps;
  input.control.airspeed_hold = true;

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
  input.control.altitude_setpoint_m = 1000.0;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = 50.0;
  input.control.airspeed_hold = true;

  return input;
}

model::FlightDynamicInput ManeuverController::ComputeWaypoint(
    const model::FlightDynamicOutput& current,
    const WaypointList& waypoints,
    const WaypointParams& params,
    std::size_t* wp_index,
    bool* all_reached) const {
  if (waypoints.empty() || !wp_index) {
    if (all_reached) *all_reached = true;
    return model::FlightDynamicInput{};
  }

  const std::size_t idx = *wp_index;
  if (idx >= waypoints.size()) {
    if (all_reached) *all_reached = true;
    return model::FlightDynamicInput{};
  }
  if (all_reached) *all_reached = false;

  const auto& target = waypoints[idx];

  // 计算到当前航路点的距离
  oneq::coordinate::LlaPositionDegM current_lla{};
  if (current.kinematics.position_frame == oneq::coordinate::PositionFrame::kLla) {
    current_lla = current.kinematics.position_lla_deg_m;
  } else {
    if (!oneq::coordinate::TryEcefToLla(current.kinematics.position_ecef_m,
                                         &current_lla)) {
      model::FlightDynamicInput input{};
      input.dt_sec = 0.05f;
      input.control.heading_hold = true;
      input.control.throttle = params.segment_params.base_throttle;
      return input;
    }
  }

  const double dist_m = ComputeGreatCircleDistanceM(current_lla, target);

  // 转弯提前量：若接近当前航路点且有下一航路点，提前转向
  double bearing_deg = ComputeForwardAzimuthDeg(current_lla, target);
  if (dist_m < params.turn_anticipation_m && idx + 1U < waypoints.size()) {
    const double next_bearing =
        ComputeForwardAzimuthDeg(current_lla, waypoints[idx + 1U]);
    const double t = 1.0 - dist_m / params.turn_anticipation_m;
    bearing_deg = bearing_deg * (1.0 - t) + next_bearing * t;
  }

  // 到达判定
  const bool reached = dist_m < params.segment_params.arrival_distance_m;
  if (reached) {
    *wp_index = idx + 1U;
    if (*wp_index >= waypoints.size()) {
      if (all_reached) *all_reached = true;
      model::FlightDynamicInput input{};
      input.dt_sec = 0.05f;
      input.control.throttle = params.segment_params.base_throttle;
      return input;
    }
  }

  model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;
  input.control.heading_setpoint_deg = bearing_deg;
  input.control.heading_hold = true;

  input.control.altitude_setpoint_m = target.altitude_m;
  input.control.altitude_hold = true;

  input.control.airspeed_setpoint_mps = params.segment_params.cruise_speed_mps;
  input.control.airspeed_hold = true;

  return input;
}

model::FlightDynamicInput ManeuverController::ComputeBarrelRoll(
    const model::FlightDynamicOutput& current,
    const BarrelRollParams& params,
    double sim_time_s,
    BarrelRollState* state) const {
  model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;

  if (!state) return input;

  constexpr double kDt = 0.05;

  // 首次调用初始化
  if (!state->initialized) {
    state->phase = BarrelRollPhase::kRolling;
    state->roll_start_time_s = sim_time_s;
    state->initial_altitude_m = current.state.altitude_msl_m;
    state->cumulative_roll_deg = 0.0;
    state->roll_integral = 0.0;
    state->alt_integral = 0.0;
    state->initialized = true;
  }

  // 安全检查：高度损失超限 → 中止
  const double alt_loss =
      state->initial_altitude_m - current.state.altitude_msl_m;
  if (alt_loss > params.max_altitude_loss_m) {
    state->phase = BarrelRollPhase::kAborted;
  }

  // 安全检查：海拔过低 → 中止
  if (current.state.altitude_msl_m < 200.0) {
    state->phase = BarrelRollPhase::kAborted;
  }

  switch (state->phase) {
    case BarrelRollPhase::kRolling: {
      // 累积滚转角（使用机体滚转角速率积分，避免 Euler 角跳变）
      state->cumulative_roll_deg +=
          current.state.roll_rate_radps * kRadToDeg * kDt;

      // 目标滚转角：线性爬坡到 target_roll_deg
      const double elapsed = sim_time_s - state->roll_start_time_s;
      const double target_roll =
          std::min(elapsed * params.roll_rate_degps, params.target_roll_deg);

      // 滚转 PID → 副翼
      const double roll_error = target_roll - state->cumulative_roll_deg;
      state->roll_integral += roll_error * kDt;
      state->roll_integral =
          std::max(-50.0, std::min(50.0, state->roll_integral));
      double aileron =
          params.roll_kp * roll_error + params.roll_ki * state->roll_integral;
      aileron = std::max(-1.0, std::min(1.0, aileron));

      // 高度 PID → 升降舵（cos 修正：倒飞时升降舵反向）
      const double alt_error =
          params.base_altitude_m - current.state.altitude_msl_m;
      state->alt_integral += alt_error * kDt;
      state->alt_integral =
          std::max(-200.0, std::min(200.0, state->alt_integral));
      double elevator =
          params.alt_kp * alt_error + params.alt_ki * state->alt_integral;
      elevator *= std::cos(state->cumulative_roll_deg * kDegToRad);
      elevator = std::max(-1.0, std::min(1.0, elevator));

      input.control.aileron = aileron;
      input.control.elevator = elevator;
      input.control.heading_hold = false;
      input.control.altitude_hold = false;
      input.control.airspeed_setpoint_mps = params.cruise_speed_mps;
      input.control.airspeed_hold = true;
      input.control.throttle = 0.9;

      // 滚转完成判定
      if (target_roll >= params.target_roll_deg &&
          state->cumulative_roll_deg >= params.target_roll_deg * 0.9) {
        state->phase = BarrelRollPhase::kRecovery;
      }
      break;
    }

    case BarrelRollPhase::kRecovery: {
      // 恢复 AP 控制
      input.control.heading_setpoint_deg = current.state.yaw_deg;
      input.control.heading_hold = true;
      input.control.altitude_setpoint_m = params.base_altitude_m;
      input.control.altitude_hold = true;
      input.control.airspeed_setpoint_mps = params.cruise_speed_mps;
      input.control.airspeed_hold = true;

      if (std::abs(current.state.roll_deg) < 10.0) {
        state->phase = BarrelRollPhase::kCompleted;
      }
      break;
    }

    case BarrelRollPhase::kAborted: {
      // 紧急恢复：副翼平飞
      input.control.aileron =
          std::max(-1.0, std::min(1.0, -current.state.roll_deg * 0.05));
      input.control.altitude_setpoint_m = params.base_altitude_m;
      input.control.altitude_hold = true;
      input.control.airspeed_setpoint_mps = params.cruise_speed_mps;
      input.control.airspeed_hold = true;
      input.control.throttle = 0.8;

      if (std::abs(current.state.roll_deg) < 10.0) {
        state->phase = BarrelRollPhase::kCompleted;
      }
      break;
    }

    case BarrelRollPhase::kCompleted: {
      input.control.heading_hold = true;
      input.control.heading_setpoint_deg = current.state.yaw_deg;
      input.control.altitude_setpoint_m = params.base_altitude_m;
      input.control.altitude_hold = true;
      input.control.throttle = 0.75;
      break;
    }
  }

  return input;
}

model::FlightDynamicInput ManeuverController::ComputeOrbit(
    const model::FlightDynamicOutput& current,
    const OrbitParams& params,
    double /*sim_time_s*/,
    OrbitState* state) const {
  model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;

  if (!state) return input;

  // 从 ECEF 位置提取当前 LLA
  oneq::coordinate::LlaPositionDegM current_lla{};
  if (current.kinematics.position_frame == oneq::coordinate::PositionFrame::kLla) {
    current_lla = current.kinematics.position_lla_deg_m;
  } else {
    if (!oneq::coordinate::TryEcefToLla(current.kinematics.position_ecef_m,
                                         &current_lla)) {
      return input;
    }
  }

  // 计算到盘旋中心的方位角和距离
  const double dist_m =
      ComputeGreatCircleDistanceM(current_lla, params.center_lla);
  const double bearing_to_center =
      ComputeForwardAzimuthDeg(current_lla, params.center_lla);

  // 切线航向：垂直于径向方向
  const double dir_sign = params.clockwise ? 1.0 : -1.0;
  double tangent_deg = bearing_to_center + dir_sign * 90.0;

  // 径向修正：比例控制保持飞机在轨道半径上
  // 归一化径向误差到 [-1, 1]，限制最大修正角 ±45°
  const double radial_error = (dist_m - params.radius_m) /
                              std::max(1.0, params.radius_m);
  const double max_correction_deg = 45.0;
  const double correction_deg = std::max(-max_correction_deg,
      std::min(max_correction_deg, -dir_sign * radial_error * max_correction_deg));

  double target_heading = tangent_deg + correction_deg;

  // 规范化到 [0, 360)
  while (target_heading < 0.0) target_heading += 360.0;
  while (target_heading >= 360.0) target_heading -= 360.0;

  if (!state->initialized) {
    state->initialized = true;
  }

  input.control.heading_setpoint_deg = target_heading;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = params.altitude_m;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = params.cruise_speed_mps;
  input.control.airspeed_hold = true;

  return input;
}

model::FlightDynamicInput ManeuverController::ComputeEvasion(
    const model::FlightDynamicOutput& current,
    const EvasionParams& params,
    double sim_time_s,
    EvasionState* state) const {
  model::FlightDynamicInput input{};
  input.dt_sec = 0.05f;

  if (!state) return input;

  // 首次调用初始化
  if (!state->initialized) {
    state->phase = EvasionPhase::kBreaking;
    state->start_time_s = sim_time_s;
    state->initialized = true;
  }

  const double elapsed = sim_time_s - state->start_time_s;

  // 规避机动：急转弯到规避航向 + 下降 + 加速
  input.control.heading_setpoint_deg = params.evasion_heading_deg;
  input.control.heading_hold = true;
  input.control.altitude_setpoint_m = params.target_altitude_m;
  input.control.altitude_hold = true;
  input.control.airspeed_setpoint_mps = params.cruise_speed_mps;
  input.control.airspeed_hold = true;

  // 阶段转换
  if (state->phase == EvasionPhase::kBreaking) {
    // 破转阶段：航向收敛到规避航向后进入下降阶段
    double heading_err = std::abs(current.state.yaw_deg - params.evasion_heading_deg);
    if (heading_err > 180.0) heading_err = 360.0 - heading_err;
    if (heading_err < 10.0) {
      state->phase = EvasionPhase::kDescending;
    }
  }

  // 持续时间到期 → 完成
  if (elapsed >= params.duration_s) {
    state->phase = EvasionPhase::kCompleted;
  }

  return input;
}

}  // namespace maneuver
}  // namespace flight_dynamic
