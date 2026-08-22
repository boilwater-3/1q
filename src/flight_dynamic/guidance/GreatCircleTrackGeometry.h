/**
 * @file GreatCircleTrackGeometry.h
 * @brief 提供全球航路点通过判定所需的球面大圆航迹几何。
 */

#ifndef FLIGHT_DYNAMIC_GUIDANCE_GREAT_CIRCLE_TRACK_GEOMETRY_H_
#define FLIGHT_DYNAMIC_GUIDANCE_GREAT_CIRCLE_TRACK_GEOMETRY_H_

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace oneq {
namespace flight_dynamic {
namespace guidance {
namespace great_circle_track {

constexpr double kEarthRadiusM = 6378137.0;
using oneq::common::numerics::kPi;

/**
 * @brief 大圆航迹上的沿航向与横航向距离。
 */
struct TrackMetricsM {
  bool valid = false;          /**< 是否成功解析非退化大圆航段。 */
  double leg_length_m = 0.0;   /**< 航段起点到目标点的大圆距离（单位：m）。 */
  double along_track_m = 0.0;  /**< 点沿起点到目标初始航向的有符号距离（单位：m）。 */
  double cross_track_m = 0.0;  /**< 点到大圆航迹的有符号最短距离（单位：m）。 */
};

/**
 * @brief 将经度差归一化到 [-π, π]，保证跨日期变更线时选择短弧。
 * @param[in] angle_rad 待归一化角度（单位：rad）。
 * @return 归一化角度（单位：rad）。
 */
inline double NormalizeLongitudeDeltaRad(double angle_rad) {
  return std::remainder(angle_rad, 2.0 * kPi);
}

/**
 * @brief 计算两点球面角距离。
 * @param[in] from_lat_rad 起点纬度（单位：rad）。
 * @param[in] from_lon_rad 起点经度（单位：rad）。
 * @param[in] to_lat_rad 终点纬度（单位：rad）。
 * @param[in] to_lon_rad 终点经度（单位：rad）。
 * @return 两点间的球面角距离（单位：rad）。
 */
inline double AngularDistanceRad(double from_lat_rad, double from_lon_rad,
                                 double to_lat_rad, double to_lon_rad) {
  const double delta_lat = to_lat_rad - from_lat_rad;
  const double delta_lon = NormalizeLongitudeDeltaRad(to_lon_rad - from_lon_rad);
  const double sin_half_lat = std::sin(0.5 * delta_lat);
  const double sin_half_lon = std::sin(0.5 * delta_lon);
  const double a = sin_half_lat * sin_half_lat +
                   std::cos(from_lat_rad) * std::cos(to_lat_rad) *
                       sin_half_lon * sin_half_lon;
  const double clamped_a = std::max(0.0, std::min(a, 1.0));
  return 2.0 * std::atan2(std::sqrt(clamped_a), std::sqrt(1.0 - clamped_a));
}

/**
 * @brief 计算起点指向终点的大圆初始方位角。
 * @param[in] from_lat_rad 起点纬度（单位：rad）。
 * @param[in] from_lon_rad 起点经度（单位：rad）。
 * @param[in] to_lat_rad 终点纬度（单位：rad）。
 * @param[in] to_lon_rad 终点经度（单位：rad）。
 * @return 初始方位角（单位：rad）。
 */
inline double InitialBearingRad(double from_lat_rad, double from_lon_rad,
                                double to_lat_rad, double to_lon_rad) {
  const double delta_lon = NormalizeLongitudeDeltaRad(to_lon_rad - from_lon_rad);
  const double y = std::sin(delta_lon) * std::cos(to_lat_rad);
  const double x = std::cos(from_lat_rad) * std::sin(to_lat_rad) -
                   std::sin(from_lat_rad) * std::cos(to_lat_rad) *
                       std::cos(delta_lon);
  return std::atan2(y, x);
}

/**
 * @brief 解析点相对大圆航段的沿航向与横航向距离。
 *
 * @param[in] start_lat_rad 航段起点纬度（单位：rad）。
 * @param[in] start_lon_rad 航段起点经度（单位：rad）。
 * @param[in] target_lat_rad 航段目标纬度（单位：rad）。
 * @param[in] target_lon_rad 航段目标经度（单位：rad）。
 * @param[in] point_lat_rad 待判定点纬度（单位：rad）。
 * @param[in] point_lon_rad 待判定点经度（单位：rad）。
 * @return 航迹距离；近零航段、近对跖航段或非有限输入返回 `valid == false`。
 */
inline TrackMetricsM ResolveTrackMetricsM(double start_lat_rad, double start_lon_rad,
                                          double target_lat_rad, double target_lon_rad,
                                          double point_lat_rad, double point_lon_rad) {
  TrackMetricsM result;
  const bool inputs_finite =
      std::isfinite(start_lat_rad) && std::isfinite(start_lon_rad) &&
      std::isfinite(target_lat_rad) && std::isfinite(target_lon_rad) &&
      std::isfinite(point_lat_rad) && std::isfinite(point_lon_rad);
  if (!inputs_finite || std::fabs(start_lat_rad) > 0.5 * kPi ||
      std::fabs(target_lat_rad) > 0.5 * kPi ||
      std::fabs(point_lat_rad) > 0.5 * kPi) {
    return result;
  }

  const double leg_angular_distance =
      AngularDistanceRad(start_lat_rad, start_lon_rad, target_lat_rad, target_lon_rad);
  constexpr double kDegenerateAngularDistanceRad = 1.0e-10;
  if (!std::isfinite(leg_angular_distance) ||
      leg_angular_distance <= kDegenerateAngularDistanceRad ||
      kPi - leg_angular_distance <= kDegenerateAngularDistanceRad) {
    return result;
  }

  const double point_angular_distance =
      AngularDistanceRad(start_lat_rad, start_lon_rad, point_lat_rad, point_lon_rad);
  const double leg_bearing =
      InitialBearingRad(start_lat_rad, start_lon_rad, target_lat_rad, target_lon_rad);
  const double point_bearing =
      InitialBearingRad(start_lat_rad, start_lon_rad, point_lat_rad, point_lon_rad);
  const double bearing_delta = point_bearing - leg_bearing;
  const double cross_argument = std::max(
      -1.0, std::min(std::sin(point_angular_distance) * std::sin(bearing_delta), 1.0));
  const double cross_angular_distance = std::asin(cross_argument);
  const double along_angular_distance =
      std::atan2(std::sin(point_angular_distance) * std::cos(bearing_delta),
                 std::cos(point_angular_distance));

  result.valid = std::isfinite(cross_angular_distance) &&
                 std::isfinite(along_angular_distance);
  if (!result.valid) return TrackMetricsM{};
  result.leg_length_m = leg_angular_distance * kEarthRadiusM;
  result.along_track_m = along_angular_distance * kEarthRadiusM;
  result.cross_track_m = cross_angular_distance * kEarthRadiusM;
  return result;
}

}  // namespace great_circle_track
}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // FLIGHT_DYNAMIC_GUIDANCE_GREAT_CIRCLE_TRACK_GEOMETRY_H_
