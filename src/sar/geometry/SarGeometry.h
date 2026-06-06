/**
 * @file SarGeometry.h
 * @brief SAR 内部本地几何与 L1 条带轨迹工具。
 */

#ifndef ONEQ_SRC_SAR_GEOMETRY_SAR_GEOMETRY_H_
#define ONEQ_SRC_SAR_GEOMETRY_SAR_GEOMETRY_H_

#include <cstdint>
#include <vector>

namespace sar {
namespace geometry {

/**
 * @brief SAR 本地 Cartesian 坐标，x=azimuth, y=ground range, z=altitude。
 */
struct LocalPoint {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
};

/**
 * @brief 单脉冲平台状态。
 */
struct PlatformPulseState {
  std::uint64_t pulse_id{0U};
  double time_s{0.0};
  LocalPoint position_m{};
  double velocity_x_mps{0.0};
  double velocity_y_mps{0.0};
  double velocity_z_mps{0.0};
};

/**
 * @brief L1 匀速直线条带轨迹配置。
 */
struct StraightStripmapTrackConfig {
  LocalPoint start_position_m{};
  double velocity_x_mps{0.0};
  double prf_hz{0.0};
  std::uint64_t first_pulse_id{0U};
  std::uint32_t pulse_count{0U};
};

struct PerturbedStripmapTrackConfig {
  StraightStripmapTrackConfig ideal{};
  LocalPoint initial_position_error_m{};
  double velocity_error_stddev_x_mps{0.0};
  double velocity_error_stddev_y_mps{0.0};
  double velocity_error_stddev_z_mps{0.0};
  std::uint32_t random_seed{0U};
};

struct TrajectoryErrorDiagnostics {
  double max_position_error_m{0.0};
  double rms_position_error_m{0.0};
  double max_velocity_error_mps{0.0};
  double rms_velocity_error_mps{0.0};
};

/**
 * @brief fractional PRF 脉冲计数状态。
 */
struct FractionalPrfState {
  double carry_pulses{0.0};
};

bool GenerateStraightStripmapTrack(const StraightStripmapTrackConfig& config,
                                   std::vector<PlatformPulseState>* pulses);

bool GeneratePerturbedStripmapTrack(const PerturbedStripmapTrackConfig& config,
                                    std::vector<PlatformPulseState>* pulses,
                                    TrajectoryErrorDiagnostics* diagnostics);

bool AdvanceFractionalPrf(double dt_s, double prf_hz, FractionalPrfState* state,
                          std::uint32_t* emitted_pulses);

double Distance(const LocalPoint& a, const LocalPoint& b);

}  // namespace geometry
}  // namespace sar

#endif  // ONEQ_SRC_SAR_GEOMETRY_SAR_GEOMETRY_H_
