/**
 * @file SarGeometry.h
 * @brief SAR 内部本地几何与 L1-L3 轨迹工具。
 */

#ifndef ONEQ_SRC_SAR_GEOMETRY_SAR_GEOMETRY_H_
#define ONEQ_SRC_SAR_GEOMETRY_SAR_GEOMETRY_H_

#include <cstdint>
#include <random>
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

struct Waypoint {
  double time_s{0.0};
  LocalPoint position_m{};
};

struct WaypointTrackConfig {
  std::vector<Waypoint> waypoints{};
  std::vector<double> pulse_times_s{};
  std::uint64_t first_pulse_id{0U};
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

bool GenerateWaypointTrack(const WaypointTrackConfig& config,
                           std::vector<PlatformPulseState>* pulses);

bool AdvanceFractionalPrf(double dt_s, double prf_hz, FractionalPrfState* state,
                          std::uint32_t* emitted_pulses);

double Distance(const LocalPoint& a, const LocalPoint& b);

// ────────────────────────────────────────────────────────────
// 数学工具
// ────────────────────────────────────────────────────────────

/**
 * @brief 归一化 sinc 函数 sin(πx)/(πx), x→0 时返回 1。
 */
double Sinc(double x);

/**
 * @brief 确定性 Box-Muller 复高斯采样器, 内建 std::mt19937。
 *        保持 seed→序列映射不变(可直接用 seed 2026 复现)。
 */
class DeterministicGaussianSampler {
 public:
  explicit DeterministicGaussianSampler(std::uint32_t seed);
  double Sample();

 private:
  std::mt19937 generator_;
  bool has_spare_{false};
  double spare_{0.0};
};

// ────────────────────────────────────────────────────────────
// 斜距模型
// ────────────────────────────────────────────────────────────

/**
 * @brief 单脉冲平台到目标的瞬时 3D 斜距。
 */
double ExactSlantRange(const PlatformPulseState& platform, const LocalPoint& target);

/**
 * @brief 轨道上到目标的最接近斜距(遍历取 min Distance)。
 */
double ClosestSlantRange(const std::vector<PlatformPulseState>& track,
                         const LocalPoint& target);

/**
 * @brief 二次近似斜距参数(用于轨迹中心/零多普勒附近)。
 */
struct QuadraticRangeApprox {
  double reference_range_m{0.0};
  double broadside_time_s{0.0};
  double platform_velocity_mps{0.0};
};

/**
 * @brief 二次近似斜距: R(t) ≈ R0 + 0.5·(v²/R0)·(t - t0)²。
 */
double QuadraticApproxRange(const QuadraticRangeApprox& approx, double time_s);

/**
 * @brief 瞬时视线速度(距离变化率)。
 */
double RangeRate(const PlatformPulseState& platform, const LocalPoint& target);

// ────────────────────────────────────────────────────────────
// 多普勒模型
// ────────────────────────────────────────────────────────────

/**
 * @brief 多普勒参数(中心频率、调频率、合成孔径时间、带宽)。
 */
struct DopplerParams {
  double fd_central_hz{0.0};
  double fd_rate_hz_per_s{0.0};
  double synthetic_aperture_time_s{0.0};
  double doppler_bandwidth_hz{0.0};
};

/**
 * @brief 多普勒参数计算输入。
 */
struct DopplerComputationInput {
  double wavelength_m{0.0};
  double platform_velocity_mps{0.0};
  double reference_slant_range_m{0.0};
  double squint_angle_rad{0.0};
  double real_aperture_length_m{0.0};
};

/**
 * @brief 由配置计算多普勒参数。
 *        fd_rate = 2·v²·cos³(θ_sq)/(λ·R0)。
 *        fd_central = 2·v·sin(θ_sq)/λ。
 */
bool ComputeDopplerParams(const DopplerComputationInput& input, DopplerParams* params);

/**
 * @brief 慢时间 t 处的瞬时多普勒频率。
 *        fd(t) = fd_central + fd_rate · (t - t0), 其中 t0 为合成孔径中心。
 */
double DopplerFrequencyAt(const DopplerParams& params, double slow_time_s);

/**
 * @brief 方位分辨率: ρ_az = v / B_doppler。
 */
double AzimuthResolution(const DopplerParams& params, double platform_velocity_mps);

/**
 * @brief FFT 频点索引→多普勒频率映射(双边界)。
 *        index≤N/2 取正频率, 否则取 f - PRF。
 */
double DopplerBinFrequency(std::size_t index, std::size_t count, double prf_hz);

}  // namespace geometry
}  // namespace sar

#endif  // ONEQ_SRC_SAR_GEOMETRY_SAR_GEOMETRY_H_
