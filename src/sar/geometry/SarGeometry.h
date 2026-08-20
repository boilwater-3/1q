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
 * @brief SAR 本地 Cartesian 坐标，x=East, y=North, z=Up（scene-center-relative ENU）。
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
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
};

/**
 * @brief L1 匀速直线条带轨迹配置。
 */
struct StraightStripmapTrackConfig {
  LocalPoint start_position_m{};
  double start_time_s{0.0};
  double velocity_x_mps{0.0};
  double velocity_y_mps{0.0};
  double velocity_z_mps{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
  double prf_hz{0.0};
  std::uint64_t first_pulse_id{0U};
  std::uint32_t pulse_count{0U};
};

/**
 * @brief L1 匀速直线条带轨迹叠加确定性扰动配置（运动补偿测试用）。
 */
struct PerturbedStripmapTrackConfig {
  StraightStripmapTrackConfig ideal{};   /**< 理想匀速直线轨迹 */
  LocalPoint initial_position_error_m{}; /**< 初始位置误差（m） */
  double velocity_error_stddev_x_mps{0.0}; /**< x 向速度误差标准差（m/s） */
  double velocity_error_stddev_y_mps{0.0}; /**< y 向速度误差标准差（m/s） */
  double velocity_error_stddev_z_mps{0.0}; /**< z 向速度误差标准差（m/s） */
  std::uint32_t random_seed{0U};          /**< 确定性扰动随机种子 */
};

/**
 * @brief 任意时刻的航点（waypoint）位置（L3 多航点轨迹用）。
 */
struct Waypoint {
  double time_s{0.0};        /**< 相对会话起始的时刻（s） */
  LocalPoint position_m{};   /**< 航点位置（m） */
};

/**
 * @brief L3 多航点轨迹配置。
 */
struct WaypointTrackConfig {
  std::vector<Waypoint> waypoints{};   /**< 航点序列（时间单调递增） */
  std::vector<double> pulse_times_s{}; /**< 各脉冲对应的慢时间（s） */
  std::uint64_t first_pulse_id{0U};    /**< 首个脉冲 ID */
};

/**
 * @brief 扰动轨迹相对理想轨迹的误差诊断。
 */
struct TrajectoryErrorDiagnostics {
  double max_position_error_m{0.0};      /**< 最大位置误差（m） */
  double rms_position_error_m{0.0};      /**< RMS 位置误差（m） */
  double max_velocity_error_mps{0.0};    /**< 最大速度误差（m/s） */
  double rms_velocity_error_mps{0.0};    /**< RMS 速度误差（m/s） */
};

/**
 * @brief fractional PRF 脉冲计数状态。
 */
struct FractionalPrfState {
  double carry_pulses{0.0};
};

/**
 * @brief 生成 L1 匀速直线条带轨迹脉冲序列。
 * @param[in] config 轨迹配置。
 * @param[out] pulses 输出脉冲序列。
 * @return 成功返回 true，失败返回 false。
 */
bool GenerateStraightStripmapTrack(const StraightStripmapTrackConfig& config,
                                   std::vector<PlatformPulseState>* pulses);

/**
 * @brief 生成叠加确定性扰动的条带轨迹脉冲序列，并回填误差诊断。
 * @param[in] config 含理想轨迹与扰动参数的配置。
 * @param[out] pulses 输出含扰动的脉冲序列。
 * @param[out] diagnostics 扰动相对理想轨迹的误差诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool GeneratePerturbedStripmapTrack(const PerturbedStripmapTrackConfig& config,
                                    std::vector<PlatformPulseState>* pulses,
                                    TrajectoryErrorDiagnostics* diagnostics);

/**
 * @brief 由航点序列插值生成 L3 多航点轨迹脉冲序列。
 * @param[in] config 航点轨迹配置。
 * @param[out] pulses 输出脉冲序列。
 * @return 成功返回 true，失败返回 false。
 */
bool GenerateWaypointTrack(const WaypointTrackConfig& config,
                           std::vector<PlatformPulseState>* pulses);

/**
 * @brief 按周期步长与 PRF 推进 fractional PRF 状态并返回本周期发射脉冲数。
 * @param[in] dt_s 周期步长（s）。
 * @param[in] prf_hz 脉冲重复频率（Hz）。
 * @param[in,out] state fractional PRF 累积状态，调用后更新。
 * @param[out] emitted_pulses 本周期应发射的脉冲数。
 * @return 成功返回 true，失败返回 false。
 */
bool AdvanceFractionalPrf(double dt_s, double prf_hz, FractionalPrfState* state,
                          std::uint32_t* emitted_pulses);

/**
 * @brief 计算两点间的欧氏距离。
 * @param[in] a 点 A。
 * @param[in] b 点 B。
 * @return 距离（m）。
 */
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
  /**
   * @brief 用给定随机种子构造采样器。
   * @param[in] seed 随机种子（相同 seed 产生相同序列）。
   */
  explicit DeterministicGaussianSampler(std::uint32_t seed);
  /**
   * @brief 采样一个标准正态分布随机数。
   * @return 采样值。
   */
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
