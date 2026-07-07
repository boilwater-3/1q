/**
 * @file SarEcho.h
 * @brief SAR 内部点目标原始回波生成工具。
 */

#ifndef ONEQ_SRC_SAR_ECHO_SAR_ECHO_H_
#define ONEQ_SRC_SAR_ECHO_SAR_ECHO_H_

#include <cstddef>
#include <vector>

#include "sar/geometry/SarAntenna.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/geometry/SarScanBurst.h"
#include "sar/geometry/SarSpotlightBeam.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace echo {

/**
 * @brief 本地场景点目标。
 */
struct PointTarget {
  geometry::LocalPoint position_m{};
  double rcs_m2{1.0};
};

/**
 * @brief 单个点目标回波诊断。
 */
struct EchoTargetDiagnostic {
  std::size_t target_index{0U};
  double slant_range_m{0.0};
  double two_way_delay_s{0.0};
  std::size_t delay_sample_index{0U};
  double fractional_delay_samples{0.0};
  bool clipped{false};
  std::size_t clipped_samples{0U};
};

/**
 * @brief 原始回波生成配置。
 */
struct RawEchoConfig {
  double sample_rate_hz{0.0};
  double carrier_frequency_hz{0.0};
  std::size_t range_sample_count{0U};
};

/**
 * @brief 单脉冲原始回波结果。
 */
struct RawEchoResult {
  signal::ComplexVector samples{};
  std::vector<EchoTargetDiagnostic> diagnostics{};
  bool has_clipping{false};
};

bool GeneratePointTargetRawEcho(const RawEchoConfig& config,
                                const geometry::PlatformPulseState& platform,
                                const std::vector<PointTarget>& targets,
                                const signal::ComplexVector& transmit_waveform,
                                RawEchoResult* result);

/**
 * @brief 天线方向图调制配置(聚束/斜视模式)。
 *
 * enabled=false 时,回波生成退化为无方向图加权(条带兼容,broadside 退化不变量)。
 * enabled=true 时,每个目标的幅度乘以方位 sinc² 方向图 AzimuthPattern(off_boresight)。
 */
struct AntennaModulationConfig {
  geometry::AntennaParams antenna{};
  geometry::SpotlightBeamState beam_state{};  /**< 该脉冲的波束指向 */
  bool enabled{false};
};

/**
 * @brief 带天线方向图调制的点目标回波生成(聚束/斜视模式)。
 *
 * 与 GeneratePointTargetRawEcho 逻辑相同,仅在幅度项增加方位方向图加权:
 *   off_boresight = target_azimuth − beam_state.boresight_azimuth
 *   amplitude *= AzimuthPattern(antenna, λ, off_boresight)
 * antenna_config.enabled=false 时,与 GeneratePointTargetRawEcho 输出完全一致。
 */
bool GeneratePointTargetRawEchoWithAntenna(const RawEchoConfig& config,
                                           const AntennaModulationConfig& antenna_config,
                                           const geometry::PlatformPulseState& platform,
                                           const std::vector<PointTarget>& targets,
                                           const signal::ComplexVector& transmit_waveform,
                                           RawEchoResult* result);

/**
 * @brief elevation 距离门控配置(扫描 ScanSAR 模式)。
 *
 * enabled=false 时,回波生成退化为无门控(条带兼容,单子带退化不变量)。
 * enabled=true 时,仅当目标斜距落在 burst_state 的 [near_range, far_range) 窗口内、
 * 且 burst_state.illuminated 为真时,该目标才贡献回波;否则跳过(幅度置 0)。
 *
 * 与聚束 AntennaModulationConfig(方位 sinc² 软加权)正交:ScanSAR 用距离硬门控
 * 表达"天线此刻只看这个子带"(契约 scansar_mode.md §3.3)。
 */
struct ElevationGateConfig {
  geometry::ScanBurstState burst_state{};  /**< 该脉冲的 burst 调度(elevation 窗口 + illuminated) */
  bool enabled{false};
};

/**
 * @brief 带 elevation 距离门控的点目标回波生成(扫描 ScanSAR 模式)。
 *
 * 与 GeneratePointTargetRawEcho 逻辑相同,仅增加距离窗口硬门控:
 *   gate_enabled 时,slant_range ∉ [near_range, far_range) 或 illuminated=false 的目标被跳过。
 * gate_config.enabled=false 时,与 GeneratePointTargetRawEcho 输出完全一致(条带兼容)。
 */
bool GeneratePointTargetRawEchoWithElevationGate(
    const RawEchoConfig& config, const ElevationGateConfig& gate_config,
    const geometry::PlatformPulseState& platform, const std::vector<PointTarget>& targets,
    const signal::ComplexVector& transmit_waveform, RawEchoResult* result);

/**
 * @brief 频域分数延迟:对 input 施加 fractional_delay 样点的子采样时移。
 */
bool ApplyFractionalDelay(const signal::ComplexVector& input, double fractional_delay,
                          signal::ComplexVector* output);

// ────────────────────────────────────────────────────────────
// 接收机噪声
// ────────────────────────────────────────────────────────────

/**
 * @brief 加性噪声规格。
 */
struct NoiseSpec {
  double signal_to_noise_ratio_db{0.0};
  std::uint32_t random_seed{2026U};
};

/**
 * @brief 根据 SNR 向样点序列叠加复高斯噪声(实虚部独立,各 σ/√2)。
 *        信号功率从 samples 估算,噪声功率 = 信号功率 / 10^(snr_db/10)。
 *        使用 DeterministicGaussianSampler(seed) 确保可复现。
 */
bool AddNoise(const NoiseSpec& spec, signal::ComplexVector* samples);

// ────────────────────────────────────────────────────────────
// 杂波
// ────────────────────────────────────────────────────────────

/**
 * @brief 杂波类型。
 */
enum class ClutterType {
  kGamma = 0,  /**< γ 常数模型(陆) */
  kSea = 1,    /**< 海杂波 GIT 经验模型 */
};

/**
 * @brief 杂波模型参数。
 */
struct ClutterModel {
  ClutterType type{ClutterType::kGamma};
  double gamma_constant{0.0};      /**< γ 常数(kGamma 时有效) */
  double sea_state{3.0};           /**< 海况级数(kSea 时有效) */
  double wind_speed_mps{5.0};      /**< 风速 m/s(kSea 时有效) */
  double incidence_angle_rad{0.0}; /**< 局部入射角 */
  double resolution_cell_area_m2{1.0};  /**< 分辨单元面积 m² */
};

/**
 * @brief γ 模型:σ = γ·sin(θ_inc)·A_cell。
 */
double GammaClutterRcs(const ClutterModel& model);

/**
 * @brief 海杂波 GIT 经验模型:σ = f(海况, 风速, 入射角)·A_cell。
 */
double SeaClutterRcs(const ClutterModel& model);

// ────────────────────────────────────────────────────────────
// 面目标场景
// ────────────────────────────────────────────────────────────

/**
 * @brief 面目标/场景描述。
 */
struct SceneDescription {
  geometry::LocalPoint scene_center{};
  double scene_extent_x_m{0.0};
  double scene_extent_y_m{0.0};
  std::vector<PointTarget> point_targets{};
  ClutterModel clutter{};
  double clutter_grid_spacing_m{10.0};
};

/**
 * @brief 生成含点目标 + 杂波的原始回波。
 *        1) 先生成点目标回波,2) 叠加规则网格杂波单元。
 *        grid_spacing ≤ 0 则跳过杂波。
 */
bool GenerateClutterScene(const RawEchoConfig& config,
                          const geometry::PlatformPulseState& platform,
                          const SceneDescription& scene,
                          const signal::ComplexVector& transmit_waveform,
                          RawEchoResult* result);

}  // namespace echo
}  // namespace sar

#endif  // ONEQ_SRC_SAR_ECHO_SAR_ECHO_H_
