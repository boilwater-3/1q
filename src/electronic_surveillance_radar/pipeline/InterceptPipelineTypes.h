/**
 * @file InterceptPipelineTypes.h
 * @brief 定义 ESR 内部流水线配置、运行态与执行结果类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_INTERCEPT_PIPELINE_TYPES_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_INTERCEPT_PIPELINE_TYPES_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "1q/electronic_surveillance_radar/config/EsrHardwareConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"

namespace electronic_surveillance_radar {
namespace extension {

using session::EmitterOutputFrame;
using session::ObservationOutputFrame;

/**
 * @brief InterceptDetectionConfig 描述截获判定配置。
 */
struct InterceptDetectionConfig {
  float minimum_snr_db{6.0f};
  float max_detect_range_m{450000.0f};
  float min_dynamic_range_margin_db{-3.0f};
  float boundary_resolution_m{50.0f};
  int boundary_max_iterations{32};
};

/**
 * @brief InterceptIntegrationMode 表示统计检测中的积累模式。
 */
enum class InterceptIntegrationMode { kNonCoherent = 0, kCoherent };

/**
 * @brief InterceptStatisticalDetectionConfig 描述统计检测与门限映射参数。
 */
struct InterceptStatisticalDetectionConfig {
  float pfa{1.0e-6f};
  float minimum_snr_db{6.0f};
  std::uint32_t pulse_count{8U};
  InterceptIntegrationMode integration_mode{InterceptIntegrationMode::kNonCoherent};
  float threshold_scale{1.0f};
  bool enable_statistical_detection{true};
};

/**
 * @brief InterceptScanConfig 描述扫描调度配置。
 */
struct InterceptScanConfig {
  float scan_start_az_deg{-60.0f};
  float scan_end_az_deg{60.0f};
  float scan_start_el_deg{-10.0f};
  float scan_end_el_deg{10.0f};
  float az_step_deg{5.0f};
  float el_step_deg{5.0f};
  int scan_start_pos{0};
  int scan_sequence{0};
};

/**
 * @brief InterceptAlgorithmConfig 描述算法辅助配置。
 */
struct InterceptAlgorithmConfig {
  unsigned int random_seed{20260323U};
  float angle_error_coefficient{0.51f};
  float rf_error_coefficient{0.5f};
  float bandwidth_error_coefficient{0.5f};
  float pri_error_coefficient{0.5f};
  float pulse_width_error_coefficient{0.5f};
};

/**
 * @brief InterceptPreprocessConfig 描述观测预处理配置。
 */
struct InterceptPreprocessConfig {
  float dedup_time_window_sec{5.0e-6f};
  double dedup_rf_window_hz{1.0e6};
  double dedup_pw_window_sec{3.0e-7};
  float dedup_az_window_deg{1.0f};
  float dedup_el_window_deg{1.0f};
  bool normalize_quality{true};
};

/**
 * @brief InterceptClusterConfig 描述聚类配置与特征尺度。
 */
struct InterceptClusterConfig {
  float radius{1.0f};
  std::uint32_t min_points{1U};
  float rf_scale_hz{5.0e6f};
  float pw_scale_sec{1.0e-6f};
  float az_scale_deg{2.0f};
  float el_scale_deg{2.0f};
  float snr_scale_db{8.0f};
};

/**
 * @brief InterceptSpectralAnalysisConfig 描述频谱特征分析与标签配置。
 */
struct InterceptSpectralAnalysisConfig {
  bool enable{true};
  std::uint32_t min_sequence_length{4U};
  std::uint32_t fft_length{16U};
  float broadband_occupancy_threshold{0.45f};
  float agile_stability_threshold_hz{1.5e6f};
  float agile_peak_sparsity_threshold{0.45f};
  float occupancy_peak_floor_ratio{0.20f};
};

/**
 * @brief InterceptAssociationConfig 描述假设关联与状态管理配置。
 */
struct InterceptAssociationConfig {
  float gate_distance{1.2f};
  std::uint32_t confirm_hits{3U};
  std::uint32_t max_missed_cycles{5U};
  float confidence_alpha{0.3f};
  bool output_tentative{true};
};

/**
 * @brief InterceptSuppressionModelConfig 描述压制分量建模参数。
 */
struct InterceptSuppressionModelConfig {
  float suppression_noise_scale{1.0f};
  float suppression_mark_threshold_w{1.0e-12f};
};

/**
 * @brief InterceptRuntimeConfig 描述会话层注入的运行态参数。
 */
struct InterceptRuntimeConfig {
  bool sensor_enabled{true};
  bool use_fixed_receiver_window{false};
  double receiver_lower_hz{0.0};
  double receiver_upper_hz{0.0};
  float integrated_receive_loss_db{0.0f};
  float antenna_mount_az_deg{0.0f};
  float antenna_mount_el_deg{0.0f};
  float scan_rate_hz{1.0f};
  config::EsrHardwareConfig receiver_hardware{};
};

/**
 * @brief InterceptPipelineConfig 描述电子侦察流水线顶层配置。
 */
struct InterceptPipelineConfig {
  InterceptDetectionConfig detection{};
  InterceptStatisticalDetectionConfig statistical_detection{};
  InterceptScanConfig scan{};
  InterceptAlgorithmConfig algorithm{};
  InterceptPreprocessConfig preprocess{};
  InterceptClusterConfig cluster{};
  InterceptSpectralAnalysisConfig spectral_analysis{};
  InterceptAssociationConfig association{};
  InterceptSuppressionModelConfig suppression_model{};
};

/**
 * @brief InterceptPipelineResult 表示电子侦察流水线单周期执行结果。
 */
struct InterceptPipelineResult {
  ObservationOutputFrame observation_output{};
  EmitterOutputFrame emitter_output{};
  float scan_azimuth_deg{0.0f}; /**< 本周期波束中心方位角（单位：deg，平台系，含天线安装角）。 */
  bool sensor_powered_off{false}; /**< 设备关机导致本周期未执行，而非合法空观测。 */
  bool rf_v2_rejected{false}; /**< RF v2 前端无法在不破坏物理合同的前提下求解。 */
  session::EsrIssueList issues{}; /**< 正常执行周期按发射源排除的 kInfo 诊断（规则 13b），
                                      经 controller 转写进 EsrCycleResult。 */
};

/**
 * @brief InterceptPipelineRuntimeState 描述 ESR 流水线运行态快照。
 */
struct InterceptPipelineRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  std::shared_ptr<const void> snapshot;
};

}  // namespace extension
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_INTERCEPT_PIPELINE_TYPES_H_
