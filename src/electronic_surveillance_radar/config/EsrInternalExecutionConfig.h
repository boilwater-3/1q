/**
 * @file EsrInternalExecutionConfig.h
 * @brief 定义 ESR 内部执行态统一配置类型。
 *
 * EsrInternalExecutionConfig 是会话层配置解析后的内部执行态唯一真值，
 * 取代此前分别携带 extension::InterceptPipelineConfig /
 * extension::InterceptRuntimeConfig 的 ResolvedEsrSessionConfig 模式。
 * Phase 1 创建内部子结构，Phase 2 将 DetectionConfig 变更为 using 别名。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_CONFIG_ESR_INTERNAL_EXECUTION_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_CONFIG_ESR_INTERNAL_EXECUTION_CONFIG_H_

#include <cmath>
#include <cstdint>

#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrHardwareConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrMissionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrPolicyConfig.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {

/** @brief DetectionConfig 为 EsrDetectionPolicyConfig 别名（不含 profile 元数据）。 */
using DetectionConfig = config::EsrDetectionPolicyConfig;

// ── Intercept 子配置 ─────────────────────────────────────────────────

/** @brief 截获检测子配置。 */
struct InterceptDetectionConfig {
  float max_detect_range_m{450000.0f};
  float min_dynamic_range_margin_db{-3.0f};
  float boundary_resolution_m{50.0f};
  int boundary_max_iterations{32};
};

/** @brief 算法辅助子配置。 */
struct InterceptAlgorithmConfig {
  unsigned int random_seed{20260323U};
  float angle_error_coefficient{0.51f};
  float rf_error_coefficient{0.5f};
  float bandwidth_error_coefficient{0.5f};
  float pri_error_coefficient{0.5f};
  float pulse_width_error_coefficient{0.5f};
};

/** @brief 观测预处理子配置。 */
struct InterceptPreprocessConfig {
  float dedup_time_window_sec{5.0e-6f};
  double dedup_rf_window_hz{1.0e6};
  double dedup_pw_window_sec{3.0e-7};
  float dedup_az_window_deg{1.0f};
  float dedup_el_window_deg{1.0f};
  bool normalize_quality{true};
};

/** @brief 聚类子配置。 */
struct InterceptClusterConfig {
  float radius{1.0f};
  std::uint32_t min_points{1U};
  float rf_scale_hz{5.0e6f};
  float pw_scale_sec{1.0e-6f};
  float az_scale_deg{2.0f};
  float el_scale_deg{2.0f};
  float snr_scale_db{8.0f};
};

/** @brief 频谱分析子配置。 */
struct InterceptSpectralAnalysisConfig {
  bool enable{true};
  std::uint32_t min_sequence_length{4U};
  std::uint32_t fft_length{16U};
  float broadband_occupancy_threshold{0.45f};
  float agile_stability_threshold_hz{1.5e6f};
  float agile_peak_sparsity_threshold{0.45f};
  float occupancy_peak_floor_ratio{0.20f};
};

/** @brief 压制分量建模子配置。 */
struct InterceptSuppressionModelConfig {
  float suppression_noise_scale{1.0f};
  float suppression_mark_threshold_w{1.0e-12f};
};

/** @brief 截获流水线子装配。 */
struct InterceptConfig {
  InterceptDetectionConfig detection{};
  InterceptAlgorithmConfig algorithm{};
  InterceptPreprocessConfig preprocess{};
  InterceptClusterConfig cluster{};
  InterceptSpectralAnalysisConfig spectral_analysis{};
  InterceptSuppressionModelConfig suppression{};
};

// ── Runtime 子配置 ─────────────────────────────────────────────────

/** @brief 运行期积累器子配置。 */
struct RuntimeIntegratorConfig {
  extension::InterceptIntegrationMode integration_mode{
      extension::InterceptIntegrationMode::kNonCoherent};
};

/** @brief 运行期跟踪子配置。 */
struct RuntimeTrackConfig {
  float gate_distance{1.2f};
  std::uint32_t confirm_hits{3U};
  std::uint32_t max_missed_cycles{5U};
  float confidence_alpha{0.3f};
  bool output_tentative{true};
};

/** @brief 运行期截获参数装配。 */
struct RuntimeConfig {
  RuntimeIntegratorConfig integrator{};
  RuntimeTrackConfig track{};
};

// ── 内部环境子配置 ───────────────────────────────────────────────

/** @brief 内部环境执行视图；当前无专属字段，复用场景配置类型。 */
using EnvironmentConfig = config::EsrEnvironmentScenarioConfig;

/**
 * @brief EsrInternalExecutionConfig 描述会话装配前的统一内部解析结果。
 *
 * 包含会话层输入经过解析后的所有执行态参数，
 * 替换原先的 ResolvedEsrSessionConfig + 分立的 pipeline/runtime/environment config。
 */
struct EsrInternalExecutionConfig {
  bool sensor_enabled{true};            /**< 全局设备开关（COMMON-OQ-4 字段提升） */
  config::EsrHardwareConfig hardware{}; /**< 装备固有参数（using 别名直接赋值） */
  config::EsrMissionConfig mission{};   /**< 任务域参数（using 别名直接赋值） */
  extension::InterceptScanConfig
      resolved_scan{};             /**< 由 mission.scan + hardware 解析生成的扫描配置 */
  DetectionConfig base_detection{}; /**< 未施加工作模式倍率的探测策略真值。 */
  DetectionConfig detection{};     /**< 解析后的探测策略参数（SNR/PFA/脉冲/门限/统计） */
  InterceptConfig intercept{};     /**< 截获流水线配置（算法/预处理/检测子/聚类/频谱/建模） */
  RuntimeConfig runtime{};         /**< 运行期可变截获参数（积累器/跟踪关联） */
  EnvironmentConfig environment{}; /**< 环境执行态参数（大气物理/派生上下文/preset） */
};

/**
 * @brief 从 EsrInternalExecutionConfig 构建 extension::InterceptPipelineConfig。
 *
 * 用于 Pipeline 内部将内部配置转换为流水线组件需要的扩展类型。
 * @param[in] internal 内部执行态配置。
 * @return 扩展类型流水线配置。
 */
inline extension::InterceptPipelineConfig BuildPipelineConfig(
    const EsrInternalExecutionConfig& internal) {
  extension::InterceptPipelineConfig ext;
  ext.detection.minimum_snr_db = internal.detection.minimum_snr_db;
  ext.detection.max_detect_range_m = internal.intercept.detection.max_detect_range_m;
  ext.detection.min_dynamic_range_margin_db =
      internal.intercept.detection.min_dynamic_range_margin_db;
  ext.detection.boundary_resolution_m = internal.intercept.detection.boundary_resolution_m;
  ext.detection.boundary_max_iterations = internal.intercept.detection.boundary_max_iterations;

  ext.statistical_detection.pfa = internal.detection.pfa;
  ext.statistical_detection.minimum_snr_db = internal.detection.minimum_snr_db;
  ext.statistical_detection.pulse_count = internal.detection.pulse_count;
  ext.statistical_detection.integration_mode = internal.runtime.integrator.integration_mode;
  ext.statistical_detection.threshold_scale = internal.detection.threshold_scale;
  ext.statistical_detection.enable_statistical_detection =
      internal.detection.enable_statistical_detection;

  ext.scan = internal.resolved_scan;

  ext.algorithm.random_seed = internal.intercept.algorithm.random_seed;
  ext.algorithm.angle_error_coefficient = internal.intercept.algorithm.angle_error_coefficient;
  ext.algorithm.rf_error_coefficient = internal.intercept.algorithm.rf_error_coefficient;
  ext.algorithm.bandwidth_error_coefficient = internal.intercept.algorithm.bandwidth_error_coefficient;
  ext.algorithm.pri_error_coefficient = internal.intercept.algorithm.pri_error_coefficient;
  ext.algorithm.pulse_width_error_coefficient =
      internal.intercept.algorithm.pulse_width_error_coefficient;

  ext.preprocess = extension::InterceptPreprocessConfig();
  ext.preprocess.dedup_time_window_sec = internal.intercept.preprocess.dedup_time_window_sec;
  ext.preprocess.dedup_rf_window_hz = internal.intercept.preprocess.dedup_rf_window_hz;
  ext.preprocess.dedup_pw_window_sec = internal.intercept.preprocess.dedup_pw_window_sec;
  ext.preprocess.dedup_az_window_deg = internal.intercept.preprocess.dedup_az_window_deg;
  ext.preprocess.dedup_el_window_deg = internal.intercept.preprocess.dedup_el_window_deg;
  ext.preprocess.normalize_quality = internal.intercept.preprocess.normalize_quality;

  ext.cluster = extension::InterceptClusterConfig();
  ext.cluster.radius = internal.intercept.cluster.radius;
  ext.cluster.min_points = internal.intercept.cluster.min_points;
  ext.cluster.rf_scale_hz = internal.intercept.cluster.rf_scale_hz;
  ext.cluster.pw_scale_sec = internal.intercept.cluster.pw_scale_sec;
  ext.cluster.az_scale_deg = internal.intercept.cluster.az_scale_deg;
  ext.cluster.el_scale_deg = internal.intercept.cluster.el_scale_deg;
  ext.cluster.snr_scale_db = internal.intercept.cluster.snr_scale_db;

  ext.spectral_analysis = extension::InterceptSpectralAnalysisConfig();
  ext.spectral_analysis.enable = internal.intercept.spectral_analysis.enable;
  ext.spectral_analysis.min_sequence_length =
      internal.intercept.spectral_analysis.min_sequence_length;
  ext.spectral_analysis.fft_length = internal.intercept.spectral_analysis.fft_length;
  ext.spectral_analysis.broadband_occupancy_threshold =
      internal.intercept.spectral_analysis.broadband_occupancy_threshold;
  ext.spectral_analysis.agile_stability_threshold_hz =
      internal.intercept.spectral_analysis.agile_stability_threshold_hz;
  ext.spectral_analysis.agile_peak_sparsity_threshold =
      internal.intercept.spectral_analysis.agile_peak_sparsity_threshold;
  ext.spectral_analysis.occupancy_peak_floor_ratio =
      internal.intercept.spectral_analysis.occupancy_peak_floor_ratio;

  ext.suppression_model.suppression_noise_scale =
      internal.intercept.suppression.suppression_noise_scale;
  ext.suppression_model.suppression_mark_threshold_w =
      internal.intercept.suppression.suppression_mark_threshold_w;

  ext.association.gate_distance = internal.runtime.track.gate_distance;
  ext.association.confirm_hits = internal.runtime.track.confirm_hits;
  ext.association.max_missed_cycles = internal.runtime.track.max_missed_cycles;
  ext.association.confidence_alpha = internal.runtime.track.confidence_alpha;
  ext.association.output_tentative = internal.runtime.track.output_tentative;

  return ext;
}

/**
 * @brief 从 EsrInternalExecutionConfig 构建 extension::InterceptRuntimeConfig。
 * @param[in] internal 内部执行态配置。
 * @return 扩展类型运行时配置。
 */
inline extension::InterceptRuntimeConfig BuildRuntimeConfig(
    const EsrInternalExecutionConfig& internal) {
  extension::InterceptRuntimeConfig rt;
  rt.sensor_enabled = internal.sensor_enabled;
  rt.antenna_mount_az_deg = internal.hardware.antenna_mount_az_deg;
  rt.antenna_mount_el_deg = internal.hardware.antenna_mount_el_deg;
  rt.integrated_receive_loss_db = internal.hardware.integrated_receive_loss_db;
  rt.scan_rate_hz = internal.mission.scan.scan_rate_hz;
  rt.receiver_hardware = internal.hardware;
  // Use fixed receiver window when hardware band is valid
  if (std::isfinite(internal.hardware.receiver_band_lower_hz) &&
      std::isfinite(internal.hardware.receiver_band_upper_hz) &&
      internal.hardware.receiver_band_upper_hz > internal.hardware.receiver_band_lower_hz) {
    rt.use_fixed_receiver_window = true;
    rt.receiver_lower_hz = internal.hardware.receiver_band_lower_hz;
    rt.receiver_upper_hz = internal.hardware.receiver_band_upper_hz;
  } else {
    rt.use_fixed_receiver_window = false;
    rt.receiver_lower_hz = 0.0;
    rt.receiver_upper_hz = 0.0;
  }
  return rt;
}

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_CONFIG_ESR_INTERNAL_EXECUTION_CONFIG_H_
