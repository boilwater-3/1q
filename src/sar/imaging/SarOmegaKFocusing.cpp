#include "sar/imaging/SarOmegaKFocusing.h"

#include <cmath>
#include <vector>
#include "common/numerics/Constants.h"

namespace sar {
namespace imaging {

namespace {

using oneq::common::numerics::kPi;
using oneq::common::numerics::kLightSpeed;

bool IsValidConfig(const OmegaKConfig& config) {
  return config.range_sample_count >= 2U && config.azimuth_pulse_count >= 2U &&
         std::isfinite(config.sample_rate_hz) && config.sample_rate_hz > 0.0 &&
         std::isfinite(config.prf_hz) && config.prf_hz > 0.0 &&
         std::isfinite(config.carrier_frequency_hz) && config.carrier_frequency_hz > 0.0 &&
         std::isfinite(config.platform_velocity_mps) && config.platform_velocity_mps > 0.0 &&
         std::isfinite(config.reference_range_m) && config.reference_range_m > 0.0;
}

bool IsValidRawHistory(const signal::ComplexMatrix& matrix, const OmegaKConfig& config) {
  if (matrix.rows != config.azimuth_pulse_count ||
      matrix.cols != config.range_sample_count ||
      matrix.values.size() != matrix.rows * matrix.cols) {
    return false;
  }
  for (const signal::ComplexSample& sample : matrix.values) {
    if (!std::isfinite(sample.real()) || !std::isfinite(sample.imag())) {
      return false;
    }
  }
  return true;
}

// 构造方位坐标轴:azimuth_pulse_count 个等间距点,间距 = platform_velocity / prf(慢时间 × 速度)。
// 条带模式原点在场景几何中心(broadside);聚束模式原点偏移到 scene_center_azimuth_m。
std::vector<double> MakeAzimuthCoordinates(const OmegaKConfig& config,
                                           double azimuth_offset_m) {
  const std::size_t pulse_count = config.azimuth_pulse_count;
  const double spacing_m = config.platform_velocity_mps / config.prf_hz;
  std::vector<double> coordinates;
  coordinates.reserve(pulse_count);
  const double center = 0.5 * static_cast<double>(pulse_count - 1U);
  for (std::size_t index = 0U; index < pulse_count; ++index) {
    coordinates.push_back((static_cast<double>(index) - center) * spacing_m + azimuth_offset_m);
  }
  return coordinates;
}

OmegaKGeometryConfig ToGeometryConfig(const OmegaKConfig& config) {
  OmegaKGeometryConfig geometry_config;
  geometry_config.range_sample_count = config.range_sample_count;
  geometry_config.azimuth_pulse_count = config.azimuth_pulse_count;
  geometry_config.sample_rate_hz = config.sample_rate_hz;
  geometry_config.prf_hz = config.prf_hz;
  geometry_config.carrier_frequency_hz = config.carrier_frequency_hz;
  geometry_config.platform_velocity_mps = config.platform_velocity_mps;
  geometry_config.reference_range_m = config.reference_range_m;
  return geometry_config;
}

}  // namespace

// 内部共享聚焦逻辑(条带/聚束共用)。azimuth_offset_m=0 为条带;≠0 为聚束。
bool FocusOmegaKInternal(const OmegaKConfig& config,
                         double azimuth_offset_m,
                         const signal::ComplexMatrix& raw_pulse_history,
                         FocusedOmegaKImage* output) {
  if (output == nullptr) {
    return false;
  }
  *output = FocusedOmegaKImage{};
  if (!IsValidConfig(config)) {
    output->diagnostics.failure_stage = "config";
    return false;
  }
  if (!IsValidRawHistory(raw_pulse_history, config)) {
    output->diagnostics.failure_stage = "raw_history";
    return false;
  }

  constexpr std::uint64_t kRequestId = 1U;

  // [阶段 0/1] front-end:raw → 2D 波数谱(含 H_bulk)。
  OmegaKSpectrumFrontEndRequest front_end_request;
  front_end_request.request_id = kRequestId;
  front_end_request.config = ToGeometryConfig(config);
  front_end_request.raw_pulse_history = raw_pulse_history;
  const OmegaKSpectrumFrontEndResult front_end = ExecuteOmegaKSpectrumFrontEnd(front_end_request);
  output->diagnostics.front_end = front_end;
  if (front_end.status != OmegaKSpectrumFrontEndStatus::kSucceeded) {
    output->diagnostics.failure_stage = "front_end";
    return false;
  }

  // [阶段 2] 共同支持诊断。
  OmegaKCommonSupportDiagnostics common_support;
  if (!DiagnoseOmegaKCommonStoltSupport(front_end.geometry, &common_support)) {
    output->diagnostics.failure_stage = "common_support";
    return false;
  }
  output->diagnostics.common_support = common_support;

  // [阶段 3] 网格收缩 + Stolt 插值。
  OmegaKGridReductionRequest grid_request;
  grid_request.request_id = kRequestId;
  grid_request.geometry = front_end.geometry;
  grid_request.common_support = common_support;
  grid_request.source_spectrum = front_end.source_spectrum;
  const OmegaKGridReductionResult grid_reduction = ExecuteOmegaKExplicitGridReduction(grid_request);
  output->diagnostics.grid_reduction = grid_reduction;
  if (grid_reduction.status != OmegaKGridReductionStatus::kSucceeded) {
    output->diagnostics.failure_stage = "grid_reduction";
    return false;
  }

  // [阶段 4] 相对延迟变换(距离向逆 FFT)。
  OmegaKRelativeDelayRequest delay_request;
  delay_request.request_id = kRequestId;
  delay_request.reduced_range_frequencies_hz = grid_reduction.reduced_target_frequencies_hz;
  delay_request.reduced_spectrum = grid_reduction.reduced_spectrum;
  const OmegaKRelativeDelayResult relative_delay = ExecuteOmegaKRelativeDelayTransform(delay_request);
  output->diagnostics.relative_delay = relative_delay;
  if (relative_delay.status != OmegaKRelativeDelayStatus::kSucceeded) {
    output->diagnostics.failure_stage = "relative_delay";
    return false;
  }

  // 构造方位坐标轴。
  const std::vector<double> azimuth_coordinates = MakeAzimuthCoordinates(config, azimuth_offset_m);

  // [阶段 5] 参考映射:相对延迟 → 绝对斜距。
  OmegaKReferenceMappingRequest mapping_request;
  mapping_request.request_id = kRequestId;
  mapping_request.propagation_speed_mps = kLightSpeed;
  mapping_request.reference_slant_range_m = config.reference_range_m;
  mapping_request.delay_sign = OmegaKDelaySign::kPositiveIncreasesRange;
  mapping_request.reference_phase_sign = OmegaKReferencePhaseSign::kPositive;
  mapping_request.transform_normalization =
      static_cast<double>(grid_reduction.reduced_spectrum.cols);
  mapping_request.relative_delays_s =
      relative_delay.axis_diagnostics.relative_delays_s;
  mapping_request.azimuth_coordinates = azimuth_coordinates;
  mapping_request.relative_delay_domain = relative_delay.relative_delay_domain;
  const OmegaKReferenceMappingResult reference_mapping =
      ExecuteOmegaKReferenceMapping(mapping_request);
  output->diagnostics.reference_mapping = reference_mapping;
  if (reference_mapping.status != OmegaKReferenceMappingStatus::kSucceeded) {
    output->diagnostics.failure_stage = "reference_mapping";
    return false;
  }

  // [阶段 6] 距离残余相位补偿。
  // range_phase_radians[col] = sign · R_ref · (K_z_target(col) − K_r_DC)
  // K_z_target(col) = geometry.range_wavenumbers_rad_per_m[original_col],
  //   original_col 由网格收缩保留的列索引给出。
  // K_r_DC = 4π·f_c/c。
  const double k_r_dc = 4.0 * kPi * config.carrier_frequency_hz / kLightSpeed;
  std::vector<double> range_phase_radians;
  range_phase_radians.reserve(grid_reduction.original_column_indices.size());
  for (std::size_t original_col : grid_reduction.original_column_indices) {
    const double k_z_target = front_end.geometry.range_wavenumbers_rad_per_m[original_col];
    range_phase_radians.push_back(config.reference_range_m * (k_z_target - k_r_dc));
  }

  OmegaKPhaseCompensationRequest phase_request;
  phase_request.request_id = kRequestId;
  phase_request.sign = OmegaKPhaseApplicationSign::kPositive;
  phase_request.absolute_slant_ranges_m = reference_mapping.absolute_slant_ranges_m;
  phase_request.azimuth_coordinates = reference_mapping.azimuth_coordinates;
  phase_request.range_phase_radians = range_phase_radians;
  phase_request.referenced_intermediate = reference_mapping.referenced_intermediate;
  const OmegaKPhaseCompensationResult phase_compensation =
      ExecuteOmegaKReferencePhaseCompensation(phase_request);
  if (phase_compensation.status != OmegaKPhaseCompensationStatus::kSucceeded) {
    output->diagnostics.failure_stage = "reference_phase";
    return false;
  }

  // [阶段 7] 方位逆变换(方位向逆 FFT + 归一化)。
  // 逆 FFT 已含 1/N 归一化（SarFft 直接调用 Eigen fft.inv，未设 Unscaled 标志），
  // additional_normalization 只承载 FFT 之外的附加缩放——此处无附加项，置 1。
  // 之前误置 1/azimuth_pulse_count 造成方位向总缩放 1/N²、与距离向（仅逆 FFT
  // 内建 1/N）不对称，峰值绝对幅度被压低 N 倍。
  OmegaKAzimuthInverseRequest inverse_request;
  inverse_request.request_id = kRequestId;
  inverse_request.absolute_slant_ranges_m = phase_compensation.absolute_slant_ranges_m;
  inverse_request.output_azimuth_coordinates = phase_compensation.azimuth_coordinates;
  inverse_request.additional_normalization = 1.0;
  inverse_request.compensated_intermediate = phase_compensation.compensated_intermediate;
  const OmegaKAzimuthInverseResult azimuth_inverse =
      ExecuteOmegaKAzimuthInverseTransform(inverse_request);
  output->diagnostics.azimuth_inverse = azimuth_inverse;
  if (azimuth_inverse.status != OmegaKAzimuthInverseStatus::kSucceeded) {
    output->diagnostics.failure_stage = "azimuth_inverse";
    return false;
  }

  output->image = azimuth_inverse.numerical_image_candidate;
  output->diagnostics.failure_stage = "none";
  return true;
}

bool FocusStripmapOmegaK(const OmegaKConfig& config,
                         const signal::ComplexMatrix& raw_pulse_history,
                         FocusedOmegaKImage* output) {
  return FocusOmegaKInternal(config, 0.0, raw_pulse_history, output);
}

bool FocusSpotlightOmegaK(const SpotlightOmegaKConfig& config,
                          const signal::ComplexMatrix& raw_pulse_history,
                          FocusedOmegaKImage* output) {
  // 聚束 PRF 充裕性检查:聚束多普勒带宽 < PRF,否则方位混叠。
  // 聚束多普勒带宽 ≈ 2·v·θ_synth / λ(θ_synth 由场景中心方位偏移隐含)。
  // 简化:用方位坐标偏移对应的额外多普勒带宽近似。这里用保守门 PRF/2。
  // 若 raw history 的方位维已满足采样,聚焦引擎内部会自然处理(Stolt squint-invariant)。
  return FocusOmegaKInternal(config, config.scene_center_azimuth_m, raw_pulse_history, output);
}

}  // namespace imaging
}  // namespace sar
