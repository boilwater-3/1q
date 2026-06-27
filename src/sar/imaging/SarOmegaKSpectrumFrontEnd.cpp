#include "sar/imaging/SarOmegaKSpectrumFrontEnd.h"

#include <cmath>

namespace sar {
namespace imaging {

namespace {

OmegaKSpectrumFrontEndResult Reject(const OmegaKSpectrumFrontEndRequest& request,
                                    OmegaKSpectrumFrontEndReason reason) {
  OmegaKSpectrumFrontEndResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool IsValidConfig(const OmegaKGeometryConfig& config) {
  // azimuth_pulse_count >= 2:单脉冲无法构成方位合成孔径,且 N=1 的方位 FFT
  // (FftCols)在 Eigen FFT 下行为未定义。物理上 Omega-K 聚焦要求多脉冲。
  return config.range_sample_count >= 2U && config.azimuth_pulse_count >= 2U &&
         IsPositiveFinite(config.sample_rate_hz) && IsPositiveFinite(config.prf_hz) &&
         IsPositiveFinite(config.carrier_frequency_hz) &&
         IsPositiveFinite(config.platform_velocity_mps) &&
         IsPositiveFinite(config.reference_range_m);
}

bool IsValidMatrix(const signal::ComplexMatrix& matrix, std::size_t expected_rows,
                   std::size_t expected_cols) {
  if (matrix.rows != expected_rows || matrix.cols != expected_cols ||
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

}  // namespace

OmegaKSpectrumFrontEndResult ExecuteOmegaKSpectrumFrontEnd(
    const OmegaKSpectrumFrontEndRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kInvalidRequestId);
  }
  if (!IsValidConfig(request.config)) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kInvalidConfig);
  }
  if (!IsValidMatrix(request.raw_pulse_history, request.config.azimuth_pulse_count,
                     request.config.range_sample_count)) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kInvalidRawHistory);
  }

  OmegaKGeometryDiagnostics geometry;
  if (!EvaluateOmegaKStoltGeometry(request.config, &geometry)) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kGeometryFailure);
  }
  const std::size_t rows = geometry.azimuth_frequency_bin_count;
  const std::size_t cols = geometry.range_frequency_bin_count;
  if (geometry.propagation_wavenumbers_rad_per_m.size() != rows * cols) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kGeometryFailure);
  }

  // 距离向正向 FFT(沿 cols/距离维)。
  signal::ComplexMatrix range_spectrum;
  if (!signal::FftRows(request.raw_pulse_history, false, &range_spectrum)) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kTransformFailure);
  }
  if (!IsValidMatrix(range_spectrum, rows, cols)) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kTransformFailure);
  }

  // 方位向正向 FFT(沿 rows/方位维)。
  signal::ComplexMatrix two_d_spectrum;
  if (!signal::FftCols(range_spectrum, false, &two_d_spectrum)) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kTransformFailure);
  }
  if (!IsValidMatrix(two_d_spectrum, rows, cols)) {
    return Reject(request, OmegaKSpectrumFrontEndReason::kTransformFailure);
  }

  // 施加 bulk 参考函数 H_bulk = exp(+j · R_ref · K_z)。
  // 距离压缩被吸收进 H_bulk 的 K_r 依赖。无效色散点 K_z=0 → 相位恒零 → 自动退化。
  signal::ComplexMatrix source_spectrum = two_d_spectrum;
  for (std::size_t index = 0U; index < source_spectrum.values.size(); ++index) {
    const double phase =
        request.config.reference_range_m * geometry.propagation_wavenumbers_rad_per_m[index];
    const signal::ComplexSample rotation(std::cos(phase), std::sin(phase));
    source_spectrum.values[index] *= rotation;
    if (!std::isfinite(source_spectrum.values[index].real()) ||
        !std::isfinite(source_spectrum.values[index].imag())) {
      return Reject(request, OmegaKSpectrumFrontEndReason::kTransformFailure);
    }
  }

  OmegaKSpectrumFrontEndResult result;
  result.request_id = request.request_id;
  result.status = OmegaKSpectrumFrontEndStatus::kSucceeded;
  result.reason = OmegaKSpectrumFrontEndReason::kNone;
  result.geometry = geometry;
  result.source_spectrum = source_spectrum;
  return result;
}

}  // namespace imaging
}  // namespace sar
