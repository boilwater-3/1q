#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKGridReduction.h"

namespace sar {
namespace imaging {
namespace {

signal::ComplexSample Affine(double row_offset, double frequency_hz) {
  return signal::ComplexSample(row_offset + 2.0 * frequency_hz,
                               -row_offset + 3.0 * frequency_hz);
}

OmegaKGridReductionRequest ValidRequest() {
  OmegaKGeometryConfig config;
  config.range_sample_count = 5U;
  config.azimuth_pulse_count = 5U;
  config.sample_rate_hz = 2.0e8;
  config.prf_hz = 100.0;
  config.carrier_frequency_hz = 1.0e9;
  config.platform_velocity_mps = 100.0;
  config.reference_range_m = 1000.0;

  OmegaKGridReductionRequest request;
  request.request_id = 41U;
  EXPECT_TRUE(EvaluateOmegaKStoltGeometry(config, &request.geometry));
  EXPECT_TRUE(DiagnoseOmegaKCommonStoltSupport(request.geometry, &request.common_support));
  request.source_spectrum.rows = config.azimuth_pulse_count;
  request.source_spectrum.cols = config.range_sample_count;
  request.source_spectrum.values.resize(config.azimuth_pulse_count * config.range_sample_count);
  for (std::size_t row = 0U; row < config.azimuth_pulse_count; ++row) {
    for (std::size_t col = 0U; col < config.range_sample_count; ++col) {
      request.source_spectrum(row, col) =
          Affine(10.0 * static_cast<double>(row), request.geometry.range_frequencies_hz[col]);
    }
  }
  return request;
}

TEST(SarOmegaKGridReductionTest, ReducesToCommonWindowAndRecoversAffineTruth) {
  const OmegaKGridReductionRequest request = ValidRequest();
  const OmegaKGridReductionResult result = ExecuteOmegaKExplicitGridReduction(request);

  ASSERT_EQ(result.status, OmegaKGridReductionStatus::kSucceeded);
  ASSERT_EQ(result.reduced_target_frequencies_hz.size(),
            request.common_support.largest_contiguous_column_count);
  EXPECT_EQ(result.original_column_indices,
            request.common_support.largest_contiguous_original_column_indices);
  EXPECT_EQ(result.reduced_spectrum.rows, request.source_spectrum.rows);
  EXPECT_EQ(result.reduced_spectrum.cols, result.reduced_target_frequencies_hz.size());
  for (std::size_t row = 0U; row < result.reduced_spectrum.rows; ++row) {
    for (std::size_t col = 0U; col < result.reduced_spectrum.cols; ++col) {
      const double query_hz =
          request.geometry.source_range_frequency_queries_hz[
              row * request.geometry.range_frequency_bin_count +
              result.original_column_indices[col]];
      const signal::ComplexSample expected = Affine(10.0 * static_cast<double>(row), query_hz);
      EXPECT_NEAR(result.reduced_spectrum(row, col).real(), expected.real(), 1.0e-6);
      EXPECT_NEAR(result.reduced_spectrum(row, col).imag(), expected.imag(), 1.0e-6);
    }
  }
}

TEST(SarOmegaKGridReductionTest, RejectsTamperedSupportAtomically) {
  OmegaKGridReductionRequest request = ValidRequest();
  request.common_support.common_valid_column_mask[
      request.common_support.largest_contiguous_original_column_indices.front()] = false;
  const OmegaKGridReductionResult result = ExecuteOmegaKExplicitGridReduction(request);
  EXPECT_EQ(result.status, OmegaKGridReductionStatus::kRejected);
  EXPECT_EQ(result.reason, OmegaKGridReductionReason::kInvalidCommonSupport);
  EXPECT_TRUE(result.reduced_spectrum.values.empty());
}

TEST(SarOmegaKGridReductionTest, RejectsInvalidRequestAndIsDeterministic) {
  const OmegaKGridReductionRequest request = ValidRequest();
  const OmegaKGridReductionResult first = ExecuteOmegaKExplicitGridReduction(request);
  const OmegaKGridReductionResult second = ExecuteOmegaKExplicitGridReduction(request);
  EXPECT_EQ(first.reduced_spectrum.values, second.reduced_spectrum.values);
  EXPECT_EQ(first.reduced_target_frequencies_hz, second.reduced_target_frequencies_hz);

  OmegaKGridReductionRequest invalid = request;
  invalid.request_id = 0U;
  EXPECT_EQ(ExecuteOmegaKExplicitGridReduction(invalid).reason,
            OmegaKGridReductionReason::kInvalidRequestId);
  invalid = request;
  invalid.source_spectrum.values.pop_back();
  EXPECT_EQ(ExecuteOmegaKExplicitGridReduction(invalid).reason,
            OmegaKGridReductionReason::kInvalidSourceSpectrum);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
