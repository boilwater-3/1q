#include <gtest/gtest.h>

#include <complex>
#include <vector>

#include "sar/imaging/SarOmegaKGeometry.h"
#include "sar/imaging/SarOmegaKStoltInterpolation.h"

namespace sar {
namespace imaging {
namespace {

signal::ComplexSample Affine(double row_offset, double frequency_hz) {
  return signal::ComplexSample(row_offset + 2.0 * frequency_hz,
                               -row_offset + 3.0 * frequency_hz);
}

StoltInterpolationRequest AffineRequest() {
  StoltInterpolationRequest request;
  request.source_range_frequencies_hz = {0.0, 2.0, -2.0, -1.0, 1.0};
  request.target_range_frequencies_hz = request.source_range_frequencies_hz;
  request.source_spectrum.rows = 2U;
  request.source_spectrum.cols = 5U;
  request.source_spectrum.values.resize(10U);
  for (std::size_t row = 0U; row < 2U; ++row) {
    for (std::size_t col = 0U; col < 5U; ++col) {
      request.source_spectrum(row, col) =
          Affine(10.0 * static_cast<double>(row),
                 request.source_range_frequencies_hz[col]);
    }
  }
  request.source_frequency_queries_hz.rows = 2U;
  request.source_frequency_queries_hz.cols = 5U;
  request.source_frequency_queries_hz.values = {
      {0.0, 0.0}, {1.5, 0.0}, {-1.5, 0.0}, {-1.0, 0.0}, {0.5, 0.0},
      {0.0, 0.0}, {1.0, 0.0}, {-2.0, 0.0}, {-0.5, 0.0}, {2.0, 0.0},
  };
  for (std::size_t col = 0U; col < request.target_range_frequencies_hz.size(); ++col) {
    request.target_range_frequencies_hz[col] = request.source_frequency_queries_hz(0U, col).real();
  }
  return request;
}

TEST(SarOmegaKStoltInterpolationTest, RecoversAffineComplexTruthInUnshiftedOrder) {
  const StoltInterpolationRequest request = AffineRequest();
  const StoltInterpolationResult result = InterpolateOmegaKStoltLinear(request);

  ASSERT_EQ(result.status, StoltInterpolationStatus::kSucceeded);
  EXPECT_EQ(result.reason, StoltInterpolationRejectionReason::kNone);
  EXPECT_EQ(result.diagnostics.exact_hit_count, 6U);
  EXPECT_EQ(result.diagnostics.linear_interpolation_count, 4U);
  for (std::size_t row = 0U; row < 2U; ++row) {
    for (std::size_t col = 0U; col < 5U; ++col) {
      EXPECT_NEAR(result.interpolated_spectrum(row, col).real(),
                  Affine(10.0 * static_cast<double>(row),
                         request.source_frequency_queries_hz(row, col).real()).real(),
                  1.0e-12);
      EXPECT_NEAR(result.interpolated_spectrum(row, col).imag(),
                  Affine(10.0 * static_cast<double>(row),
                         request.source_frequency_queries_hz(row, col).real()).imag(),
                  1.0e-12);
    }
  }
}

TEST(SarOmegaKStoltInterpolationTest, ZeroShiftStrictlyPreservesSpectrum) {
  StoltInterpolationRequest request = AffineRequest();
  for (std::size_t row = 0U; row < request.source_spectrum.rows; ++row) {
    for (std::size_t col = 0U; col < request.source_spectrum.cols; ++col) {
      request.source_frequency_queries_hz(row, col) =
          signal::ComplexSample(request.source_range_frequencies_hz[col], 0.0);
    }
  }
  request.target_range_frequencies_hz = request.source_range_frequencies_hz;
  const StoltInterpolationResult result = InterpolateOmegaKStoltLinear(request);

  ASSERT_EQ(result.status, StoltInterpolationStatus::kSucceeded);
  EXPECT_EQ(result.interpolated_spectrum.values, request.source_spectrum.values);
  EXPECT_EQ(result.diagnostics.exact_hit_count, 10U);
  EXPECT_EQ(result.diagnostics.linear_interpolation_count, 0U);
}

TEST(SarOmegaKStoltInterpolationTest, RejectsOutOfSupportQueriesFromFullGeometryGrid) {
  OmegaKGeometryConfig config;
  config.range_sample_count = 5U;
  config.azimuth_pulse_count = 5U;
  config.sample_rate_hz = 2.0e8;
  config.prf_hz = 100.0;
  config.carrier_frequency_hz = 1.0e9;
  config.platform_velocity_mps = 100.0;
  config.reference_range_m = 1000.0;
  OmegaKGeometryDiagnostics geometry;
  ASSERT_TRUE(EvaluateOmegaKStoltGeometry(config, &geometry));
  ASSERT_FALSE(geometry.valid);
  ASSERT_GT(geometry.out_of_support_stolt_query_count, 0U);

  StoltInterpolationRequest request;
  request.source_range_frequencies_hz = geometry.range_frequencies_hz;
  request.target_range_frequencies_hz = geometry.range_frequencies_hz;
  request.source_spectrum.rows = config.azimuth_pulse_count;
  request.source_spectrum.cols = config.range_sample_count;
  request.source_spectrum.values.resize(config.azimuth_pulse_count * config.range_sample_count);
  request.source_frequency_queries_hz.rows = config.azimuth_pulse_count;
  request.source_frequency_queries_hz.cols = config.range_sample_count;
  request.source_frequency_queries_hz.values.resize(request.source_spectrum.values.size());
  for (std::size_t row = 0U; row < config.azimuth_pulse_count; ++row) {
    for (std::size_t col = 0U; col < config.range_sample_count; ++col) {
      request.source_spectrum(row, col) =
          Affine(10.0 * static_cast<double>(row), geometry.range_frequencies_hz[col]);
      request.source_frequency_queries_hz(row, col) =
          signal::ComplexSample(
              geometry.source_range_frequency_queries_hz[row * config.range_sample_count + col],
              0.0);
    }
  }

  const StoltInterpolationResult result = InterpolateOmegaKStoltLinear(request);
  EXPECT_EQ(result.status, StoltInterpolationStatus::kRejected);
  EXPECT_EQ(result.reason, StoltInterpolationRejectionReason::kOutOfSupportQuery);
  EXPECT_TRUE(result.interpolated_spectrum.values.empty());
}

TEST(SarOmegaKStoltInterpolationTest, RejectsOutOfSupportQueryAtomically) {
  StoltInterpolationRequest request = AffineRequest();
  request.source_frequency_queries_hz(1U, 4U) = signal::ComplexSample(2.1, 0.0);
  const StoltInterpolationResult result = InterpolateOmegaKStoltLinear(request);

  EXPECT_EQ(result.status, StoltInterpolationStatus::kRejected);
  EXPECT_EQ(result.reason, StoltInterpolationRejectionReason::kOutOfSupportQuery);
  EXPECT_EQ(result.diagnostics.out_of_support_query_count, 1U);
  EXPECT_TRUE(result.interpolated_spectrum.values.empty());
}

TEST(SarOmegaKStoltInterpolationTest, RejectsInvalidStructureAndIsDeterministic) {
  const StoltInterpolationRequest request = AffineRequest();
  const StoltInterpolationResult first = InterpolateOmegaKStoltLinear(request);
  const StoltInterpolationResult second = InterpolateOmegaKStoltLinear(request);
  EXPECT_EQ(first.interpolated_spectrum.values, second.interpolated_spectrum.values);

  StoltInterpolationRequest duplicate_axis = request;
  duplicate_axis.source_range_frequencies_hz[1] = 1.0;
  EXPECT_EQ(InterpolateOmegaKStoltLinear(duplicate_axis).reason,
            StoltInterpolationRejectionReason::kInvalidFrequencyAxis);

  StoltInterpolationRequest bad_queries = request;
  bad_queries.source_frequency_queries_hz.values.pop_back();
  EXPECT_EQ(InterpolateOmegaKStoltLinear(bad_queries).reason,
            StoltInterpolationRejectionReason::kInvalidQueries);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
