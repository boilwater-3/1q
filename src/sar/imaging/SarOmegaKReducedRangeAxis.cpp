#include "sar/imaging/SarOmegaKReducedRangeAxis.h"

#include <algorithm>
#include <cmath>
#include "common/numerics/Constants.h"

namespace sar {
namespace imaging {

namespace {

using oneq::common::numerics::kLightSpeed;

}  // namespace

bool DiagnoseOmegaKReducedRangeAxis(
    const std::vector<double>& reduced_range_frequencies_hz,
    OmegaKReducedRangeAxisDiagnostics* diagnostics) {
  if (diagnostics == nullptr) {
    return false;
  }
  *diagnostics = OmegaKReducedRangeAxisDiagnostics{};
  if (reduced_range_frequencies_hz.size() < 2U) {
    return false;
  }
  for (std::size_t index = 0U; index < reduced_range_frequencies_hz.size(); ++index) {
    if (!std::isfinite(reduced_range_frequencies_hz[index]) ||
        (index > 0U &&
         reduced_range_frequencies_hz[index] <= reduced_range_frequencies_hz[index - 1U])) {
      return false;
    }
  }

  const double spacing_hz =
      reduced_range_frequencies_hz[1] - reduced_range_frequencies_hz[0];
  const double tolerance_hz =
      std::max(1.0e-12, std::abs(spacing_hz) * 1.0e-12);
  for (std::size_t index = 1U; index < reduced_range_frequencies_hz.size(); ++index) {
    const double actual_spacing =
        reduced_range_frequencies_hz[index] - reduced_range_frequencies_hz[index - 1U];
    diagnostics->maximum_abs_spacing_deviation_hz =
        std::max(diagnostics->maximum_abs_spacing_deviation_hz,
                 std::abs(actual_spacing - spacing_hz));
  }
  if (diagnostics->maximum_abs_spacing_deviation_hz > tolerance_hz) {
    *diagnostics = OmegaKReducedRangeAxisDiagnostics{};
    return false;
  }

  diagnostics->sample_count = reduced_range_frequencies_hz.size();
  diagnostics->frequency_spacing_hz = spacing_hz;
  diagnostics->effective_bandwidth_hz =
      static_cast<double>(diagnostics->sample_count - 1U) * spacing_hz;
  diagnostics->unambiguous_delay_window_s = 1.0 / spacing_hz;
  diagnostics->relative_delay_spacing_s =
      1.0 / (static_cast<double>(diagnostics->sample_count) * spacing_hz);
  diagnostics->relative_range_spacing_m =
      0.5 * kLightSpeed * diagnostics->relative_delay_spacing_s;
  diagnostics->relative_delays_s.reserve(diagnostics->sample_count);
  for (std::size_t index = 0U; index < diagnostics->sample_count; ++index) {
    diagnostics->relative_delays_s.push_back(
        static_cast<double>(index) * diagnostics->relative_delay_spacing_s);
  }
  diagnostics->valid = true;
  return true;
}

}  // namespace imaging
}  // namespace sar
