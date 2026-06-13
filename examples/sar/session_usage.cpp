#include <cmath>
#include <cstddef>
#include <iostream>

#include "1q/sar/sar.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;

sar::config::SarSessionConfig MakeConfig() {
  sar::config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;

  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;

  config.policy.enable_raw_echo_generation = true;
  config.policy.enable_range_compression = true;
  config.policy.enable_l1_rda_imaging = true;
  config.policy.enable_diagnostics = true;
  return config;
}

sar::session::SarCycleInput MakeInput() {
  sar::session::SarCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1;

  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;
  input.platform.velocity_east_mps = 2.0;

  sar::session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / kEarthRadiusM * 180.0 / kPi;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

std::size_t FindPeakIndex(const sar::session::SarFocusedImage& image) {
  std::size_t peak_index = 0U;
  double peak_power = -1.0;
  for (std::size_t index = 0U; index < image.real_values.size(); ++index) {
    const double real = image.real_values[index];
    const double imag = image.imaginary_values[index];
    const double power = real * real + imag * imag;
    if (power > peak_power) {
      peak_power = power;
      peak_index = index;
    }
  }
  return peak_index;
}

const char* SeverityName(sar::session::SarDiagnosticSeverity severity) {
  switch (severity) {
    case sar::session::SarDiagnosticSeverity::kInfo:
      return "info";
    case sar::session::SarDiagnosticSeverity::kWarning:
      return "warning";
    case sar::session::SarDiagnosticSeverity::kError:
      return "error";
  }
  return "unknown";
}

}  // namespace

int main() {
  sar::session::SarSession session =
      sar::session::SarSessionFactory::Create(MakeConfig());
  const sar::session::SarCycleResult result = session.StepWithResult(MakeInput());

  if (result.has_error) {
    std::cerr << "SAR processing failed: " << result.abort_reason << '\n';
    for (const sar::session::SarDiagnosticIssue& issue : result.diagnostics) {
      std::cerr << '[' << SeverityName(issue.severity) << "] " << issue.code << ": "
                << issue.message << '\n';
    }
    return 1;
  }

  const sar::session::SarFocusedImage& image = result.focused_image;
  if (image.row_count == 0U || image.column_count == 0U ||
      image.real_values.size() != image.imaginary_values.size()) {
    std::cerr << "SAR processing returned no focused image.\n";
    return 2;
  }

  const std::size_t peak_index = FindPeakIndex(image);
  const std::size_t peak_row = peak_index / image.column_count;
  const std::size_t peak_col = peak_index % image.column_count;
  const double peak_magnitude =
      std::hypot(image.real_values[peak_index], image.imaginary_values[peak_index]);

  std::cout << "SAR processing succeeded\n"
            << "  focused image: " << image.row_count << " x " << image.column_count << '\n'
            << "  peak pixel: row=" << peak_row << ", col=" << peak_col
            << ", magnitude=" << peak_magnitude << '\n'
            << "  raw echo: " << result.output_frame.has_raw_echo << '\n'
            << "  range compressed: " << result.output_frame.has_range_compressed_echo << '\n'
            << "  L1 RDA image: " << result.output_frame.has_l1_image << '\n'
            << "  diagnostics: " << result.diagnostics.size() << '\n';

  for (const sar::session::SarDiagnosticIssue& issue : result.diagnostics) {
    std::cout << '[' << SeverityName(issue.severity) << "] " << issue.code << ": "
              << issue.message << '\n';
  }
  return 0;
}
