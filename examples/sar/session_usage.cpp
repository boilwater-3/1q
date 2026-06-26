#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <vector>

#include "1q/sar/session/SarSession.h"
#include "1q/sar/sar.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kTargetLengthM = 2.0;
constexpr double kTargetWidthM = 3.0;
constexpr double kTargetRcsM2 = 1.0;
constexpr double kPlatformAltitudeM = 10000.0;
constexpr double kTargetSlantRangeM = 100000.0;
constexpr double kTargetGroundRangeM =
    99498.7437106620;  // sqrt(100 km^2 - 10 km^2)
constexpr double kSampleRateHz = 1.0e6;

sar::config::SarSessionConfig MakeConfig() {
  sar::config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 9.6e9;
  config.hardware.bandwidth_hz = 0.5e6;
  config.hardware.pulse_width_s = 20.0e-6;
  config.hardware.pulse_repetition_frequency_hz = 100.0;
  config.hardware.sample_rate_hz = kSampleRateHz;

  config.mission.nominal_slant_range_m = kTargetSlantRangeM;
  config.mission.platform_speed_mps = 180.0;
  config.mission.range_sample_count = 1024U;
  config.mission.azimuth_pulse_count = 33U;

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
  input.platform.altitude_m = kPlatformAltitudeM;
  input.platform.velocity_east_mps = 180.0;

  sar::session::SarPointTarget target;
  target.latitude_deg = kTargetGroundRangeM / kEarthRadiusM * 180.0 / kPi;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 10.0 * std::log10(kTargetRcsM2);
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
      sar::session::SarSession::Create(MakeConfig());
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
  const double observed_slant_range_m =
      static_cast<double>(peak_col) * kSpeedOfLightMps / (2.0 * kSampleRateHz);
  const double range_bin_spacing_m = kSpeedOfLightMps / (2.0 * kSampleRateHz);
  const double slant_range_error_m = std::abs(observed_slant_range_m - kTargetSlantRangeM);
  if (peak_magnitude <= 0.0 || slant_range_error_m > range_bin_spacing_m) {
    std::cerr << "The 100 km, 1 m^2 RCS target was not focused within one range bin.\n";
    return 3;
  }

  std::cout << "SAR processing succeeded\n"
            << "  target model: point target approximation, length=" << kTargetLengthM
            << " m, width=" << kTargetWidthM << " m, RCS=" << kTargetRcsM2 << " m^2\n"
            << "  target altitude: 0 m, platform altitude: " << kPlatformAltitudeM << " m\n"
            << "  expected ground/slant range: " << kTargetGroundRangeM << " / "
            << kTargetSlantRangeM << " m\n"
            << "  focused image: " << image.row_count << " x " << image.column_count << '\n'
            << "  peak pixel: row=" << peak_row << ", col=" << peak_col
            << ", magnitude=" << peak_magnitude
            << ", observed slant range=" << observed_slant_range_m
            << " m, range error=" << slant_range_error_m << " m\n"
            << "  raw echo: " << result.output_frame.has_raw_echo << '\n'
            << "  range compressed: " << result.output_frame.has_range_compressed_echo << '\n'
            << "  L1 RDA image: " << result.output_frame.has_l1_image << '\n'
            << "  diagnostics: " << result.diagnostics.size() << '\n';

  for (const sar::session::SarDiagnosticIssue& issue : result.diagnostics) {
    std::cout << '[' << SeverityName(issue.severity) << "] " << issue.code << ": "
              << issue.message << '\n';
  }

  // Export focused image as raw binary for external visualization.
  {
    const std::string output_path = "/tmp/sar_focused_image.raw";
    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
      std::cerr << "Failed to open " << output_path << " for writing.\n";
      return 4;
    }
    const std::uint32_t rows = image.row_count;
    const std::uint32_t cols = image.column_count;
    const std::uint32_t has_imag = 1U;
    const std::uint32_t peak_r = static_cast<std::uint32_t>(peak_row);
    const std::uint32_t peak_c = static_cast<std::uint32_t>(peak_col);
    output.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
    output.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
    output.write(reinterpret_cast<const char*>(&has_imag), sizeof(has_imag));
    output.write(reinterpret_cast<const char*>(&peak_r), sizeof(peak_r));
    output.write(reinterpret_cast<const char*>(&peak_c), sizeof(peak_c));
    for (std::size_t i = 0U; i < image.real_values.size(); ++i) {
      float real = static_cast<float>(image.real_values[i]);
      float imag = static_cast<float>(image.imaginary_values[i]);
      output.write(reinterpret_cast<const char*>(&real), sizeof(real));
      output.write(reinterpret_cast<const char*>(&imag), sizeof(imag));
    }
    std::cout << "  exported focused image: " << output_path << " ("
              << (output.tellp()) << " bytes)\n";
  }
  return 0;
}
