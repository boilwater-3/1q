#include "sar/echo/SarEcho.h"

#include <algorithm>
#include <cmath>

namespace sar {
namespace echo {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;

bool IsValid(const RawEchoConfig& config, const signal::ComplexVector& transmit_waveform) {
  return config.sample_rate_hz > 0.0 && config.carrier_frequency_hz > 0.0 &&
         config.range_sample_count > 0U && !transmit_waveform.empty();
}

}  // namespace

bool GeneratePointTargetRawEcho(const RawEchoConfig& config,
                                const geometry::PlatformPulseState& platform,
                                const std::vector<PointTarget>& targets,
                                const signal::ComplexVector& transmit_waveform,
                                RawEchoResult* result) {
  if (result == nullptr || !IsValid(config, transmit_waveform)) {
    return false;
  }

  result->samples.assign(config.range_sample_count, signal::ComplexSample(0.0, 0.0));
  result->diagnostics.clear();
  result->has_clipping = false;

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  for (std::size_t target_index = 0U; target_index < targets.size(); ++target_index) {
    const PointTarget& target = targets[target_index];
    const double slant_range_m = geometry::Distance(platform.position_m, target.position_m);
    if (slant_range_m <= 0.0 || target.rcs_m2 < 0.0) {
      continue;
    }

    const double two_way_delay_s = 2.0 * slant_range_m / kSpeedOfLightMps;
    const std::size_t delay_sample_index =
        static_cast<std::size_t>(std::llround(two_way_delay_s * config.sample_rate_hz));
    const double amplitude = std::sqrt(target.rcs_m2) / (slant_range_m * slant_range_m);
    const double phase = -4.0 * kPi * slant_range_m / wavelength_m;
    const signal::ComplexSample propagation(amplitude * std::cos(phase),
                                            amplitude * std::sin(phase));

    EchoTargetDiagnostic diagnostic;
    diagnostic.target_index = target_index;
    diagnostic.slant_range_m = slant_range_m;
    diagnostic.two_way_delay_s = two_way_delay_s;
    diagnostic.delay_sample_index = delay_sample_index;

    if (delay_sample_index >= config.range_sample_count) {
      diagnostic.clipped = true;
      diagnostic.clipped_samples = transmit_waveform.size();
      result->has_clipping = true;
      result->diagnostics.push_back(diagnostic);
      continue;
    }

    const std::size_t writable_count =
        std::min(transmit_waveform.size(), config.range_sample_count - delay_sample_index);
    for (std::size_t i = 0U; i < writable_count; ++i) {
      result->samples[delay_sample_index + i] += transmit_waveform[i] * propagation;
    }

    if (writable_count < transmit_waveform.size()) {
      diagnostic.clipped = true;
      diagnostic.clipped_samples = transmit_waveform.size() - writable_count;
      result->has_clipping = true;
    }
    result->diagnostics.push_back(diagnostic);
  }
  return true;
}

}  // namespace echo
}  // namespace sar
