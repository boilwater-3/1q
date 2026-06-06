#include "sar/echo/SarEcho.h"

#include <algorithm>
#include <cmath>

namespace sar {
namespace echo {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kFractionalDelayThreshold = 1e-12;

bool IsValid(const RawEchoConfig& config, const signal::ComplexVector& transmit_waveform) {
  return config.sample_rate_hz > 0.0 && config.carrier_frequency_hz > 0.0 &&
         config.range_sample_count > 0U && !transmit_waveform.empty();
}

bool ApplyFractionalDelay(const signal::ComplexVector& input, double fractional_delay,
                           signal::ComplexVector* output) {
  if (output == nullptr || input.empty() || fractional_delay == 0.0) {
    return false;
  }

  // Zero-pad by one sample to prevent circular wrap-around.
  const std::size_t padded_size = input.size() + 1U;
  signal::ComplexVector padded(padded_size, signal::ComplexSample(0.0, 0.0));
  std::copy(input.begin(), input.end(), padded.begin());

  signal::ComplexVector spectrum;
  if (!signal::Fft1D(padded, false, &spectrum)) {
    return false;
  }

  // Phase ramp: Y[k] = X[k] * exp(-j*2*pi*k*d/N) corresponds to time shift d.
  const double N = static_cast<double>(spectrum.size());
  for (std::size_t k = 0U; k < spectrum.size(); ++k) {
    const double phase = -2.0 * kPi * static_cast<double>(k) * fractional_delay / N;
    spectrum[k] *= signal::ComplexSample(std::cos(phase), std::sin(phase));
  }

  signal::ComplexVector shifted;
  if (!signal::Fft1D(spectrum, true, &shifted)) {
    return false;
  }

  // Same length as input; the fractional tail is absorbed by the zero-padded sample.
  output->assign(shifted.begin(),
                 shifted.begin() + static_cast<std::ptrdiff_t>(input.size()));
  return true;
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
    const double delay_samples = two_way_delay_s * config.sample_rate_hz;
    const std::size_t delay_sample_index =
        static_cast<std::size_t>(std::llround(delay_samples));
    const double fractional_delay = delay_samples - static_cast<double>(delay_sample_index);

    const double amplitude = std::sqrt(target.rcs_m2) / (slant_range_m * slant_range_m);
    const double phase = -4.0 * kPi * slant_range_m / wavelength_m;
    const signal::ComplexSample propagation(amplitude * std::cos(phase),
                                            amplitude * std::sin(phase));

    EchoTargetDiagnostic diagnostic;
    diagnostic.target_index = target_index;
    diagnostic.slant_range_m = slant_range_m;
    diagnostic.two_way_delay_s = two_way_delay_s;
    diagnostic.delay_sample_index = delay_sample_index;
    diagnostic.fractional_delay_samples = fractional_delay;

    if (delay_sample_index >= config.range_sample_count) {
      diagnostic.clipped = true;
      diagnostic.clipped_samples = transmit_waveform.size();
      result->has_clipping = true;
      result->diagnostics.push_back(diagnostic);
      continue;
    }

    // Apply sub-sample delay via frequency-domain phase ramp when needed.
    signal::ComplexVector effective_waveform;
    if (std::abs(fractional_delay) > kFractionalDelayThreshold) {
      if (!ApplyFractionalDelay(transmit_waveform, fractional_delay, &effective_waveform)) {
        return false;
      }
    } else {
      effective_waveform = transmit_waveform;
    }

    const std::size_t writable_count =
        std::min(effective_waveform.size(), config.range_sample_count - delay_sample_index);
    for (std::size_t i = 0U; i < writable_count; ++i) {
      result->samples[delay_sample_index + i] += effective_waveform[i] * propagation;
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
