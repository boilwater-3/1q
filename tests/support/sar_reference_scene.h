/**
 * @file sar_reference_scene.h
 * @brief SAR 聚焦算法共用的确定性点目标参考场景。
 */

#ifndef ONEQ_TESTS_SUPPORT_SAR_REFERENCE_SCENE_H_
#define ONEQ_TESTS_SUPPORT_SAR_REFERENCE_SCENE_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarRda.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace test_support {

constexpr double kReferenceSpeedOfLightMps = 299792458.0;

struct ReferencePointScene {
  double sample_rate_hz{100.0e6};
  double carrier_frequency_hz{1.0e9};
  double prf_hz{20.0};
  double platform_velocity_mps{2.0};
  std::size_t range_sample_count{64U};
  std::uint32_t pulse_count{9U};
  signal::LfmWaveform waveform{};
  signal::ComplexVector matched_filter{};
  std::vector<geometry::PlatformPulseState> pulses{};
};

struct ReferenceRawHistoryDiagnostics {
  std::size_t clipped_pulse_count{0U};
  std::size_t clipped_target_count{0U};
  std::size_t clipped_sample_count{0U};
};

struct ReferenceNoiseDiagnostics {
  double signal_energy{0.0};
  double noise_energy{0.0};
  double requested_snr_db{0.0};
  double realized_snr_db{0.0};
  std::uint64_t seed{0U};
};

struct ReferenceClutterGridConfig {
  std::size_t azimuth_count{0U};
  std::size_t range_count{0U};
  double azimuth_start_m{0.0};
  double azimuth_spacing_m{0.0};
  std::size_t range_start_sample{0U};
  std::size_t range_spacing_samples{0U};
  double requested_scr_db{0.0};
  std::uint64_t seed{0U};
};

struct ReferenceClutterDiagnostics {
  double target_energy{0.0};
  double clutter_energy{0.0};
  double requested_scr_db{0.0};
  double realized_scr_db{0.0};
  std::uint64_t seed{0U};
  std::size_t scatterer_count{0U};
  std::size_t grid_rows{0U};
  std::size_t grid_cols{0U};
  ReferenceRawHistoryDiagnostics raw_diagnostics{};
};

inline double NextReferenceUniformOpen(std::uint64_t* state) {
  *state += UINT64_C(0x9e3779b97f4a7c15);
  std::uint64_t value = *state;
  value = (value ^ (value >> 30U)) * UINT64_C(0xbf58476d1ce4e5b9);
  value = (value ^ (value >> 27U)) * UINT64_C(0x94d049bb133111eb);
  value ^= value >> 31U;
  return (static_cast<double>(value >> 11U) + 0.5) / 9007199254740992.0;
}

inline bool AddDeterministicComplexGaussianNoise(const signal::ComplexMatrix& input,
                                                 double requested_snr_db, std::uint64_t seed,
                                                 signal::ComplexMatrix* output,
                                                 ReferenceNoiseDiagnostics* diagnostics) {
  if (output == nullptr || diagnostics == nullptr || input.rows == 0U || input.cols == 0U ||
      input.values.size() != input.rows * input.cols || !std::isfinite(requested_snr_db)) {
    return false;
  }
  double signal_energy = 0.0;
  for (const signal::ComplexSample& sample : input.values) {
    signal_energy += std::norm(sample);
  }
  if (!(signal_energy > 0.0)) {
    return false;
  }

  signal::ComplexVector noise(input.values.size());
  double candidate_noise_energy = 0.0;
  std::uint64_t state = seed;
  const double two_pi = 6.283185307179586476925286766559005768;
  for (signal::ComplexSample& sample : noise) {
    const double radius = std::sqrt(-2.0 * std::log(NextReferenceUniformOpen(&state)));
    const double angle = two_pi * NextReferenceUniformOpen(&state);
    sample = signal::ComplexSample(radius * std::cos(angle), radius * std::sin(angle));
    candidate_noise_energy += std::norm(sample);
  }
  if (!(candidate_noise_energy > 0.0)) {
    return false;
  }

  const double target_noise_energy = signal_energy / std::pow(10.0, requested_snr_db / 10.0);
  const double scale = std::sqrt(target_noise_energy / candidate_noise_energy);
  *output = input;
  double realized_noise_energy = 0.0;
  for (std::size_t index = 0U; index < noise.size(); ++index) {
    const signal::ComplexSample scaled_noise = noise[index] * scale;
    output->values[index] += scaled_noise;
    realized_noise_energy += std::norm(scaled_noise);
  }
  diagnostics->signal_energy = signal_energy;
  diagnostics->noise_energy = realized_noise_energy;
  diagnostics->requested_snr_db = requested_snr_db;
  diagnostics->realized_snr_db = 10.0 * std::log10(signal_energy / realized_noise_energy);
  diagnostics->seed = seed;
  return true;
}

inline echo::PointTarget MakeReferenceTargetAtDelay(std::size_t delay_sample, double sample_rate_hz,
                                                    double desired_amplitude) {
  const double range_m =
      static_cast<double>(delay_sample) * kReferenceSpeedOfLightMps / (2.0 * sample_rate_hz);
  echo::PointTarget target;
  target.position_m = {0.0, range_m, 0.0};
  target.rcs_m2 = std::pow(desired_amplitude * range_m * range_m, 2.0);
  return target;
}

inline echo::PointTarget MakeReferenceTargetAtPosition(double azimuth_m, std::size_t delay_sample,
                                                       double sample_rate_hz,
                                                       double desired_amplitude) {
  echo::PointTarget target =
      MakeReferenceTargetAtDelay(delay_sample, sample_rate_hz, desired_amplitude);
  target.position_m.x_m = azimuth_m;
  const double range_m = target.position_m.y_m;
  const double slant_range_m = std::sqrt(azimuth_m * azimuth_m + range_m * range_m);
  target.rcs_m2 = std::pow(desired_amplitude * slant_range_m * slant_range_m, 2.0);
  return target;
}

inline bool BuildReferencePointScene(ReferencePointScene* scene) {
  if (scene == nullptr) {
    return false;
  }
  signal::LfmWaveformConfig waveform_config;
  waveform_config.bandwidth_hz = 25.0e6;
  waveform_config.time_bandwidth_product = 4.0;
  waveform_config.sample_rate_hz = scene->sample_rate_hz;
  if (!signal::GenerateLfmWaveform(waveform_config, &scene->waveform) ||
      !signal::BuildMatchedFilter(scene->waveform.samples, &scene->matched_filter)) {
    return false;
  }

  geometry::StraightStripmapTrackConfig track_config;
  track_config.velocity_x_mps = scene->platform_velocity_mps;
  track_config.prf_hz = scene->prf_hz;
  track_config.pulse_count = scene->pulse_count;
  track_config.start_position_m.x_m = -0.5 * static_cast<double>(scene->pulse_count - 1U) *
                                      scene->platform_velocity_mps / scene->prf_hz;
  return geometry::GenerateStraightStripmapTrack(track_config, &scene->pulses);
}

inline bool BuildReferenceRawHistory(const ReferencePointScene& scene,
                                     const std::vector<echo::PointTarget>& targets,
                                     signal::ComplexMatrix* history,
                                     ReferenceRawHistoryDiagnostics* diagnostics) {
  if (history == nullptr || scene.pulses.empty() || scene.waveform.samples.empty()) {
    return false;
  }
  if (diagnostics != nullptr) {
    *diagnostics = ReferenceRawHistoryDiagnostics{};
  }
  history->rows = scene.pulses.size();
  history->cols = scene.range_sample_count;
  history->values.assign(history->rows * history->cols, signal::ComplexSample(0.0, 0.0));

  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = scene.sample_rate_hz;
  echo_config.carrier_frequency_hz = scene.carrier_frequency_hz;
  echo_config.range_sample_count = scene.range_sample_count;
  for (std::size_t row = 0U; row < scene.pulses.size(); ++row) {
    echo::RawEchoResult echo;
    if (!echo::GeneratePointTargetRawEcho(echo_config, scene.pulses[row], targets,
                                          scene.waveform.samples, &echo)) {
      return false;
    }
    if (diagnostics != nullptr && echo.has_clipping) {
      ++diagnostics->clipped_pulse_count;
      for (const echo::EchoTargetDiagnostic& target_diagnostic : echo.diagnostics) {
        if (target_diagnostic.clipped) {
          ++diagnostics->clipped_target_count;
          diagnostics->clipped_sample_count += target_diagnostic.clipped_samples;
        }
      }
    }
    for (std::size_t col = 0U; col < scene.range_sample_count; ++col) {
      (*history)(row, col) = echo.samples[col];
    }
  }
  return true;
}

inline bool BuildReferenceRawHistory(const ReferencePointScene& scene,
                                     const std::vector<echo::PointTarget>& targets,
                                     signal::ComplexMatrix* history) {
  return BuildReferenceRawHistory(scene, targets, history, nullptr);
}

inline bool BuildDeterministicDistributedClutter(
    const ReferencePointScene& scene, const signal::ComplexMatrix& target_history,
    const ReferenceClutterGridConfig& config, signal::ComplexMatrix* mixed_history,
    signal::ComplexMatrix* clutter_history, ReferenceClutterDiagnostics* diagnostics) {
  if (mixed_history == nullptr || clutter_history == nullptr || diagnostics == nullptr ||
      target_history.rows == 0U || target_history.cols == 0U ||
      target_history.values.size() != target_history.rows * target_history.cols ||
      target_history.rows != scene.pulses.size() || target_history.cols != scene.range_sample_count ||
      config.azimuth_count == 0U || config.range_count == 0U ||
      config.azimuth_spacing_m <= 0.0 || config.range_spacing_samples == 0U ||
      !std::isfinite(config.requested_scr_db)) {
    return false;
  }

  double target_energy = 0.0;
  for (const signal::ComplexSample& sample : target_history.values) {
    target_energy += std::norm(sample);
  }
  if (!(target_energy > 0.0)) {
    return false;
  }

  const std::size_t scatterer_count = config.azimuth_count * config.range_count;
  if (scatterer_count < 2U) {
    return false;
  }
  signal::ComplexVector coefficients(scatterer_count);
  signal::ComplexSample coefficient_mean(0.0, 0.0);
  std::uint64_t state = config.seed;
  for (signal::ComplexSample& coefficient : coefficients) {
    coefficient = signal::ComplexSample(2.0 * NextReferenceUniformOpen(&state) - 1.0,
                                        2.0 * NextReferenceUniformOpen(&state) - 1.0);
    coefficient_mean += coefficient;
  }
  coefficient_mean /= static_cast<double>(scatterer_count);
  for (signal::ComplexSample& coefficient : coefficients) {
    coefficient -= coefficient_mean;
  }

  clutter_history->rows = target_history.rows;
  clutter_history->cols = target_history.cols;
  clutter_history->values.assign(target_history.values.size(), signal::ComplexSample(0.0, 0.0));
  ReferenceRawHistoryDiagnostics aggregate_raw_diagnostics;
  std::size_t scatterer_index = 0U;
  for (std::size_t azimuth_index = 0U; azimuth_index < config.azimuth_count; ++azimuth_index) {
    const double azimuth_m =
        config.azimuth_start_m + static_cast<double>(azimuth_index) * config.azimuth_spacing_m;
    for (std::size_t range_index = 0U; range_index < config.range_count;
         ++range_index, ++scatterer_index) {
      const std::size_t delay_sample =
          config.range_start_sample + range_index * config.range_spacing_samples;
      signal::ComplexMatrix scatterer_history;
      ReferenceRawHistoryDiagnostics scatterer_diagnostics;
      if (!BuildReferenceRawHistory(
              scene, {MakeReferenceTargetAtPosition(azimuth_m, delay_sample, scene.sample_rate_hz,
                                                    1.0)},
              &scatterer_history, &scatterer_diagnostics)) {
        return false;
      }
      aggregate_raw_diagnostics.clipped_pulse_count += scatterer_diagnostics.clipped_pulse_count;
      aggregate_raw_diagnostics.clipped_target_count += scatterer_diagnostics.clipped_target_count;
      aggregate_raw_diagnostics.clipped_sample_count += scatterer_diagnostics.clipped_sample_count;
      for (std::size_t sample_index = 0U; sample_index < scatterer_history.values.size();
           ++sample_index) {
        clutter_history->values[sample_index] +=
            scatterer_history.values[sample_index] * coefficients[scatterer_index];
      }
    }
  }

  double candidate_clutter_energy = 0.0;
  for (const signal::ComplexSample& sample : clutter_history->values) {
    candidate_clutter_energy += std::norm(sample);
  }
  if (!(candidate_clutter_energy > 0.0)) {
    return false;
  }
  const double target_clutter_energy =
      target_energy / std::pow(10.0, config.requested_scr_db / 10.0);
  const double scale = std::sqrt(target_clutter_energy / candidate_clutter_energy);
  double realized_clutter_energy = 0.0;
  for (signal::ComplexSample& sample : clutter_history->values) {
    sample *= scale;
    realized_clutter_energy += std::norm(sample);
  }

  *mixed_history = target_history;
  for (std::size_t index = 0U; index < mixed_history->values.size(); ++index) {
    mixed_history->values[index] += clutter_history->values[index];
  }
  diagnostics->target_energy = target_energy;
  diagnostics->clutter_energy = realized_clutter_energy;
  diagnostics->requested_scr_db = config.requested_scr_db;
  diagnostics->realized_scr_db = 10.0 * std::log10(target_energy / realized_clutter_energy);
  diagnostics->seed = config.seed;
  diagnostics->scatterer_count = scatterer_count;
  diagnostics->grid_rows = config.azimuth_count;
  diagnostics->grid_cols = config.range_count;
  diagnostics->raw_diagnostics = aggregate_raw_diagnostics;
  return true;
}

inline imaging::RdaConfig MakeReferenceRdaConfig(const ReferencePointScene& scene,
                                                 std::size_t reference_delay) {
  imaging::RdaConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.prf_hz = scene.prf_hz;
  config.platform_velocity_mps = scene.platform_velocity_mps;
  config.reference_range_m = static_cast<double>(reference_delay) * kReferenceSpeedOfLightMps /
                             (2.0 * scene.sample_rate_hz);
  config.rcmc_interpolation = imaging::RcmcInterpolation::kLinear;
  return config;
}

}  // namespace test_support
}  // namespace sar

#endif  // ONEQ_TESTS_SUPPORT_SAR_REFERENCE_SCENE_H_
