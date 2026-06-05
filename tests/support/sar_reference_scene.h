/**
 * @file sar_reference_scene.h
 * @brief SAR 聚焦算法共用的确定性点目标参考场景。
 */

#ifndef ONEQ_TESTS_SUPPORT_SAR_REFERENCE_SCENE_H_
#define ONEQ_TESTS_SUPPORT_SAR_REFERENCE_SCENE_H_

#include <cmath>
#include <cstdint>
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

inline echo::PointTarget MakeReferenceTargetAtDelay(std::size_t delay_sample, double sample_rate_hz,
                                                    double desired_amplitude) {
  const double range_m =
      static_cast<double>(delay_sample) * kReferenceSpeedOfLightMps / (2.0 * sample_rate_hz);
  echo::PointTarget target;
  target.position_m = {0.0, range_m, 0.0};
  target.rcs_m2 = std::pow(desired_amplitude * range_m * range_m, 2.0);
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
                                     signal::ComplexMatrix* history) {
  if (history == nullptr || scene.pulses.empty() || scene.waveform.samples.empty()) {
    return false;
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
    for (std::size_t col = 0U; col < scene.range_sample_count; ++col) {
      (*history)(row, col) = echo.samples[col];
    }
  }
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
