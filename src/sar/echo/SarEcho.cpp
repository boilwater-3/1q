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
  return config.sample_rate_hz > 0.0 && std::isfinite(config.sample_rate_hz) &&
         config.carrier_frequency_hz > 0.0 && std::isfinite(config.carrier_frequency_hz) &&
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

// ────────────────────────────────────────────────────────────
// 频域分数延迟(公开)
// ────────────────────────────────────────────────────────────

bool ApplyFractionalDelay(const signal::ComplexVector& input, double fractional_delay,
                          signal::ComplexVector* output) {
  if (output == nullptr || input.empty() || fractional_delay == 0.0) {
    return false;
  }

  const std::size_t padded_size = input.size() + 1U;
  signal::ComplexVector padded(padded_size, signal::ComplexSample(0.0, 0.0));
  std::copy(input.begin(), input.end(), padded.begin());

  signal::ComplexVector spectrum;
  if (!signal::Fft1D(padded, false, &spectrum)) {
    return false;
  }

  const double N = static_cast<double>(spectrum.size());
  for (std::size_t k = 0U; k < spectrum.size(); ++k) {
    const double phase = -2.0 * kPi * static_cast<double>(k) * fractional_delay / N;
    spectrum[k] *= signal::ComplexSample(std::cos(phase), std::sin(phase));
  }

  signal::ComplexVector shifted;
  if (!signal::Fft1D(spectrum, true, &shifted)) {
    return false;
  }

  output->assign(shifted.begin(),
                 shifted.begin() + static_cast<std::ptrdiff_t>(input.size()));
  return true;
}

// ────────────────────────────────────────────────────────────
// 接收机噪声
// ────────────────────────────────────────────────────────────

bool AddNoise(const NoiseSpec& spec, signal::ComplexVector* samples) {
  if (samples == nullptr || samples->empty() || !std::isfinite(spec.signal_to_noise_ratio_db)) {
    return false;
  }

  // 估算信号功率(平均幅度平方)
  double signal_power = 0.0;
  for (const auto& v : *samples) {
    signal_power += std::norm(v);
  }
  signal_power /= static_cast<double>(samples->size());

  const double snr_linear = std::pow(10.0, spec.signal_to_noise_ratio_db / 10.0);
  double noise_power = signal_power / std::max(snr_linear, 1.0e-30);

  // 实虚部独立, 每部方差 = noise_power / 2
  const double sigma = std::sqrt(std::max(noise_power * 0.5, 0.0));

  geometry::DeterministicGaussianSampler gaussian(spec.random_seed);
  for (auto& v : *samples) {
    const double nr = gaussian.Sample() * sigma;
    const double ni = gaussian.Sample() * sigma;
    v = signal::ComplexSample(v.real() + nr, v.imag() + ni);
  }
  return true;
}

// ────────────────────────────────────────────────────────────
// 杂波
// ────────────────────────────────────────────────────────────

double GammaClutterRcs(const ClutterModel& model) {
  return model.gamma_constant * std::sin(std::max(model.incidence_angle_rad, 1.0e-6)) *
         model.resolution_cell_area_m2;
}

double SeaClutterRcs(const ClutterModel& model) {
  // GIT 经验模型简化版: σ⁰ 随海况、风速、入射角变化。
  // σ⁰(dB) ≈ -64 + 6·(sea_state - 1) + 10·log₁₀(wind_speed/5) + (incidence_angle_deg - 30)/2
  const double incidence_deg = model.incidence_angle_rad * 180.0 / kPi;
  const double sea_factor = 6.0 * (model.sea_state - 1.0);
  const double wind_factor = 10.0 * std::log10(std::max(model.wind_speed_mps, 0.1) / 5.0);
  const double incidence_factor = (incidence_deg - 30.0) * 0.5;
  const double sigma0_db = -64.0 + sea_factor + wind_factor + incidence_factor;
  const double sigma0_linear = std::pow(10.0, sigma0_db / 10.0);
  return sigma0_linear * model.resolution_cell_area_m2;
}

// ────────────────────────────────────────────────────────────
// 面目标场景
// ────────────────────────────────────────────────────────────

bool GenerateClutterScene(const RawEchoConfig& config,
                          const geometry::PlatformPulseState& platform,
                          const SceneDescription& scene,
                          const signal::ComplexVector& transmit_waveform,
                          RawEchoResult* result) {
  if (result == nullptr || !IsValid(config, transmit_waveform)) {
    return false;
  }

  // 1) 点目标回波
  if (!GeneratePointTargetRawEcho(config, platform, scene.point_targets,
                                   transmit_waveform, result)) {
    return false;
  }

  // 2) 杂波网格
  const double spacing = scene.clutter_grid_spacing_m;
  if (spacing <= 0.0 || scene.scene_extent_x_m <= 0.0 || scene.scene_extent_y_m <= 0.0) {
    return true;  // 无杂波网格, 跳过
  }

  const double half_x = scene.scene_extent_x_m * 0.5;
  const double half_y = scene.scene_extent_y_m * 0.5;
  const double cell_area = spacing * spacing;

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;

  for (double cx = scene.scene_center.x_m - half_x; cx < scene.scene_center.x_m + half_x;
       cx += spacing) {
    for (double cy = scene.scene_center.y_m - half_y; cy < scene.scene_center.y_m + half_y;
         cy += spacing) {
      geometry::LocalPoint cell_pos;
      cell_pos.x_m = cx;
      cell_pos.y_m = cy;
      cell_pos.z_m = scene.scene_center.z_m;

      const double slant_range_m = geometry::Distance(platform.position_m, cell_pos);
      if (slant_range_m <= 0.0) {
        continue;
      }

      // 局部入射角(简化:地面水平, cosθ_inc ≈ platform.z / R)
      const double cos_inc = std::max(std::abs(platform.position_m.z_m - cell_pos.z_m), 1.0) /
                             std::max(slant_range_m, 1.0e-6);
      const double inc_angle_rad = std::acos(std::min(cos_inc, 1.0));

      ClutterModel cell_model = scene.clutter;
      cell_model.incidence_angle_rad = inc_angle_rad;
      cell_model.resolution_cell_area_m2 = cell_area;

      const double rcs =
          cell_model.type == ClutterType::kSea ? SeaClutterRcs(cell_model) : GammaClutterRcs(cell_model);

      if (!std::isfinite(rcs) || rcs <= 0.0) {
        continue;
      }

      const double two_way_delay_s = 2.0 * slant_range_m / kSpeedOfLightMps;
      const double delay_samples = two_way_delay_s * config.sample_rate_hz;
      const std::size_t delay_sample_index =
          static_cast<std::size_t>(std::llround(delay_samples));
      const double fractional_delay = delay_samples - static_cast<double>(delay_sample_index);

      const double amplitude = std::sqrt(rcs) / (slant_range_m * slant_range_m);
      const double phase = -4.0 * kPi * slant_range_m / wavelength_m;
      const signal::ComplexSample propagation(amplitude * std::cos(phase),
                                               amplitude * std::sin(phase));

      if (delay_sample_index >= config.range_sample_count) {
        result->has_clipping = true;
        continue;
      }

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
        result->has_clipping = true;
      }
    }
  }

  return true;
}

}  // namespace echo
}  // namespace sar
