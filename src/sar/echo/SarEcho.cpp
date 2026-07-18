#include "sar/echo/SarEcho.h"

#include <algorithm>
#include <cmath>

#include "sar/geometry/SarAntenna.h"

namespace sar {
namespace echo {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kFractionalDelayThreshold = 1e-12;

bool IsValid(const RawEchoConfig& config, const signal::ComplexVector& transmit_waveform) {
  return config.sample_rate_hz > 0.0 && std::isfinite(config.sample_rate_hz) &&
         config.carrier_frequency_hz > 0.0 && std::isfinite(config.carrier_frequency_hz) &&
         config.range_sample_count > 0U &&
         config.atmospheric_loss_db_per_km >= 0.0 &&
         std::isfinite(config.atmospheric_loss_db_per_km) && !transmit_waveform.empty();
}

double AtmosphericAmplitudeScale(const RawEchoConfig& config, double slant_range_m) {
  if (!config.enable_atmospheric_attenuation) {
    return 1.0;
  }
  const double two_way_loss_db =
      2.0 * config.atmospheric_loss_db_per_km * slant_range_m / 1000.0;
  return std::pow(10.0, -two_way_loss_db / 20.0);
}

double MeanPower(const signal::ComplexVector& samples) {
  if (samples.empty()) {
    return 0.0;
  }
  double power = 0.0;
  for (const signal::ComplexSample& sample : samples) {
    power += std::norm(sample);
  }
  return power / static_cast<double>(samples.size());
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
  result->point_target_mean_power = 0.0;
  result->distributed_clutter_mean_power = 0.0;
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

    const double amplitude = std::sqrt(target.rcs_m2) /
                             (slant_range_m * slant_range_m) *
                             AtmosphericAmplitudeScale(config, slant_range_m);
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
  result->point_target_mean_power = MeanPower(result->samples);
  return true;
}

bool GeneratePointTargetRawEchoWithAntenna(const RawEchoConfig& config,
                                           const AntennaModulationConfig& antenna_config,
                                           const geometry::PlatformPulseState& platform,
                                           const std::vector<PointTarget>& targets,
                                           const signal::ComplexVector& transmit_waveform,
                                           RawEchoResult* result) {
  // antenna_config.enabled=false 时,严格退化为无调制路径(条带兼容)。
  if (!antenna_config.enabled) {
    return GeneratePointTargetRawEcho(config, platform, targets, transmit_waveform, result);
  }
  if (result == nullptr || !IsValid(config, transmit_waveform)) {
    return false;
  }

  result->samples.assign(config.range_sample_count, signal::ComplexSample(0.0, 0.0));
  result->diagnostics.clear();
  result->point_target_mean_power = 0.0;
  result->distributed_clutter_mean_power = 0.0;
  result->has_clipping = false;

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  const double boresight_rad = antenna_config.beam_state.boresight_azimuth_rad;
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

    // 天线方位调制:off_boresight = 目标方位角 − 波束指向角。
    const double target_dx = target.position_m.x_m - platform.position_m.x_m;
    const double target_dy = target.position_m.y_m - platform.position_m.y_m;
    const double target_azimuth_rad = std::atan2(target_dx, target_dy);
    double off_boresight_rad = target_azimuth_rad - boresight_rad;
    while (off_boresight_rad > kPi) {
      off_boresight_rad -= 2.0 * kPi;
    }
    while (off_boresight_rad < -kPi) {
      off_boresight_rad += 2.0 * kPi;
    }
    // 双程方向图:幅度乘 √(pattern)(场强 vs 功率)。
    const double pattern =
        geometry::AzimuthPattern(antenna_config.antenna, wavelength_m, off_boresight_rad);
    const double amplitude_weight = std::sqrt(std::max(0.0, pattern));

    double amplitude = std::sqrt(target.rcs_m2) / (slant_range_m * slant_range_m);
    amplitude *= amplitude_weight;
    amplitude *= AtmosphericAmplitudeScale(config, slant_range_m);
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
  result->point_target_mean_power = MeanPower(result->samples);
  return true;
}

bool GeneratePointTargetRawEchoWithElevationGate(
    const RawEchoConfig& config, const ElevationGateConfig& gate_config,
    const geometry::PlatformPulseState& platform, const std::vector<PointTarget>& targets,
    const signal::ComplexVector& transmit_waveform, RawEchoResult* result) {
  // gate_config.enabled=false 时,严格退化为无门控路径(条带兼容,单子带退化不变量)。
  if (!gate_config.enabled) {
    return GeneratePointTargetRawEcho(config, platform, targets, transmit_waveform, result);
  }
  if (result == nullptr || !IsValid(config, transmit_waveform)) {
    return false;
  }

  result->samples.assign(config.range_sample_count, signal::ComplexSample(0.0, 0.0));
  result->diagnostics.clear();
  result->point_target_mean_power = 0.0;
  result->distributed_clutter_mean_power = 0.0;
  result->has_clipping = false;

  // 门控有效条件:illuminated 且 near < far(子带窗口有效)。
  const bool gate_active = gate_config.burst_state.illuminated &&
                           gate_config.burst_state.near_range_m > 0.0 &&
                           gate_config.burst_state.far_range_m > gate_config.burst_state.near_range_m;

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  for (std::size_t target_index = 0U; target_index < targets.size(); ++target_index) {
    const PointTarget& target = targets[target_index];
    const double slant_range_m = geometry::Distance(platform.position_m, target.position_m);
    if (slant_range_m <= 0.0 || target.rcs_m2 < 0.0) {
      continue;
    }

    EchoTargetDiagnostic diagnostic;
    diagnostic.target_index = target_index;
    diagnostic.slant_range_m = slant_range_m;
    diagnostic.two_way_delay_s = 2.0 * slant_range_m / kSpeedOfLightMps;
    diagnostic.delay_sample_index =
        static_cast<std::size_t>(std::llround(diagnostic.two_way_delay_s * config.sample_rate_hz));
    diagnostic.fractional_delay_samples =
        diagnostic.two_way_delay_s * config.sample_rate_hz -
        static_cast<double>(diagnostic.delay_sample_index);

    // elevation 距离门控:目标斜距不在子带 [near, far) 窗口内 → 跳过(零贡献)。
    if (gate_active) {
      geometry::ScanSubswath subswath;
      subswath.near_range_m = gate_config.burst_state.near_range_m;
      subswath.far_range_m = gate_config.burst_state.far_range_m;
      if (!geometry::IsInSubswath(subswath, slant_range_m)) {
        // 门控拒绝的目标不入诊断(物理上该脉冲天线根本没看它)。
        continue;
      }
    }
    // gate_active=false(illuminated=false 或窗口无效)→ 该脉冲全程无贡献(天线在别处或无定义)。
    if (!gate_active) {
      continue;
    }

    const std::size_t delay_sample_index = diagnostic.delay_sample_index;
    if (delay_sample_index >= config.range_sample_count) {
      diagnostic.clipped = true;
      diagnostic.clipped_samples = transmit_waveform.size();
      result->has_clipping = true;
      result->diagnostics.push_back(diagnostic);
      continue;
    }

    const double amplitude = std::sqrt(target.rcs_m2) /
                             (slant_range_m * slant_range_m) *
                             AtmosphericAmplitudeScale(config, slant_range_m);
    const double phase = -4.0 * kPi * slant_range_m / wavelength_m;
    const signal::ComplexSample propagation(amplitude * std::cos(phase),
                                            amplitude * std::sin(phase));

    signal::ComplexVector effective_waveform;
    if (std::abs(diagnostic.fractional_delay_samples) > kFractionalDelayThreshold) {
      if (!ApplyFractionalDelay(transmit_waveform, diagnostic.fractional_delay_samples,
                                &effective_waveform)) {
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
  result->point_target_mean_power = MeanPower(result->samples);
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

double ConstantSigma0ClutterRcs(const ClutterModel& model) {
  return model.sigma0_linear * model.resolution_cell_area_m2;
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
  const double cell_area = scene.clutter_cell_area_m2 > 0.0
                               ? scene.clutter_cell_area_m2
                               : spacing * spacing;

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  signal::ComplexVector clutter_samples(config.range_sample_count,
                                        signal::ComplexSample(0.0, 0.0));

  for (double cx = scene.scene_center.x_m - half_x + 0.5 * spacing;
       cx < scene.scene_center.x_m + half_x;
       cx += spacing) {
    for (double cy = scene.scene_center.y_m - half_y + 0.5 * spacing;
         cy < scene.scene_center.y_m + half_y;
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

      double rcs = 0.0;
      switch (cell_model.type) {
        case ClutterType::kGamma:
          rcs = GammaClutterRcs(cell_model);
          break;
        case ClutterType::kSea:
          rcs = SeaClutterRcs(cell_model);
          break;
        case ClutterType::kConstantSigma0:
          rcs = ConstantSigma0ClutterRcs(cell_model);
          break;
      }

      if (!std::isfinite(rcs) || rcs <= 0.0) {
        continue;
      }

      const double two_way_delay_s = 2.0 * slant_range_m / kSpeedOfLightMps;
      const double delay_samples = two_way_delay_s * config.sample_rate_hz;
      const std::size_t delay_sample_index =
          static_cast<std::size_t>(std::llround(delay_samples));
      const double fractional_delay = delay_samples - static_cast<double>(delay_sample_index);

      const double amplitude = std::sqrt(rcs) /
                               (slant_range_m * slant_range_m) *
                               AtmosphericAmplitudeScale(config, slant_range_m);
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
        clutter_samples[delay_sample_index + i] += effective_waveform[i] * propagation;
      }
      if (writable_count < transmit_waveform.size()) {
        result->has_clipping = true;
      }
    }
  }

  result->distributed_clutter_mean_power = MeanPower(clutter_samples);
  for (std::size_t index = 0U; index < result->samples.size(); ++index) {
    result->samples[index] += clutter_samples[index];
  }

  return true;
}

}  // namespace echo
}  // namespace sar
