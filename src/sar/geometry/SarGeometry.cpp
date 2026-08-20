#include "sar/geometry/SarGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace sar {
namespace geometry {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

bool IsFinite(const LocalPoint& point) {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m) && std::isfinite(point.z_m);
}


}  // namespace

bool GenerateStraightStripmapTrack(const StraightStripmapTrackConfig& config,
                                   std::vector<PlatformPulseState>* pulses) {
  if (pulses == nullptr || !std::isfinite(config.start_time_s) ||
      !std::isfinite(config.velocity_x_mps) || !std::isfinite(config.velocity_y_mps) ||
      !std::isfinite(config.velocity_z_mps) || !std::isfinite(config.roll_deg) ||
      !std::isfinite(config.pitch_deg) || !std::isfinite(config.yaw_deg) ||
      !std::isfinite(config.prf_hz) || config.prf_hz <= 0.0 ||
      config.pulse_count == 0U) {
    return false;
  }

  pulses->clear();
  pulses->reserve(config.pulse_count);
  for (std::uint32_t i = 0U; i < config.pulse_count; ++i) {
    const double time_offset_s = static_cast<double>(i) / config.prf_hz;
    PlatformPulseState pulse;
    pulse.pulse_id = config.first_pulse_id + i;
    pulse.time_s = config.start_time_s + time_offset_s;
    pulse.position_m = config.start_position_m;
    pulse.position_m.x_m += config.velocity_x_mps * time_offset_s;
    pulse.position_m.y_m += config.velocity_y_mps * time_offset_s;
    pulse.position_m.z_m += config.velocity_z_mps * time_offset_s;
    pulse.velocity_x_mps = config.velocity_x_mps;
    pulse.velocity_y_mps = config.velocity_y_mps;
    pulse.velocity_z_mps = config.velocity_z_mps;
    pulse.roll_deg = config.roll_deg;
    pulse.pitch_deg = config.pitch_deg;
    pulse.yaw_deg = config.yaw_deg;
    pulses->push_back(pulse);
  }
  return true;
}

bool GeneratePerturbedStripmapTrack(const PerturbedStripmapTrackConfig& config,
                                    std::vector<PlatformPulseState>* pulses,
                                    TrajectoryErrorDiagnostics* diagnostics) {
  const double ideal_speed_squared =
      config.ideal.velocity_x_mps * config.ideal.velocity_x_mps +
      config.ideal.velocity_y_mps * config.ideal.velocity_y_mps +
      config.ideal.velocity_z_mps * config.ideal.velocity_z_mps;
  if (pulses == nullptr || diagnostics == nullptr || ideal_speed_squared <= 0.0 ||
      config.ideal.prf_hz <= 0.0 || config.ideal.pulse_count == 0U ||
      config.velocity_error_stddev_x_mps < 0.0 || config.velocity_error_stddev_y_mps < 0.0 ||
      config.velocity_error_stddev_z_mps < 0.0) {
    return false;
  }

  std::vector<PlatformPulseState> ideal_pulses;
  if (!GenerateStraightStripmapTrack(config.ideal, &ideal_pulses)) {
    return false;
  }
  if (config.initial_position_error_m.x_m == 0.0 && config.initial_position_error_m.y_m == 0.0 &&
      config.initial_position_error_m.z_m == 0.0 && config.velocity_error_stddev_x_mps == 0.0 &&
      config.velocity_error_stddev_y_mps == 0.0 && config.velocity_error_stddev_z_mps == 0.0) {
    *pulses = ideal_pulses;
    *diagnostics = TrajectoryErrorDiagnostics{};
    return true;
  }

  DeterministicGaussianSampler gaussian{config.random_seed};
  const double dt_s = 1.0 / config.ideal.prf_hz;
  pulses->assign(config.ideal.pulse_count, PlatformPulseState{});
  *diagnostics = TrajectoryErrorDiagnostics{};
  double position_error_squared_sum = 0.0;
  double velocity_error_squared_sum = 0.0;

  for (std::size_t index = 0U; index < pulses->size(); ++index) {
    PlatformPulseState& pulse = (*pulses)[index];
    pulse.pulse_id = ideal_pulses[index].pulse_id;
    pulse.time_s = ideal_pulses[index].time_s;
    pulse.roll_deg = ideal_pulses[index].roll_deg;
    pulse.pitch_deg = ideal_pulses[index].pitch_deg;
    pulse.yaw_deg = ideal_pulses[index].yaw_deg;
    if (index == 0U) {
      pulse.position_m = config.ideal.start_position_m;
      pulse.position_m.x_m += config.initial_position_error_m.x_m;
      pulse.position_m.y_m += config.initial_position_error_m.y_m;
      pulse.position_m.z_m += config.initial_position_error_m.z_m;
    } else {
      const PlatformPulseState& previous = (*pulses)[index - 1U];
      pulse.position_m.x_m = previous.position_m.x_m + previous.velocity_x_mps * dt_s;
      pulse.position_m.y_m = previous.position_m.y_m + previous.velocity_y_mps * dt_s;
      pulse.position_m.z_m = previous.position_m.z_m + previous.velocity_z_mps * dt_s;
    }
    pulse.velocity_x_mps =
        config.ideal.velocity_x_mps + gaussian.Sample() * config.velocity_error_stddev_x_mps;
    pulse.velocity_y_mps =
        config.ideal.velocity_y_mps + gaussian.Sample() * config.velocity_error_stddev_y_mps;
    pulse.velocity_z_mps =
        config.ideal.velocity_z_mps + gaussian.Sample() * config.velocity_error_stddev_z_mps;

    const double position_error_m = Distance(pulse.position_m, ideal_pulses[index].position_m);
    const double velocity_error_x_mps = pulse.velocity_x_mps - config.ideal.velocity_x_mps;
    const double velocity_error_y_mps = pulse.velocity_y_mps - config.ideal.velocity_y_mps;
    const double velocity_error_z_mps = pulse.velocity_z_mps - config.ideal.velocity_z_mps;
    const double velocity_error_mps = std::sqrt(velocity_error_x_mps * velocity_error_x_mps +
                                                velocity_error_y_mps * velocity_error_y_mps +
                                                velocity_error_z_mps * velocity_error_z_mps);
    diagnostics->max_position_error_m =
        std::max(diagnostics->max_position_error_m, position_error_m);
    diagnostics->max_velocity_error_mps =
        std::max(diagnostics->max_velocity_error_mps, velocity_error_mps);
    position_error_squared_sum += position_error_m * position_error_m;
    velocity_error_squared_sum += velocity_error_mps * velocity_error_mps;
  }
  diagnostics->rms_position_error_m =
      std::sqrt(position_error_squared_sum / static_cast<double>(pulses->size()));
  diagnostics->rms_velocity_error_mps =
      std::sqrt(velocity_error_squared_sum / static_cast<double>(pulses->size()));
  return true;
}

bool GenerateWaypointTrack(const WaypointTrackConfig& config,
                           std::vector<PlatformPulseState>* pulses) {
  if (pulses == nullptr || config.waypoints.size() < 2U || config.pulse_times_s.empty() ||
      config.pulse_times_s.size() - 1U >
          std::numeric_limits<std::uint64_t>::max() - config.first_pulse_id) {
    return false;
  }
  for (std::size_t index = 0U; index < config.waypoints.size(); ++index) {
    if (!std::isfinite(config.waypoints[index].time_s) ||
        !IsFinite(config.waypoints[index].position_m) ||
        (index > 0U && config.waypoints[index].time_s <= config.waypoints[index - 1U].time_s)) {
      return false;
    }
  }
  for (std::size_t index = 0U; index < config.pulse_times_s.size(); ++index) {
    if (!std::isfinite(config.pulse_times_s[index]) ||
        config.pulse_times_s[index] < config.waypoints.front().time_s ||
        config.pulse_times_s[index] > config.waypoints.back().time_s ||
        (index > 0U && config.pulse_times_s[index] <= config.pulse_times_s[index - 1U])) {
      return false;
    }
  }

  pulses->clear();
  pulses->reserve(config.pulse_times_s.size());
  for (std::size_t index = 0U; index < config.pulse_times_s.size(); ++index) {
    const double pulse_time_s = config.pulse_times_s[index];
    const std::vector<Waypoint>::const_iterator upper = std::upper_bound(
        config.waypoints.begin(), config.waypoints.end(), pulse_time_s,
        [](double time_s, const Waypoint& waypoint) { return time_s < waypoint.time_s; });
    const std::size_t segment =
        upper == config.waypoints.end()
            ? config.waypoints.size() - 2U
            : static_cast<std::size_t>(upper - config.waypoints.begin() - 1);
    const Waypoint& start = config.waypoints[segment];
    const Waypoint& end = config.waypoints[segment + 1U];
    const double duration_s = end.time_s - start.time_s;
    const double fraction = (pulse_time_s - start.time_s) / duration_s;

    PlatformPulseState pulse;
    pulse.pulse_id = config.first_pulse_id + index;
    pulse.time_s = pulse_time_s;
    pulse.position_m.x_m =
        start.position_m.x_m + fraction * (end.position_m.x_m - start.position_m.x_m);
    pulse.position_m.y_m =
        start.position_m.y_m + fraction * (end.position_m.y_m - start.position_m.y_m);
    pulse.position_m.z_m =
        start.position_m.z_m + fraction * (end.position_m.z_m - start.position_m.z_m);
    pulse.velocity_x_mps = (end.position_m.x_m - start.position_m.x_m) / duration_s;
    pulse.velocity_y_mps = (end.position_m.y_m - start.position_m.y_m) / duration_s;
    pulse.velocity_z_mps = (end.position_m.z_m - start.position_m.z_m) / duration_s;
    pulses->push_back(pulse);
  }
  return true;
}

bool AdvanceFractionalPrf(double dt_s, double prf_hz, FractionalPrfState* state,
                          std::uint32_t* emitted_pulses) {
  if (state == nullptr || emitted_pulses == nullptr || !std::isfinite(dt_s) || dt_s < 0.0 ||
      !std::isfinite(prf_hz) || prf_hz <= 0.0) {
    return false;
  }

  const double total_pulses = state->carry_pulses + dt_s * prf_hz;
  const double whole_pulses = std::floor(total_pulses);
  *emitted_pulses = static_cast<std::uint32_t>(whole_pulses);
  state->carry_pulses = total_pulses - whole_pulses;
  return true;
}

double Distance(const LocalPoint& a, const LocalPoint& b) {
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ────────────────────────────────────────────────────────────
// 数学工具
// ────────────────────────────────────────────────────────────

double Sinc(double x) {
  if (std::abs(x) < 1.0e-12) {
    return 1.0;
  }
  return std::sin(kPi * x) / (kPi * x);
}

DeterministicGaussianSampler::DeterministicGaussianSampler(std::uint32_t seed)
    : generator_(seed), has_spare_(false), spare_(0.0) {}

double DeterministicGaussianSampler::Sample() {
  if (has_spare_) {
    has_spare_ = false;
    return spare_;
  }
  constexpr double kScale = 1.0 / (static_cast<double>(std::mt19937::max()) + 1.0);
  const double u1 = (static_cast<double>(generator_()) + 0.5) * kScale;
  const double u2 = (static_cast<double>(generator_()) + 0.5) * kScale;
  const double r = std::sqrt(-2.0 * std::log(u1));
  spare_ = r * std::sin(2.0 * kPi * u2);
  has_spare_ = true;
  return r * std::cos(2.0 * kPi * u2);
}

// ────────────────────────────────────────────────────────────
// 斜距模型
// ────────────────────────────────────────────────────────────

double ExactSlantRange(const PlatformPulseState& platform, const LocalPoint& target) {
  return Distance(platform.position_m, target);
}

double ClosestSlantRange(const std::vector<PlatformPulseState>& track, const LocalPoint& target) {
  if (track.empty()) {
    return 0.0;
  }
  double closest = Distance(track.front().position_m, target);
  for (std::size_t i = 1U; i < track.size(); ++i) {
    const double d = Distance(track[i].position_m, target);
    if (d < closest) {
      closest = d;
    }
  }
  return closest;
}

double QuadraticApproxRange(const QuadraticRangeApprox& approx, double time_s) {
  const double dt = time_s - approx.broadside_time_s;
  const double r0 = approx.reference_range_m;
  const double v = approx.platform_velocity_mps;
  if (r0 <= 0.0 || v <= 0.0) {
    return r0;
  }
  return r0 + 0.5 * (v * v / r0) * dt * dt;
}

double RangeRate(const PlatformPulseState& platform, const LocalPoint& target) {
  // dR/dt = (platform - target) · v / R, 闭合时 < 0。
  const double dx = platform.position_m.x_m - target.x_m;
  const double dy = platform.position_m.y_m - target.y_m;
  const double dz = platform.position_m.z_m - target.z_m;
  const double range_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (range_m < 1.0e-30) {
    return 0.0;
  }
  return (dx * platform.velocity_x_mps + dy * platform.velocity_y_mps +
          dz * platform.velocity_z_mps) /
         range_m;
}

// ────────────────────────────────────────────────────────────
// 多普勒模型
// ────────────────────────────────────────────────────────────

bool ComputeDopplerParams(const DopplerComputationInput& input, DopplerParams* params) {
  if (params == nullptr || !std::isfinite(input.wavelength_m) || input.wavelength_m <= 0.0 ||
      !std::isfinite(input.platform_velocity_mps) || input.platform_velocity_mps <= 0.0 ||
      !std::isfinite(input.reference_slant_range_m) || input.reference_slant_range_m <= 0.0) {
    return false;
  }

  *params = DopplerParams{};
  params->fd_central_hz =
      2.0 * input.platform_velocity_mps * std::sin(input.squint_angle_rad) / input.wavelength_m;
  const double cos_theta = std::cos(input.squint_angle_rad);
  params->fd_rate_hz_per_s = 2.0 * input.platform_velocity_mps * input.platform_velocity_mps *
                             cos_theta * cos_theta * cos_theta /
                             (input.wavelength_m * input.reference_slant_range_m);

  if (std::isfinite(input.real_aperture_length_m) && input.real_aperture_length_m > 0.0) {
    const double beam_width_rad = input.wavelength_m / input.real_aperture_length_m;
    params->synthetic_aperture_time_s =
        input.reference_slant_range_m * beam_width_rad / input.platform_velocity_mps;
    params->doppler_bandwidth_hz =
        std::abs(params->fd_rate_hz_per_s) * params->synthetic_aperture_time_s;
  }
  return true;
}

double DopplerFrequencyAt(const DopplerParams& params, double slow_time_s) {
  return params.fd_central_hz + params.fd_rate_hz_per_s * slow_time_s;
}

double AzimuthResolution(const DopplerParams& params, double platform_velocity_mps) {
  if (params.doppler_bandwidth_hz <= 0.0 || !std::isfinite(params.doppler_bandwidth_hz)) {
    return 0.0;
  }
  return platform_velocity_mps / params.doppler_bandwidth_hz;
}

double DopplerBinFrequency(std::size_t index, std::size_t count, double prf_hz) {
  if (count == 0U || !std::isfinite(prf_hz)) {
    return 0.0;
  }
  const double raw = static_cast<double>(index) * prf_hz / static_cast<double>(count);
  return (index <= count / 2U) ? raw : (raw - prf_hz);
}

}  // namespace geometry
}  // namespace sar
