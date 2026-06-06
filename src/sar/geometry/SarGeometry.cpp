#include "sar/geometry/SarGeometry.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace sar {
namespace geometry {

bool GenerateStraightStripmapTrack(const StraightStripmapTrackConfig& config,
                                   std::vector<PlatformPulseState>* pulses) {
  if (pulses == nullptr || config.prf_hz <= 0.0 || config.pulse_count == 0U) {
    return false;
  }

  pulses->clear();
  pulses->reserve(config.pulse_count);
  for (std::uint32_t i = 0U; i < config.pulse_count; ++i) {
    const double time_s = static_cast<double>(i) / config.prf_hz;
    PlatformPulseState pulse;
    pulse.pulse_id = config.first_pulse_id + i;
    pulse.time_s = time_s;
    pulse.position_m = config.start_position_m;
    pulse.position_m.x_m += config.velocity_x_mps * time_s;
    pulse.velocity_x_mps = config.velocity_x_mps;
    pulses->push_back(pulse);
  }
  return true;
}

bool GeneratePerturbedStripmapTrack(const PerturbedStripmapTrackConfig& config,
                                    std::vector<PlatformPulseState>* pulses,
                                    TrajectoryErrorDiagnostics* diagnostics) {
  if (pulses == nullptr || diagnostics == nullptr || config.ideal.velocity_x_mps <= 0.0 ||
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

  std::mt19937 generator(config.random_seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  const double dt_s = 1.0 / config.ideal.prf_hz;
  pulses->assign(config.ideal.pulse_count, PlatformPulseState{});
  *diagnostics = TrajectoryErrorDiagnostics{};
  double position_error_squared_sum = 0.0;
  double velocity_error_squared_sum = 0.0;

  for (std::size_t index = 0U; index < pulses->size(); ++index) {
    PlatformPulseState& pulse = (*pulses)[index];
    pulse.pulse_id = ideal_pulses[index].pulse_id;
    pulse.time_s = ideal_pulses[index].time_s;
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
        config.ideal.velocity_x_mps + normal(generator) * config.velocity_error_stddev_x_mps;
    pulse.velocity_y_mps = normal(generator) * config.velocity_error_stddev_y_mps;
    pulse.velocity_z_mps = normal(generator) * config.velocity_error_stddev_z_mps;

    const double position_error_m = Distance(pulse.position_m, ideal_pulses[index].position_m);
    const double velocity_error_x_mps = pulse.velocity_x_mps - config.ideal.velocity_x_mps;
    const double velocity_error_mps = std::sqrt(velocity_error_x_mps * velocity_error_x_mps +
                                                pulse.velocity_y_mps * pulse.velocity_y_mps +
                                                pulse.velocity_z_mps * pulse.velocity_z_mps);
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

bool AdvanceFractionalPrf(double dt_s, double prf_hz, FractionalPrfState* state,
                          std::uint32_t* emitted_pulses) {
  if (state == nullptr || emitted_pulses == nullptr || dt_s < 0.0 || prf_hz <= 0.0) {
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

}  // namespace geometry
}  // namespace sar
