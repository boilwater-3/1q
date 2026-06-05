#include "sar/geometry/SarGeometry.h"

#include <cmath>

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
