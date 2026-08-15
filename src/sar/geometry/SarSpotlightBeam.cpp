// MSVC 的 <cmath> 默认不定义 M_PI，需在第一个 <cmath> include 前定义 _USE_MATH_DEFINES。
// 必须放在本文件第一个 #include 之前（SarSpotlightBeam.h 链会引入 <cmath>）。
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "sar/geometry/SarSpotlightBeam.h"

#include <cmath>

#include "sar/signal/SarAngleWrap.h"

namespace sar {
namespace geometry {

bool GenerateSpotlightBeamTrack(const SpotlightBeamTrackConfig& config,
                                const std::vector<PlatformPulseState>& platform_pulses,
                                std::vector<SpotlightBeamState>* beam_states) {
  if (beam_states == nullptr) {
    return false;
  }
  beam_states->clear();
  if (platform_pulses.empty()) {
    return false;
  }
  if (!config.pulse_times_s.empty() && config.pulse_times_s.size() != platform_pulses.size()) {
    return false;
  }
  beam_states->reserve(platform_pulses.size());
  for (std::size_t i = 0U; i < platform_pulses.size(); ++i) {
    SpotlightBeamState state;
    state.time_s = !config.pulse_times_s.empty() ? config.pulse_times_s[i]
                                                  : platform_pulses[i].time_s;
    // boresight_azimuth(t) = atan2(scene_x − platform_x, scene_y − platform_y)
    const double dx = config.scene_center_m.x_m - platform_pulses[i].position_m.x_m;
    const double dy = config.scene_center_m.y_m - platform_pulses[i].position_m.y_m;
    state.boresight_azimuth_rad = std::atan2(dx, dy);
    if (!std::isfinite(state.boresight_azimuth_rad)) {
      beam_states->clear();
      return false;
    }
    beam_states->push_back(state);
  }
  return true;
}

double SpotlightSyntheticApertureTime(const std::vector<SpotlightBeamState>& beam_states,
                                      double slant_range_m, double platform_velocity_mps) {
  if (beam_states.size() < 2U || slant_range_m <= 0.0 || platform_velocity_mps <= 0.0) {
    return 0.0;
  }
  // 波束跟踪角 = 首尾指向角差(取绝对值,处理跨越 ±π)。
  double total_angle = beam_states.back().boresight_azimuth_rad -
                       beam_states.front().boresight_azimuth_rad;
  // 归一化到 [-π, π] 区间以处理跨越（常数时间，contract 规则 5）。
  total_angle = signal::WrapPhase(total_angle);
  const double abs_angle = std::fabs(total_angle);
  // T_synth = θ_synth · R_center / v
  return abs_angle * slant_range_m / platform_velocity_mps;
}

bool GenerateSpotlightTrack(const SpotlightTrackConfig& config,
                            std::vector<PlatformPulseState>* platform_pulses,
                            std::vector<SpotlightBeamState>* beam_states) {
  if (platform_pulses == nullptr || beam_states == nullptr) {
    return false;
  }
  platform_pulses->clear();
  beam_states->clear();
  if (!GenerateStraightStripmapTrack(config.platform_track, platform_pulses)) {
    return false;
  }
  SpotlightBeamTrackConfig beam_config;
  beam_config.scene_center_m = config.scene_center_m;
  if (!GenerateSpotlightBeamTrack(beam_config, *platform_pulses, beam_states)) {
    return false;
  }
  return platform_pulses->size() == beam_states->size();
}

}  // namespace geometry
}  // namespace sar
