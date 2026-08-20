#include "sar/geometry/SarScanBurst.h"

#include <cmath>

namespace sar {
namespace geometry {

namespace {

bool IsValidSubswath(const ScanSubswath& subswath) {
  return std::isfinite(subswath.near_range_m) && subswath.near_range_m > 0.0 &&
         std::isfinite(subswath.far_range_m) &&
         subswath.far_range_m > subswath.near_range_m;
}

}  // namespace

double SubswathCenterRange(const ScanSubswath& subswath) {
  if (!IsValidSubswath(subswath)) {
    return 0.0;
  }
  return 0.5 * (subswath.near_range_m + subswath.far_range_m);
}

bool IsInSubswath(const ScanSubswath& subswath, double slant_range_m) {
  if (!IsValidSubswath(subswath) || !std::isfinite(slant_range_m) || slant_range_m <= 0.0) {
    return false;
  }
  // 半开区间 [near, far):避免边界目标重复计入相邻子带。
  return slant_range_m >= subswath.near_range_m && slant_range_m < subswath.far_range_m;
}

bool GenerateScanBurstSchedule(const ScanBurstScheduleConfig& config,
                               std::vector<ScanBurstState>* burst_states) {
  if (burst_states == nullptr) {
    return false;
  }
  burst_states->clear();
  if (config.subswaths.empty() || !std::isfinite(config.dwell_time_s) ||
      config.dwell_time_s <= 0.0 || config.pulse_times_s.empty()) {
    return false;
  }
  // 校验全部子带有效。
  for (const ScanSubswath& subswath : config.subswaths) {
    if (!IsValidSubswath(subswath)) {
      return false;
    }
  }

  const std::size_t subswath_count = config.subswaths.size();
  const double cycle_time_s = config.dwell_time_s * static_cast<double>(subswath_count);

  burst_states->reserve(config.pulse_times_s.size());
  for (std::size_t i = 0U; i < config.pulse_times_s.size(); ++i) {
    const double t = config.pulse_times_s[i];
    if (!std::isfinite(t) || t < 0.0) {
      burst_states->clear();
      return false;
    }
    // 调度律(契约 §3.2):周期性子带轮转。
    const double cycle_offset = std::fmod(t, cycle_time_s);
    const std::size_t slot_index = static_cast<std::size_t>(cycle_offset / config.dwell_time_s);
    const std::size_t subswath_index =
        slot_index < subswath_count ? slot_index : subswath_count - 1U;
    const ScanSubswath& active = config.subswaths[subswath_index];

    ScanBurstState state;
    state.time_s = t;
    state.subswath_index = static_cast<std::uint32_t>(subswath_index);
    state.near_range_m = active.near_range_m;
    state.far_range_m = active.far_range_m;
    state.illuminated = true;  // 每脉冲必落在某子带的驻留窗口内
    burst_states->push_back(state);
  }
  return true;
}

bool GenerateScanSarTrack(const ScanSarTrackConfig& config,
                          std::vector<PlatformPulseState>* platform_pulses,
                          std::vector<ScanBurstState>* burst_states) {
  if (platform_pulses == nullptr || burst_states == nullptr) {
    return false;
  }
  platform_pulses->clear();
  burst_states->clear();
  if (!GenerateStraightStripmapTrack(config.platform_track, platform_pulses)) {
    return false;
  }

  ScanBurstScheduleConfig schedule_config;
  schedule_config.subswaths = config.subswaths;
  schedule_config.dwell_time_s = config.dwell_time_s;
  schedule_config.pulse_times_s.reserve(platform_pulses->size());
  for (const PlatformPulseState& pulse : *platform_pulses) {
    schedule_config.pulse_times_s.push_back(pulse.time_s);
  }
  if (!GenerateScanBurstSchedule(schedule_config, burst_states)) {
    return false;
  }
  return platform_pulses->size() == burst_states->size();
}

}  // namespace geometry
}  // namespace sar
