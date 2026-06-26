// @file SarCycleInputAdapter.cpp
// @brief Implementation of one-step SarCycleInput assembly.

#include "1q/sar/session/SarCycleInputAdapter.h"

#include <utility>
#include <vector>

namespace sar {
namespace session {

bool SarCycleInputAdapter::Build(const SarPlatformState& platform,
                                 const SarPointTargetList& targets,
                                 const config::SarMissionConfig& mission,
                                 double dt_sec,
                                 const std::vector<SarExternalPulseInput>& external_pulses,
                                 SarCycleInput* output,
                                 SarCoordinateStatus* status) {
  if (status != nullptr) {
    *status = SarCoordinateStatus::kOk;
  }

  if (output == nullptr) {
    if (status != nullptr) {
      *status = SarCoordinateStatus::kNullOutput;
    }
    return false;
  }

  // 平台/目标 LLA 直接透传，由 SarSession 内部 ToLocalPoint 转换。
  output->platform = platform;
  output->point_targets = targets;
  output->dt_sec = dt_sec;

  // 无外部脉冲时保持 raw_iq 默认空值，走内部 raw echo 生成路径。
  if (external_pulses.empty()) {
    return true;
  }

  // 构造 scene-center 局部参考系并逐脉冲转换。
  const oneq::coordinate::LocalFrameReference reference = BuildSceneCenterReference(mission);

  std::vector<SarRawIqFrame::PulseState> pulse_states;
  pulse_states.reserve(external_pulses.size());
  for (const SarExternalPulseInput& external_pulse : external_pulses) {
    SarRawIqFrame::PulseState pulse;
    if (!TryMakePulseFromExternalKinematics(external_pulse, reference, &pulse, status)) {
      return false;
    }
    pulse_states.push_back(std::move(pulse));
  }

  output->raw_iq.pulse_states = std::move(pulse_states);
  output->raw_iq.pulse_count = static_cast<std::uint32_t>(external_pulses.size());
  return true;
}

}  // namespace session
}  // namespace sar
