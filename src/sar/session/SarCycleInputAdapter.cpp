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
                                 float dt_sec,
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

  // 仅写入伴随轨迹，不提供 IQ 样本。当前 SAR 内部回波路径会从 config + platform
  // 重算轨迹，故此处的外部轨迹不进入成像——该写入为未来“外部轨迹覆盖”入口预留，
  // 且因 HasExternalRawIq 以 IQ 样本为充要条件，不会把本输出误判为外部 IQ。
  // 详见 SarCycleInputAdapter.h 的契约说明。
  output->raw_iq.pulse_states = std::move(pulse_states);
  return true;
}

}  // namespace session
}  // namespace sar
