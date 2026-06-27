#include "airborne_radar/decision/LpiEvaluator.h"

#include <string>

#include "1q/airborne_radar/session/ControlDirective.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {

namespace {

session::ControlDirective BuildLpiPowerDirective() {
  return session::ControlDirective(
      session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
      session::ControlDirectiveSource::EMISSION_CONTROL);
}

}  // namespace

LpiEvaluator::Result LpiEvaluator::Evaluate(
    const model::LpiSourceInfo& lpi_source_info,
    std::vector<session::TacticalProposal>* proposals) {
  Result result;

  if (proposals == nullptr) {
    return result;
  }

  if (!lpi_source_info.has_recon_platform) {
    PROJECT_LOG_INFO("[LpiEvaluator] No reconnaissance platform. LPI remains inactive.");
    return result;
  }

  // 距离大于 100km 且无接近趋势时暂不触发 LPI
  if (lpi_source_info.threat_range_km > 100.0f &&
      lpi_source_info.threat_closure_speed_mps < 50.0f) {
    PROJECT_LOG_INFO(
        "[LpiEvaluator] Recon platform at {:.1f}km is beyond effective LPI range.",
        lpi_source_info.threat_range_km);
    return result;
  }

  result.requests_power_reduction = true;
  const float power_scale = ComputePowerScale(lpi_source_info);
  result.power_scale = power_scale;

  std::string rationale = "recon platform";
  if (lpi_source_info.threat_range_km > 0.0f) {
    rationale += " at ";
    rationale += std::to_string(static_cast<int>(lpi_source_info.threat_range_km));
    rationale += "km";
  }
  rationale += " requires reduced emission";

  proposals->push_back(session::TacticalProposal{
      BuildLpiPowerDirective(), 60, rationale});

  PROJECT_LOG_INFO(
      "[LpiEvaluator] Recon platform detected. Appending LPI power reduction proposal "
      "(power_scale={:.2f}, range={:.1f}km, closure_speed={:.1f}m/s).",
      power_scale, lpi_source_info.threat_range_km,
      lpi_source_info.threat_closure_speed_mps);

  return result;
}

float LpiEvaluator::ComputePowerScale(const model::LpiSourceInfo& info) const {
  // 无距离信息时使用保守默认值
  if (info.threat_range_km <= 0.0f) {
    return 0.5f;
  }

  // 近距离威胁：激进降功率
  if (info.threat_range_km < 20.0f) {
    return 0.3f;
  }

  // 中距离：根据接近速度调整
  if (info.threat_range_km < 50.0f) {
    return (info.threat_closure_speed_mps > 200.0f) ? 0.4f : 0.5f;
  }

  // 50-100km：中度降功率
  if (info.threat_range_km < 100.0f) {
    return (info.threat_closure_speed_mps > 200.0f) ? 0.6f : 0.7f;
  }

  // 100km 以外：保守降功率
  return 0.8f;
}

}  // namespace decision
}  // namespace airborne_radar
