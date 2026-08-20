/**
 * @file AircraftPerformanceDerivation.cpp
 * @brief 飞机性能推导单一实现（见 AircraftPerformanceDerivation.h）。
 */

#include "flight_dynamic/AircraftPerformanceDerivation.h"

#include <cmath>

namespace oneq {
namespace flight_dynamic {

double SelectClMax(const PerformanceDerivationInputs& inputs) {
  // XML override 优先：物理合理（> 0.5）才采用，与 EngineManager 既有行为一致。
  if (inputs.has_cl_max_override && inputs.cl_max_override > 0.5) {
    return inputs.cl_max_override;
  }

  double cl_max = kClMaxTakeoffDefault;
  if (inputs.is_turboprop) {
    cl_max = kClMaxTakeoffTurboprop;
  }
  // delta / 低展弦比检测优先于 turboprop（与 EngineManager 既有顺序一致）。
  if (inputs.wingspan_ft > 1.0 && inputs.wing_area_ft2 > 1.0) {
    const double aspect_ratio = (inputs.wingspan_ft * inputs.wingspan_ft) / inputs.wing_area_ft2;
    if (aspect_ratio < kDeltaWingArThreshold) {
      cl_max = kClMaxTakeoffDeltaWing;
    }
  }
  return cl_max;
}

PerformanceDerivationResult DeriveStallAndWingLoading(const PerformanceDerivationInputs& inputs,
                                                      double rho_slugs_ft3) {
  PerformanceDerivationResult result = {};
  result.cl_max = SelectClMax(inputs);
  // 非法输入时 v_stall 留 0（调用方各自做 validation/fallback，不在 helper 内处理）。
  if (std::isfinite(inputs.weight_lbs) && inputs.weight_lbs > 0.0 &&
      std::isfinite(inputs.wing_area_ft2) && inputs.wing_area_ft2 > 0.0 &&
      std::isfinite(rho_slugs_ft3) && rho_slugs_ft3 > 0.0 &&
      std::isfinite(result.cl_max) && result.cl_max > 0.0) {
    const double v_stall_ftps =
        std::sqrt((2.0 * inputs.weight_lbs) / (rho_slugs_ft3 * inputs.wing_area_ft2 * result.cl_max));
    if (std::isfinite(v_stall_ftps)) result.v_stall_ftps = v_stall_ftps;
  }
  return result;
}

DynamicSpeedEnvelopeResult DeriveDynamicSpeedEnvelope(
    const DynamicSpeedEnvelopeInputs& inputs) {
  DynamicSpeedEnvelopeResult result = {};
  if (!std::isfinite(inputs.v_stall_mps) || inputs.v_stall_mps <= 0.0 ||
      !std::isfinite(inputs.wing_loading_lbs_ft2) ||
      inputs.wing_loading_lbs_ft2 <= 0.0) {
    return result;
  }

  double cruise_factor = 2.8;
  if (!inputs.is_piston) {
    cruise_factor = 2.89 + 0.00455 * inputs.wing_loading_lbs_ft2;
    if (inputs.has_fbw) cruise_factor += 0.25;
    if (cruise_factor < 2.8) cruise_factor = 2.8;
    if (cruise_factor > 4.0) cruise_factor = 4.0;
  }

  double max_factor = cruise_factor + 0.8;
  if (inputs.has_fbw) {
    max_factor = cruise_factor + 2.0;
  } else if (inputs.is_heavy || inputs.is_piston) {
    max_factor = cruise_factor + 0.7;
  }

  double stall_margin = 1.30;
  if (inputs.has_fbw) {
    stall_margin = 1.25;
  } else if (inputs.is_heavy && !inputs.is_piston) {
    stall_margin = 1.20;
  }

  result.valid = true;
  result.min_speed_mps = inputs.v_stall_mps * stall_margin;
  result.cruise_speed_mps = inputs.v_stall_mps * cruise_factor;
  result.max_speed_mps = inputs.v_stall_mps * max_factor;
  result.ref_speed_mps = result.max_speed_mps;
  return result;
}

}  // namespace flight_dynamic
}  // namespace oneq
