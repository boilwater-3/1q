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
  PerformanceDerivationResult result;
  result.cl_max = SelectClMax(inputs);
  // 非法输入时 v_stall 留 0（调用方各自做 validation/fallback，不在 helper 内处理）。
  if (inputs.weight_lbs > 0.0 && inputs.wing_area_ft2 > 0.0 && rho_slugs_ft3 > 0.0 &&
      result.cl_max > 0.0) {
    result.v_stall_ftps =
        std::sqrt((2.0 * inputs.weight_lbs) / (rho_slugs_ft3 * inputs.wing_area_ft2 * result.cl_max));
  }
  return result;
}

}  // namespace flight_dynamic
}  // namespace oneq
