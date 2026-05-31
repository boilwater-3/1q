#include "flight_dynamic/propulsion/EngineManager.h"

#include <string>

#include "FGFDMExec.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "models/FGPropulsion.h"
#include "models/FGFCS.h"

namespace oneq {
namespace flight_dynamic {
namespace propulsion {

EngineManager::EngineManager(adapter::JsbsimAdapter& adapter)
    : adapter_(adapter), exec_(adapter.GetFdmExec()) {
  auto* pm = exec_.GetPropertyManager().get();
  has_magneto_ = pm->GetNode(adapter::property::kMagnetoCmd) != nullptr;
  has_starter_ = pm->GetNode(adapter::property::kStarterCmd) != nullptr;
  has_mixture_ = pm->GetNode(adapter::property::kMixtureCmdNorm) != nullptr;
  has_wow_ = pm->GetNode(adapter::property::kWowMain) != nullptr;
  count_ = static_cast<int>(exec_.GetPropulsion()->GetNumEngines());
  DetectType();
}

void EngineManager::DetectType() {
  // Probe the property tree to classify the engine.
  // Magneto exists only for piston engines (JSBSim FGPiston).
  if (has_magneto_) {
    type_ = EngineType::kPiston;
    return;
  }
  // Check for turboprop: has starter but no magneto, and the engine XML
  // may declare turboprop_engine. Fall back to turbine if unclear.
  if (has_starter_ && count_ > 0) {
    // Could be turboprop or turbine. Look at engine RPM property for clue:
    // turboprops typically have propeller-related properties.
    auto* pm = exec_.GetPropertyManager().get();
    if (pm->GetNode("propulsion/engine[0]/propeller-rpm") != nullptr) {
      type_ = EngineType::kTurboprop;
      return;
    }
    type_ = EngineType::kTurbine;
    return;
  }
  // No magneto, no starter: could be turbine (InitRunning), rocket, or electric.
  if (count_ > 0) {
    type_ = EngineType::kTurbine;
  } else {
    type_ = EngineType::kUnknown;
  }
}

void EngineManager::Start() {
  switch (type_) {
    case EngineType::kPiston:
      adapter_.SetProperty(adapter::property::kMixtureCmdNorm, 1.0);
      adapter_.SetProperty(adapter::property::kMagnetoCmd, 3.0);
      adapter_.SetProperty(adapter::property::kStarterCmd, 1.0);
      break;
    case EngineType::kTurboprop:
      adapter_.SetProperty(adapter::property::kStarterCmd, 1.0);
      break;
    default:
      // Turbine, rocket, electric: started by InitRunning() during adapter init.
      break;
  }
}

void EngineManager::SetThrottle(double value) {
  // Always set the unindexed property (works for single-engine).
  adapter_.SetProperty(adapter::property::kThrottleCmd, value);
  // Also set per-engine indexed properties for multi-engine aircraft.
  for (int i = 0; i < count_; ++i) {
    SetIndexedProperty(adapter::property::kThrottleCmd, i, value);
    SetIndexedProperty(adapter::property::kThrottlePosNorm, i, value);
  }
}

void EngineManager::SetBrakes(bool on) {
  double v = on ? 1.0 : 0.0;
  adapter_.SetProperty(adapter::property::kLeftBrakeCmd, v);
  adapter_.SetProperty(adapter::property::kRightBrakeCmd, v);
  adapter_.SetProperty(adapter::property::kCenterBrakeCmd, v);
}

void EngineManager::SetFlaps(double value) {
  adapter_.SetProperty(adapter::property::kFlapCmdNorm, value);
}

void EngineManager::SetGearDown(bool down) {
  double v = down ? 1.0 : 0.0;
  adapter_.SetProperty(adapter::property::kGearCmdNorm, v);
  adapter_.SetProperty(adapter::property::kGearPosNorm, v);
}

bool EngineManager::IsWeightOnWheels() const {
  if (!has_wow_) return false;
  return GetProperty(adapter::property::kWowMain) > 0.5;
}

void EngineManager::SetMixture(double value) {
  if (has_mixture_) {
    adapter_.SetProperty(adapter::property::kMixtureCmdNorm, value);
  }
}

void EngineManager::SetIndexedProperty(const std::string& base, int index,
                                       double value) {
  adapter_.SetProperty(base + "[" + std::to_string(index) + "]", value);
}

double EngineManager::GetProperty(const std::string& name) const {
  auto* node = exec_.GetPropertyManager()->GetNode(name);
  return node ? node->getDoubleValue() : 0.0;
}

double EngineManager::GetRotationSpeedKts() const {
  // Compute stall speed from wing loading.
  // V_stall = sqrt(2*W / (rho * S * CLmax))
  // All units in JSBSim imperial: lbs, ft², slugs/ft³ → result in ft/s.
  const double weight_lbs = GetProperty("inertia/weight-lbs");
  const double wing_area_ft2 = GetProperty("metrics/Sw-sqft");
  const double rho = GetProperty("atmosphere/rho-slugs_ft3");
  constexpr double kClMaxTakeoff = 1.6;  // takeoff flaps ~10-20°

  if (weight_lbs < 1.0 || wing_area_ft2 < 1.0 || rho < 1e-9) {
    return 50.0;
  }

  const double v_stall_ftps =
      std::sqrt((2.0 * weight_lbs) / (rho * wing_area_ft2 * kClMaxTakeoff));
  const double v_stall_kts = v_stall_ftps * 0.592484;

  double vr_kts = 0.0;
  switch (type_) {
    case EngineType::kPiston:    vr_kts = 1.10 * v_stall_kts; break;
    case EngineType::kTurbine:   vr_kts = 1.20 * v_stall_kts; break;
    case EngineType::kTurboprop: vr_kts = 1.15 * v_stall_kts; break;
    default:                     vr_kts = 1.15 * v_stall_kts; break;
  }
  // Sanity floor: any flyable aircraft needs at least 40 kts to rotate.
  return std::max(vr_kts, 40.0);
}

double EngineManager::GetDefaultApproachSpeedMps() const {
  // Type-based approach speeds when Vr calculation is unavailable.
  // Target is ~1.3 × Vref for each category.
  switch (type_) {
    case EngineType::kPiston:    return 28.0;   // ~55 kts (C172: Vref~45)
    case EngineType::kTurboprop: return 41.0;   // ~80 kts
    case EngineType::kTurbine:   return 62.0;   // ~120 kts (737: Vref~130)
    case EngineType::kRocket:    return 80.0;   // ~155 kts
    default:                     return 36.0;   // ~70 kts
  }
}

double EngineManager::GetClimbPitchDeg() const {
  switch (type_) {
    case EngineType::kPiston:    return 10.0;
    case EngineType::kTurbine:   return 15.0;
    case EngineType::kTurboprop: return 10.0;
    default:                     return 10.0;
  }
}

}  // namespace propulsion
}  // namespace flight_dynamic
}  // namespace oneq
