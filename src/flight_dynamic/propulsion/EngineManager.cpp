#include "flight_dynamic/propulsion/EngineManager.h"

#include <cmath>
#include <string>

#include <spdlog/spdlog.h>

#include "FGFDMExec.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "models/FGFCS.h"
#include "models/FGPropulsion.h"

namespace oneq {
namespace flight_dynamic {
namespace propulsion {

namespace {

// --- CLmax takeoff defaults (aircraft capability) ---
// These are wing-physics parameters that vary by planform and high-lift devices.
// They feed Vr = Vr_factor × sqrt(2W / (ρ × S × CLmax)).
constexpr double kClMaxTakeoffDefault = 1.6;    // clean wing / simple flaps
constexpr double kClMaxTakeoffTurboprop = 2.0;   // high-lift wings, takeoff flaps ≥33%
constexpr double kClMaxTakeoffDeltaWing = 2.5;   // vortex lift at high AoA

// --- Delta-wing detection (aerodynamic classification) ---
// Aspect ratio = span² / area.  AR < 2.5 is the conventional boundary
// for delta / low-AR planforms that generate significant vortex lift.
constexpr double kDeltaWingArThreshold = 2.5;

// --- Vr multiplier (aircraft capability × safety margin) ---
// Vr = factor × V_stall.  Heavy aircraft with multi-slot flaps / slats
// have higher CLmax in takeoff configuration, so Vr is closer to V_stall.
// The pitch moment of inertia (Iyy) is a direct proxy for weight class.
constexpr double kIyyHeavyThreshold = 1.0e7;      // B747, MD11, Concorde
constexpr double kIyyMediumThreshold = 1.0e6;     // 737, C130-class
constexpr double kVrFactorHeavyTurbine = 1.08;
constexpr double kVrFactorMediumTurbine = 1.15;
constexpr double kVrFactorLightTurbine = 1.20;
constexpr double kVrFactorPiston = 1.10;
constexpr double kVrFactorDefault = 1.15;

// --- Safety bounds ---
constexpr double kMinVrKts = 40.0;                // any flyable aircraft needs ≥40 kts
constexpr double kFallbackVrKts = 50.0;           // returned when inputs are invalid
constexpr double kClMaxLowerBound = 1.0;          // physically impossible below ~1.0
constexpr double kClMaxUpperBound = 3.5;          // realistic upper bound (blown flaps)

// --- Approach speed defaults by engine category (last-resort fallback) ---
// Target ≈1.3 × Vref.  Only used when Vr-calculation path is unavailable.
constexpr double kApproachSpeedPistonMps = 28.0;      // ~55 kts (C172 Vref≈45)
constexpr double kApproachSpeedTurbopropMps = 41.0;   // ~80 kts
constexpr double kApproachSpeedTurbineMps = 62.0;     // ~120 kts (737 Vref≈130)
constexpr double kApproachSpeedRocketMps = 80.0;      // ~155 kts
constexpr double kApproachSpeedDefaultMps = 36.0;     // ~70 kts

// --- Climb pitch defaults by engine category ---
constexpr double kClimbPitchPistonDeg = 10.0;
constexpr double kClimbPitchTurbineDeg = 15.0;
constexpr double kClimbPitchTurbopropDeg = 10.0;
constexpr double kClimbPitchDefaultDeg = 10.0;

}  // namespace

EngineManager::EngineManager(adapter::JsbsimAdapter& adapter)
    : adapter_(adapter), exec_(adapter.GetFdmExec()) {
  auto* pm = exec_.GetPropertyManager().get();
  has_magneto_ = pm->GetNode(adapter::property::kMagnetoCmd) != nullptr;
  has_starter_ = pm->GetNode(adapter::property::kStarterCmd) != nullptr;
  // Only treat as having mixture if BOTH mixture and magneto exist (piston-only).
  // JSBSim creates indexed fcs/mixture-cmd-norm[n] for all engine types.
  has_mixture_ = pm->GetNode(adapter::property::kMixtureCmdNorm) != nullptr &&
                 has_magneto_;
  has_wow_ = pm->GetNode(adapter::property::kWowMain) != nullptr;
  count_ = static_cast<int>(exec_.GetPropulsion()->GetNumEngines());
  DetectType();
  MeasureRatedThrust();
}

void EngineManager::DetectType() {
  if (count_ <= 0) {
    type_ = EngineType::kUnknown;
    return;
  }

  switch (exec_.GetPropulsion()->GetEngine(0)->GetType()) {
    case JSBSim::FGEngine::etPiston:
      type_ = EngineType::kPiston;
      return;
    case JSBSim::FGEngine::etTurbine:
      type_ = EngineType::kTurbine;
      return;
    case JSBSim::FGEngine::etTurboprop:
      type_ = EngineType::kTurboprop;
      return;
    case JSBSim::FGEngine::etRocket:
      type_ = EngineType::kRocket;
      return;
    case JSBSim::FGEngine::etElectric:
      type_ = EngineType::kElectric;
      return;
    case JSBSim::FGEngine::etUnknown:
      break;
  }

  // Fall back to legacy property probes for aircraft with incomplete engine metadata.
  if (has_magneto_) {
    type_ = EngineType::kPiston;
  } else if (has_starter_) {
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
      for (int i = 0; i < count_; ++i) {
        exec_.GetFCS()->SetPropAdvanceCmd(i, 1.0);
        exec_.GetFCS()->SetPropAdvance(i, 1.0);
      }
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

void EngineManager::SetIndexedProperty(const std::string& base, int index, double value) {
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
  const double iyy = GetProperty("inertia/iyy-slugs_ft2");

  // --- Input validation ---
  if (weight_lbs < 1.0 || wing_area_ft2 < 1.0 || rho < 1e-9) {
    spdlog::warn("[ENGINE] GetRotationSpeedKts: invalid inputs "
                 "weight={:.0f}lbs area={:.1f}ft² rho={:.6f} → fallback {:.0f} kts",
                 weight_lbs, wing_area_ft2, rho, kFallbackVrKts);
    return kFallbackVrKts;
  }

  // --- CLmax selection (aircraft capability, wing physics) ---
  // These are wing-planform parameters, not control-law tuning knobs.
  double cl_max = kClMaxTakeoffDefault;
  if (type_ == EngineType::kTurboprop) {
    cl_max = kClMaxTakeoffTurboprop;
  }
  // Detect delta-wing / low-aspect-ratio planforms via wing geometry.
  const double wingspan_ft = GetProperty("metrics/bw-ft");
  bool is_delta_wing = false;
  double aspect_ratio = 0.0;
  if (wingspan_ft > 1.0 && wing_area_ft2 > 1.0) {
    aspect_ratio = (wingspan_ft * wingspan_ft) / wing_area_ft2;
    if (aspect_ratio < kDeltaWingArThreshold) {
      cl_max = kClMaxTakeoffDeltaWing;
      is_delta_wing = true;
    }
  }

  // Diagnostic: log CLmax for unusual values (outside realistic bounds).
  if (cl_max < kClMaxLowerBound || cl_max > kClMaxUpperBound) {
    spdlog::warn("[ENGINE] GetRotationSpeedKts: CLmax={:.2f} out of bounds [{:.1f}, {:.1f}]",
                 cl_max, kClMaxLowerBound, kClMaxUpperBound);
  }

  const double v_stall_ftps = std::sqrt((2.0 * weight_lbs) / (rho * wing_area_ft2 * cl_max));
  const double v_stall_kts = v_stall_ftps * 0.592484;

  // --- Vr factor (aircraft capability × safety margin) ---
  // Heavy aircraft with multi-slot flaps / slats have higher CLmax in
  // takeoff configuration, so Vr is closer to V_stall.  Iyy is used as
  // a weight-class proxy — it correlates strongly with aircraft size
  // and the availability of complex high-lift systems.
  double vr_factor = kVrFactorDefault;
  if (type_ == EngineType::kTurbine) {
    if (iyy > kIyyHeavyThreshold) {
      vr_factor = kVrFactorHeavyTurbine;
    } else if (iyy > kIyyMediumThreshold) {
      vr_factor = kVrFactorMediumTurbine;
    } else {
      vr_factor = kVrFactorLightTurbine;
    }
  } else if (type_ == EngineType::kPiston) {
    vr_factor = kVrFactorPiston;
  }

  const double vr_kts = std::max(vr_factor * v_stall_kts, kMinVrKts);

  spdlog::debug("[ENGINE] Vr={:.1f} kts  V_stall={:.1f} kts  CLmax={:.1f}  "
                "Vr_factor={:.3f}  Iyy={:.1e}  AR={:.2f}{}  weight={:.0f} lbs  "
                "S={:.0f} ft²  rho={:.4f}",
                vr_kts, v_stall_kts, cl_max, vr_factor, iyy, aspect_ratio,
                is_delta_wing ? " δ" : "", weight_lbs, wing_area_ft2, rho);

  return vr_kts;
}

double EngineManager::GetDefaultApproachSpeedMps() const {
  // Type-based approach speeds when Vr calculation is unavailable.
  // These are LAST-RESORT fallbacks: the Maneuver layer prioritises
  //  1) profile landing_approach_speed_mps (from XML or explicit set)
  //  2) caller-supplied approach_speed_mps argument
  //  3) Vr × 1.3 (from GetRotationSpeedKts)
  //  4) this method (engine-category default)
  // Target ≈1.3 × Vref for each category.
  double spd = kApproachSpeedDefaultMps;
  switch (type_) {
    case EngineType::kPiston:
      spd = kApproachSpeedPistonMps;
      break;
    case EngineType::kTurboprop:
      spd = kApproachSpeedTurbopropMps;
      break;
    case EngineType::kTurbine:
      spd = kApproachSpeedTurbineMps;
      break;
    case EngineType::kRocket:
      spd = kApproachSpeedRocketMps;
      break;
    default:
      break;
  }
  spdlog::debug("[ENGINE] DefaultApproachSpeed={:.1f} m/s (type={})",
                spd, static_cast<int>(type_));
  return spd;
}

double EngineManager::GetClimbPitchDeg() const {
  // Engine-type-based initial climb pitch target.
  // These are typical best-climb attitudes — turbine aircraft have
  // higher thrust-to-weight and can sustain steeper climb angles.
  double pitch = kClimbPitchDefaultDeg;
  switch (type_) {
    case EngineType::kPiston:
      pitch = kClimbPitchPistonDeg;
      break;
    case EngineType::kTurbine:
      pitch = kClimbPitchTurbineDeg;
      break;
    case EngineType::kTurboprop:
      pitch = kClimbPitchTurbopropDeg;
      break;
    default:
      break;
  }
  spdlog::debug("[ENGINE] ClimbPitch={:.0f} deg (type={})",
                pitch, static_cast<int>(type_));
  return pitch;
}

void EngineManager::MeasureRatedThrust() {
  // Estimate rated maximum static thrust from current engine state.
  // This approach is NON-INVASIVE: it does not call InitRunning or change
  // any engine state, so it cannot cause flight dynamics regressions.
  //
  // Strategy: read current thrust/HP from the JSBSim property tree and
  // scale by throttle position to estimate rated max values.
  //   rated_thrust ≈ current_thrust / max(throttle_pos, 0.1)
  // This is approximate (thrust is not perfectly linear with throttle)
  // but sufficient for TWR-based speed envelope discrimination.
  if (count_ <= 0) return;

  const double weight_lbs = GetProperty("inertia/weight-lbs");
  if (weight_lbs < 1.0) return;

  // Read current throttle position (use indexed property for multi-engine).
  double throttle_pos = GetProperty("fcs/throttle-cmd-norm");
  if (throttle_pos < 0.1) {
    // At very low throttle, the linear estimate is unreliable.
    // Fall back to engine-type-based estimate using wing loading as proxy.
    const double wing_area_ft2 = GetProperty("metrics/Sw-sqft");
    if (wing_area_ft2 > 1.0) {
      double wl = weight_lbs / wing_area_ft2;
      // Rough TWR estimates by wing loading class:
      //   GA piston (WL<30):    TWR ≈ 0.25
      //   Turboprop (WL 30-60): TWR ≈ 0.35
      //   Medium jet (WL 60-120): TWR ≈ 0.30
      //   Heavy/fighter (WL>120): TWR ≈ 0.28 (heavy) or 0.80 (fighter)
      if (wl < 30.0) {
        rated_thrust_lbs_ = 0.25 * weight_lbs;
      } else if (wl < 60.0) {
        rated_thrust_lbs_ = 0.35 * weight_lbs;
      } else if (wl < 120.0) {
        rated_thrust_lbs_ = 0.30 * weight_lbs;
      } else {
        rated_thrust_lbs_ = 0.28 * weight_lbs;
      }
    }
  } else {
    // Scale current thrust by throttle to estimate rated max.
    double total = 0.0;
    for (int i = 0; i < count_; ++i) {
      if (type_ == EngineType::kPiston) {
        const std::string hp_prop =
            "propulsion/engine[" + std::to_string(i) + "]/power-hp";
        double hp = GetProperty(hp_prop);
        total += (hp / throttle_pos) * 3.5;  // scale to rated, convert to lbs
      } else {
        const std::string thrust_prop =
            "propulsion/engine[" + std::to_string(i) + "]/thrust-lbs";
        double thrust = GetProperty(thrust_prop);
        total += thrust / throttle_pos;  // linear scale to rated
      }
    }
    rated_thrust_lbs_ = total;
  }

  // Publish TWR to JSBSim property tree for other components (Autopilot).
  if (rated_thrust_lbs_ > 0.0) {
    double twr = rated_thrust_lbs_ / weight_lbs;
    auto* pm = exec_.GetPropertyManager().get();
    auto* node = pm->GetNode("guidance/thrust-to-weight", true);
    if (node) node->setDoubleValue(twr);
    spdlog::debug("[ENGINE] Rated thrust={:.0f} lbs  weight={:.0f} lbs  TWR={:.3f}",
                  rated_thrust_lbs_, weight_lbs, twr);
  }
}

double EngineManager::GetTotalThrustLbs() const { return rated_thrust_lbs_; }

double EngineManager::GetThrustToWeight() const {
  const double weight_lbs = GetProperty("inertia/weight-lbs");
  if (weight_lbs < 1.0 || rated_thrust_lbs_ <= 0.0) return 0.0;
  return rated_thrust_lbs_ / weight_lbs;
}

}  // namespace propulsion
}  // namespace flight_dynamic
}  // namespace oneq
