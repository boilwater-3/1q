#include "1q/flight_dynamic/autopilot/Autopilot.h"

#include <algorithm>
#include <cmath>

#include "common/logging/ProjectLog.h"
#include "flight_dynamic/AircraftPerformanceDerivation.h"
#include "flight_dynamic/AngleNormalization.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oneq {
namespace flight_dynamic {
namespace autopilot {

namespace {

double Clamp(double value, double min_value, double max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

constexpr double kFtToM = 0.3048;
constexpr double kMToFt = 1.0 / kFtToM;
constexpr double kRefSpeedFps = 164.0;  // ~50 m/s reference (c172x cruise)

double ReadPropertyOrDefault(adapter::JsbsimAdapter& adapter, const char* name,
                             double default_value) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();
  auto* node = pm ? pm->GetNode(name) : nullptr;
  return node ? node->getDoubleValue() : default_value;
}

bool HasProperty(adapter::JsbsimAdapter& adapter, const char* name) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();
  return pm && pm->GetNode(name) != nullptr;
}

// Set energy-management defaults from aircraft physics.
// Speed envelope is derived from V_stall (which encodes actual weight,
// wing area, and CLmax); ceiling is derived from wing loading.
// cruise_factor is a continuous function of wing loading, replacing the
// previous 5-category discrete classification.  Safety-critical parameters
// (stall_margin, max_factor, roll_lim, etc.) remain in discrete categories.
void ApplyEnergyDefaults(AircraftControlProfile* profile) {
  if (!profile) return;

  const double vs = profile->v_stall_mps;  // clean stall speed (m/s TAS)
  if (vs <= 0.0) return;  // physics data unavailable → keep struct defaults

  const double wl = profile->wing_loading_lbs_ft2;  // wing loading
  const int n_eng = profile->engine_count;
  const bool has_fbw = profile->has_fbw_override || profile->has_roll_rate_command;
  const bool is_heavy = profile->engine_count >= 4 ||
      (profile->pitch_moi_lbsft2 > 0.0 && std::log10(profile->pitch_moi_lbsft2) > 7.0);
  const bool is_medium = !is_heavy && profile->pitch_moi_lbsft2 > 0.0 &&
      std::log10(profile->pitch_moi_lbsft2) > 6.0;

  // ── Speed envelope from V_stall ──────────────────────────────────────
  //
  // min_speed  = V_stall × margin          (prevents stall in turns)
  // cruise     = V_stall × cruise_factor   (efficient cruise ~75% power)
  // max_speed  = V_stall × max_factor      (structural / thrust limit)
  //
  // cruise_factor is now continuous in wing loading:
  //   Piston:     CF = 2.8 (flat — all GA pistons have similar WL range)
  //   Non-piston: CF = 2.89 + 0.00455 × WL  (linear fit)
  //     WL=25→3.00, WL=50→3.12, WL=92→3.30, WL=135→3.50
  //   FBW bonus: +0.25 for fly-by-wire fighters (can sustain higher cruise)
  //
  // Data from 17 aircraft (empty weight × 1.3 / wing area):
  //   c172x WL=10.9, c310 WL=21.9, DHC6 WL=24.9, A4 WL=51.2, T38 WL=57.9,
  //   f15 WL=59.9, f16 WL=75.4, 737 WL=92.1, B747 WL=120.6, MD11 WL=134.8

  // Safety-critical parameters: keep discrete categories.
  double stall_margin, pitch_cmd, roll_lim, min_thr;

  if (has_fbw) {
    stall_margin  = 1.25;
    pitch_cmd = 15.0;  roll_lim = 45.0;  min_thr = 0.35;
  } else if (is_heavy && !profile->has_mixture) {
    stall_margin  = 1.20;
    pitch_cmd = 8.0;   roll_lim = 35.0;  min_thr = 0.55;
  } else if (profile->has_mixture) {
    if (profile->engine_count >= 4) {
      // Multi-engine heavy piston (B17)
      stall_margin  = 1.30;
      pitch_cmd = 10.0;  roll_lim = 25.0;  min_thr = 0.40;
    } else {
      // Single/twin piston GA (c172x, c310)
      stall_margin  = 1.30;
      pitch_cmd = n_eng >= 2 ? 10.0 : 12.0;
      roll_lim = 30.0;
      min_thr = n_eng >= 2 ? 0.30 : 0.20;
    }
  } else {
    // Medium / light turbine / turboprop
    stall_margin  = 1.30;
    pitch_cmd = 20.0;  roll_lim = 45.0;  min_thr = 0.15;
  }

  // Speed-energy priority: high wing loading (>50 lbs/ft²) means high
  // kinetic energy at stall, so speed recovery is more urgent.  FBW
  // aircraft always get priority — they have flight envelope protection
  // that can compensate for altitude excursions.
  // Physical basis: KE = ½mv² ∝ W × V_stall² ∝ W × (W/(ρ·S·CLmax))
  //              = W²/(ρ·S·CLmax) ∝ WL × W/CLmax
  // Higher WL → proportionally more kinetic energy → speed loss harder
  // to recover → prioritize speed over altitude in energy management.
  bool spd_prio = has_fbw || (wl > 50.0);

  DynamicSpeedEnvelopeInputs envelope_inputs;
  envelope_inputs.v_stall_mps = vs;
  envelope_inputs.wing_loading_lbs_ft2 = wl;
  envelope_inputs.is_piston = profile->has_mixture;
  envelope_inputs.has_fbw = has_fbw;
  envelope_inputs.is_heavy = is_heavy;
  const DynamicSpeedEnvelopeResult envelope =
      DeriveDynamicSpeedEnvelope(envelope_inputs);
  if (envelope.valid) {
    profile->min_speed_mps = envelope.min_speed_mps;
    profile->cruise_speed_mps = envelope.cruise_speed_mps;
    profile->max_speed_mps = envelope.max_speed_mps;
    profile->ref_speed_mps = envelope.ref_speed_mps;
  }

  profile->max_pitch_command_deg = pitch_cmd;
  profile->max_roll_angle_deg    = roll_lim;
  profile->min_throttle          = min_thr;
  profile->speed_energy_priority = spd_prio;

  // ── Ceiling by category ──────────────────────────────────────────────
  //
  // Service ceiling depends primarily on engine power/supercharging,
  // which JSBSim doesn't expose as a single property.  Category defaults
  // provide a reasonable estimate; XML override handles special cases.
  if (profile->ceiling_m <= 0.0) {
    if (has_fbw)                               profile->ceiling_m = 15200.0;
    else if (is_heavy && !profile->has_mixture) profile->ceiling_m = 13700.0;
    else if (is_medium && !profile->has_mixture) profile->ceiling_m = 12500.0;
    else if (profile->has_mixture)             profile->ceiling_m = 4300.0;
    else                                       profile->ceiling_m = 7600.0;
  }
}

void ApplyXmlSpeedOverrides(adapter::JsbsimAdapter& adapter,
                            AircraftControlProfile* profile) {
  profile->ref_speed_mps =
      ReadPropertyOrDefault(adapter, "guidance/ref-speed-mps", profile->ref_speed_mps);
  profile->cruise_speed_mps = ReadPropertyOrDefault(
      adapter, "guidance/cruise-speed-mps", profile->cruise_speed_mps);
  profile->min_speed_mps =
      ReadPropertyOrDefault(adapter, "guidance/min-speed-mps", profile->min_speed_mps);
  profile->max_speed_mps =
      ReadPropertyOrDefault(adapter, "guidance/max-speed-mps", profile->max_speed_mps);
}

bool RefreshDynamicSpeedEnvelope(adapter::JsbsimAdapter& adapter,
                                 AircraftControlProfile* profile,
                                 bool allow_standard_density_fallback) {
  const double weight_lbs = ReadPropertyOrDefault(adapter, "inertia/weight-lbs", 0.0);
  const double wing_area_ft2 = ReadPropertyOrDefault(adapter, "metrics/Sw-sqft", 0.0);
  const double wingspan_ft = ReadPropertyOrDefault(adapter, "metrics/bw-ft", 0.0);
  double rho = ReadPropertyOrDefault(adapter, "atmosphere/rho-slugs_ft3", 0.0);
  if (!std::isfinite(weight_lbs) || weight_lbs <= 0.0 ||
      !std::isfinite(wing_area_ft2) || wing_area_ft2 <= 0.0) {
    return false;
  }
  if (!std::isfinite(rho) || rho <= 0.0) {
    if (!allow_standard_density_fallback) return false;
    PROJECT_LOG_WARN(
        "[AUTOPILOT] Invalid initial atmosphere density; using standard sea-level "
        "density for the baseline TAS envelope");
    rho = kStandardSeaLevelDensitySlugsFt3;
  }

  bool is_turboprop = false;
  const auto propulsion = adapter.GetFdmExec().GetPropulsion();
  if (propulsion && propulsion->GetNumEngines() > 0) {
    is_turboprop =
        propulsion->GetEngine(0)->GetType() == JSBSim::FGEngine::etTurboprop;
  }
  PerformanceDerivationInputs performance_inputs;
  performance_inputs.weight_lbs = weight_lbs;
  performance_inputs.wing_area_ft2 = wing_area_ft2;
  performance_inputs.wingspan_ft = wingspan_ft;
  performance_inputs.is_turboprop = is_turboprop;
  performance_inputs.has_cl_max_override = false;
  performance_inputs.cl_max_override = 0.0;
  const PerformanceDerivationResult performance =
      DeriveStallAndWingLoading(performance_inputs, rho);
  if (performance.v_stall_ftps <= 0.0) return false;

  const double wing_loading_lbs_ft2 = weight_lbs / wing_area_ft2;
  const double v_stall_mps = performance.v_stall_ftps * kFtToM;
  const bool has_fbw = profile->has_fbw_override || profile->has_roll_rate_command;
  const bool is_heavy = profile->engine_count >= 4 ||
      (profile->pitch_moi_lbsft2 > 0.0 && std::log10(profile->pitch_moi_lbsft2) > 7.0);
  DynamicSpeedEnvelopeInputs envelope_inputs;
  envelope_inputs.v_stall_mps = v_stall_mps;
  envelope_inputs.wing_loading_lbs_ft2 = wing_loading_lbs_ft2;
  envelope_inputs.is_piston = profile->has_mixture;
  envelope_inputs.has_fbw = has_fbw;
  envelope_inputs.is_heavy = is_heavy;
  const DynamicSpeedEnvelopeResult envelope =
      DeriveDynamicSpeedEnvelope(envelope_inputs);
  if (!envelope.valid) return false;
  profile->wing_loading_lbs_ft2 = wing_loading_lbs_ft2;
  profile->v_stall_mps = v_stall_mps;
  profile->min_speed_mps = envelope.min_speed_mps;
  profile->cruise_speed_mps = envelope.cruise_speed_mps;
  profile->max_speed_mps = envelope.max_speed_mps;
  profile->ref_speed_mps = envelope.ref_speed_mps;
  ApplyXmlSpeedOverrides(adapter, profile);
  return true;
}

// XML guidance/* properties override dynamic defaults.  This keeps property-tree
// detection as the fallback while allowing aircraft XML to carry tuning that is
// specific to its flight envelope or model limitations.
void ApplyXmlProfileOverrides(adapter::JsbsimAdapter& adapter, AircraftControlProfile* profile) {
  if (!profile) return;

  ApplyXmlSpeedOverrides(adapter, profile);
  profile->max_pitch_command_deg = ReadPropertyOrDefault(
      adapter, "guidance/max-pitch-command-deg", profile->max_pitch_command_deg);
  profile->max_roll_angle_deg =
      ReadPropertyOrDefault(adapter, "guidance/max-roll-angle-deg", profile->max_roll_angle_deg);
  profile->min_throttle =
      ReadPropertyOrDefault(adapter, "guidance/min-throttle", profile->min_throttle);
  profile->max_throttle =
      ReadPropertyOrDefault(adapter, "guidance/max-throttle", profile->max_throttle);
  profile->ceiling_m =
      ReadPropertyOrDefault(adapter, "guidance/ceiling-m", profile->ceiling_m);
  if (HasProperty(adapter, "guidance/speed-energy-priority")) {
    profile->speed_energy_priority =
        adapter.GetProperty("guidance/speed-energy-priority") > 0.5;
  }

  profile->rotation_ramp_sec =
      ReadPropertyOrDefault(adapter, "guidance/rotation-ramp-sec", profile->rotation_ramp_sec);
  profile->rotation_max_elevator = ReadPropertyOrDefault(
      adapter, "guidance/rotation-max-elevator", profile->rotation_max_elevator);
  profile->rotation_climb_rate_mps = ReadPropertyOrDefault(
      adapter, "guidance/rotation-climb-rate-mps", profile->rotation_climb_rate_mps);

  profile->landing_approach_speed_mps = ReadPropertyOrDefault(
      adapter, "guidance/landing-approach-speed-mps", profile->landing_approach_speed_mps);
  profile->landing_high_descent_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-high-descent-agl-m", profile->landing_high_descent_agl_m);
  profile->landing_staging_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-staging-agl-m", profile->landing_staging_agl_m);
  profile->landing_pattern_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-pattern-agl-m", profile->landing_pattern_agl_m);
  if (HasProperty(adapter, "guidance/landing-high-descent-orbit")) {
    profile->landing_high_descent_orbit =
        adapter.GetProperty("guidance/landing-high-descent-orbit") > 0.5;
  }
  profile->landing_descent_throttle = ReadPropertyOrDefault(
      adapter, "guidance/landing-descent-throttle", profile->landing_descent_throttle);
  profile->landing_approach_flaps_norm = ReadPropertyOrDefault(
      adapter, "guidance/landing-approach-flaps-norm", profile->landing_approach_flaps_norm);
  profile->landing_final_flaps_norm = ReadPropertyOrDefault(
      adapter, "guidance/landing-final-flaps-norm", profile->landing_final_flaps_norm);
  profile->landing_final_throttle_cap = ReadPropertyOrDefault(
      adapter, "guidance/landing-final-throttle-cap", profile->landing_final_throttle_cap);
  profile->landing_flare_initial_elevator = ReadPropertyOrDefault(
      adapter, "guidance/landing-flare-initial-elevator", profile->landing_flare_initial_elevator);
  if (HasProperty(adapter, "guidance/landing-heavy-flare")) {
    profile->landing_heavy_flare =
        adapter.GetProperty("guidance/landing-heavy-flare") > 0.5;
  }
  profile->landing_touchdown_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-touchdown-agl-m", profile->landing_touchdown_agl_m);
}

void ApplyXmlRollLimitOverride(adapter::JsbsimAdapter& adapter, AircraftControlProfile* profile) {
  if (!profile || !HasProperty(adapter, adapter::property::kGuidanceRollAngleLimit)) return;

  constexpr double kSustainedTurnFactor = 0.7;
  constexpr double kAdapterFallbackRollLimitRad = 0.785;
  const double roll_lim_rad = adapter.GetProperty(adapter::property::kGuidanceRollAngleLimit);

  const bool is_fbw = profile->has_fbw_override || profile->has_roll_rate_command;
  const bool is_heavy = profile->engine_count >= 4 ||
      (profile->pitch_moi_lbsft2 > 0.0 && std::log10(profile->pitch_moi_lbsft2) > 7.0);
  if (is_fbw || is_heavy ||
      std::abs(roll_lim_rad - kAdapterFallbackRollLimitRad) < 1.0e-6) {
    return;
  }

  profile->max_roll_angle_deg = roll_lim_rad * 180.0 / M_PI * kSustainedTurnFactor;
}

// Set rotation/takeoff parameters based on pitch moment of inertia.
// Heavier aircraft have larger Iyy → need longer ramp to avoid step input,
// but do NOT reduce max elevator (they need MORE authority to rotate).
// Thresholds on log10(Iyy):
//   >7  heavy transport  (B747 3.3e7, MD11 3.8e7, XB-70 1.6e7, Concorde 1.9e7)
//   >6  medium transport (737 1.5e6, C130 2.4e6)
//   ≤6  light aircraft   (c172x 1.3e3, fighters ~5e4)
void ApplyRotationDefaults(AircraftControlProfile* profile) {
  if (!profile || profile->pitch_moi_lbsft2 <= 0.0) return;

  double log_moi = std::log10(profile->pitch_moi_lbsft2);
  // landing_heavy_flare defaults to false.  Only aircraft XML that
  // explicitly sets guidance/landing-heavy-flare=1 (currently B747)
  // will use the transport bounce/float flare law.  Using log10(Iyy)>7
  // as a proxy also catches MD11 (Iyy=3.8e7) whose different planform
  // and flap geometry do not need the B747-tuned flare parameters.
  if (log_moi > 7.0) {
    profile->rotation_ramp_sec = 6.0;
    profile->rotation_climb_rate_mps = 3.0;
  } else if (log_moi > 6.0) {
    profile->rotation_ramp_sec = 4.0;
    profile->rotation_climb_rate_mps = 4.0;
  }
  // rotation_max_elevator stays at 0.30 for all — heavy aircraft need
  // full authority to rotate.  The longer ramp prevents step-input departure.
}

}  // namespace

Autopilot::Autopilot(adapter::JsbsimAdapter& adapter) : adapter_(adapter) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();

  // Tier 1: XML property probing — detect aircraft capabilities from
  // the JSBSim property tree.  No model-name hardcoding.
  control_profile_.has_own_autopilot = pm->GetNode(adapter::property::kApHeadingHold) != nullptr;
  control_profile_.has_generic_autopilot = pm->GetNode(adapter::property::kApRollOn) != nullptr;
  control_profile_.has_fbw_override = pm->GetNode(adapter::property::kApFbwOverride) != nullptr;
  control_profile_.has_roll_rate_command = pm->GetNode(adapter::property::kRollRateCommand) != nullptr ||
                                           pm->GetNode(adapter::property::kRollRateCmd) != nullptr;
  control_profile_.has_aileron_command = pm->GetNode(adapter::property::kAileronCmd) != nullptr;

  // Engine count and indexed throttle detection
  const auto propulsion = adapter_.GetFdmExec().GetPropulsion();
  if (propulsion) {
    control_profile_.engine_count = static_cast<int>(propulsion->GetNumEngines());
    // Multi-engine aircraft need per-engine throttle commands.
    control_profile_.indexed_throttle = control_profile_.engine_count > 1;
  }

  // Mixture detection (piston aircraft only — require both mixture and magneto,
  // since JSBSim creates indexed fcs/mixture-cmd-norm[n] for all engine types,
  // and propulsion/magneto_cmd only exists for FGPiston engines).
  control_profile_.has_mixture = pm->GetNode("fcs/mixture-cmd-norm") != nullptr &&
                                 pm->GetNode(adapter::property::kMagnetoCmd) != nullptr;

  // Pitch moment of inertia — determines rotation response and ramp scaling.
  // JSBSim stores this as inertia/iyy-slugs_ft2 (XML unit="SLUG*FT2").
  auto* iyy_node = pm->GetNode("inertia/iyy-slugs_ft2");
  if (iyy_node) {
    control_profile_.pitch_moi_lbsft2 = iyy_node->getDoubleValue();
  }

  // --- Physics-derived performance baseline ---
  // V_stall and wing loading are computed from the aircraft's actual
  // weight, wing area, and high-lift capability — not from hardcoded
  // category tables.  These drive the speed envelope and ceiling.
  {
    double weight_lbs = ReadPropertyOrDefault(adapter_, "inertia/weight-lbs", 0.0);
    double wing_ft2 = ReadPropertyOrDefault(adapter_, "metrics/Sw-sqft", 0.0);
    double span_ft = ReadPropertyOrDefault(adapter_, "metrics/bw-ft", 0.0);

    if (weight_lbs > 1.0 && wing_ft2 > 1.0) {
      control_profile_.wing_loading_lbs_ft2 = weight_lbs / wing_ft2;

      // Establish a deterministic safe baseline before current atmosphere state
      // is consumed below. Runtime updates replace it with the current TAS value.
      bool is_turboprop = false;
      if (propulsion) {
        int n = propulsion->GetNumEngines();
        if (n > 0 && propulsion->GetEngine(0)->GetType() == JSBSim::FGEngine::etTurboprop) {
          is_turboprop = true;
        }
      }
      PerformanceDerivationInputs perf_inputs;
      perf_inputs.weight_lbs = weight_lbs;
      perf_inputs.wing_area_ft2 = wing_ft2;
      perf_inputs.wingspan_ft = span_ft;
      perf_inputs.is_turboprop = is_turboprop;
      perf_inputs.has_cl_max_override = false;
      perf_inputs.cl_max_override = 0.0;
      const PerformanceDerivationResult perf =
          DeriveStallAndWingLoading(perf_inputs, kStandardSeaLevelDensitySlugsFt3);
      control_profile_.v_stall_mps = perf.v_stall_ftps * 0.3048;

      // Thrust-to-weight ratio: read from property tree.
      // EngineManager publishes this during construction via InitRunning()
      // measurement.  Falls back to 0.0 if EngineManager hasn't set it yet.
      control_profile_.thrust_to_weight =
          ReadPropertyOrDefault(adapter_, "guidance/thrust-to-weight", 0.0);
    }
  }

  ApplyRotationDefaults(&control_profile_);

  // FBW subtype detection: f16 has roll-rate PID
  if (pm->GetNode("fcs/aileron-act") != nullptr &&
      pm->GetNode(adapter::property::kRollCmd) != nullptr) {
    control_profile_.fbw_subtype = FbwSubtype::kRateIntegratorActuator;
  } else if (control_profile_.has_roll_rate_command) {
    control_profile_.fbw_subtype = FbwSubtype::kRollRatePid;
  }

  // Yaw input property detection
  if (pm->GetNode(adapter::property::kRudderCmd) != nullptr) {
    control_profile_.yaw_input_property = adapter::property::kRudderCmd;
  } else if (pm->GetNode("fcs/rudder-pedal-norm") != nullptr) {
    control_profile_.yaw_input_property = "fcs/rudder-pedal-norm";
  }

  // Pitch interface detection
  if (control_profile_.has_own_autopilot || control_profile_.has_generic_autopilot) {
    control_profile_.pitch_interface = PitchControlInterface::kNativeAutopilot;
  } else if (pm->GetNode("fcs/pitch-rate-cmd") != nullptr) {
    control_profile_.pitch_interface = PitchControlInterface::kFbwScheduled;
  }

  // Lateral interface selection.  FBW takes priority over native autopilot
  // because some FBW aircraft (f16) have ap/heading_hold from their flight
  // control system but need kFbwRateCommand for correct roll-rate handling.
  if (control_profile_.has_fbw_override || control_profile_.has_roll_rate_command) {
    control_profile_.lateral_interface = LateralControlInterface::kFbwRateCommand;
  } else if (control_profile_.has_own_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kOwnAutopilot;
  } else if (control_profile_.has_generic_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kGenericAutopilotBridge;
  } else {
    control_profile_.lateral_interface = LateralControlInterface::kDirectSurface;
  }

  // A single leaked ap/autopilot-roll-on from a shared system file
  // (Autopilot.xml) is not enough evidence for kGenericAutopilotBridge
  // when the aircraft has no own autopilot, no FBW, and fewer than 4
  // engines (indicating no real native AP system — hits OV10).
  if (control_profile_.lateral_interface == LateralControlInterface::kGenericAutopilotBridge &&
      !control_profile_.has_own_autopilot &&
      !control_profile_.has_fbw_override &&
      control_profile_.engine_count < 4) {
    control_profile_.lateral_interface = LateralControlInterface::kDirectSurface;
  }

  use_cpp_ap_ =
      control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot &&
      control_profile_.lateral_interface != LateralControlInterface::kGenericAutopilotBridge;
  ApplyEnergyDefaults(&control_profile_);
  RefreshDynamicSpeedEnvelope(adapter_, &control_profile_, true);
  ApplyXmlRollLimitOverride(adapter_, &control_profile_);
  ApplyXmlProfileOverrides(adapter_, &control_profile_);
}

void Autopilot::SetHeadingTargetRad(double heading_rad) {
  target_heading_rad_ = heading_rad;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kGuidanceHeadingRad, heading_rad);
    adapter_.SetProperty(adapter::property::kApHeadingSetpoint, RadToDeg360(heading_rad));
  }
}

void Autopilot::SetHeadingHold(bool on) {
  heading_hold_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApHeadingHold, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetHeadingSourceIsWaypoint(bool from_waypoint) {
  heading_src_wp_ = from_waypoint;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kGuidanceHeadingSwitch, from_waypoint ? 0.0 : 1.0);
  }
}

void Autopilot::SetAltitudeTargetM(double altitude_m) {
  target_altitude_m_ = altitude_m;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApAltitudeSetpoint, altitude_m * kMToFt);
  }
}

void Autopilot::SetAltitudeHold(bool on) { altitude_hold_ = on; }

void Autopilot::SetPitchTargetDeg(double pitch_deg) {
  target_pitch_deg_ = pitch_deg;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApPitchTarget, pitch_deg);
  }
}

void Autopilot::SetPitchHold(bool on) {
  pitch_hold_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApPitchHold, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetLateralGuidanceMode(LateralGuidanceMode mode) { lateral_guidance_mode_ = mode; }

void Autopilot::SetRollAttitudeMode(int mode) {
  roll_mode_ = mode;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApRollAttitudeMode, static_cast<double>(mode));
  }
}

void Autopilot::SetRollAutopilotOn(bool on) {
  roll_ap_on_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApRollOn, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetThrottleCmdNorm(double value) {
  adapter_.SetProperty(adapter::property::kThrottleCmd, value);
  if (!control_profile_.indexed_throttle &&
      control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot) {
    return;
  }
  const auto propulsion = adapter_.GetFdmExec().GetPropulsion();
  if (!propulsion) {
    return;
  }
  for (size_t engine = 0; engine < propulsion->GetNumEngines(); ++engine) {
    SetThrottleCmd(static_cast<int>(engine), value);
  }
}

void Autopilot::SetThrottleCmd(int engine, double value) {
  std::string prop = std::string(adapter::property::kThrottleCmd) + "[" + std::to_string(engine) + "]";
  adapter_.SetProperty(prop, value);
}

void Autopilot::SetYawDamper(bool on) {
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApYawDamper, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetSpeedTargetMps(double speed_mps) { target_speed_mps_ = speed_mps; }
void Autopilot::SetSpeedHold(bool on) { speed_hold_ = on; }

void Autopilot::ReleaseHolds() {
  heading_hold_ = false;
  altitude_hold_ = false;
  pitch_hold_ = false;
  speed_hold_ = false;
  roll_ap_on_ = false;
  roll_mode_ = 0;
  lateral_guidance_mode_ = LateralGuidanceMode::kHeading;

  adapter_.SetProperty(adapter::property::kApHeadingHold, 0.0);
  adapter_.SetProperty(adapter::property::kApAltitudeHold, 0.0);
  adapter_.SetProperty(adapter::property::kApAttitudeHold, 0.0);
  adapter_.SetProperty(adapter::property::kApPitchHold, 0.0);
  adapter_.SetProperty(adapter::property::kApRollOn, 0.0);
  adapter_.SetProperty(adapter::property::kApYawDamper, 0.0);
}

double Autopilot::GetTrueSpeedMps() const {
  return adapter_.GetProperty("velocities/vtrue-fps") * kFtToM;
}

double Autopilot::GetAngleToHeadingRad() const {
  if (use_cpp_ap_ ||
      control_profile_.lateral_interface == LateralControlInterface::kFbwRateCommand) {
    const auto& propagate = adapter_.GetPropagate();
    double current_heading = propagate.GetEuler(3);
    double target_heading = target_heading_rad_;
    if (heading_src_wp_) {
      double target_lat = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLat);
      double target_lon = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLon);
      target_heading = propagate.GetLocation().GetHeadingTo(target_lon, target_lat);
    }
    return NormalizeRad(target_heading - current_heading);
  }
  // kOwnAutopilot / kGenericApBridge: compute from current attitude instead
  // of guidance/angle-to-heading-rad. The JSBSim guidance angle depends on
  // navigation/actual-heading-rad which is only updated by an explicit nav
  // system (c172x/c310/global5000 have none), leaving the source frozen at
  // its initial value (0). The C++ target_heading_rad_ is what was actually
  // sent to the native AP via ap/heading_setpoint, so it is the correct
  // convergence reference.
  const auto& propagate = adapter_.GetPropagate();
  double current_heading = propagate.GetEuler(3);
  double target_heading = target_heading_rad_;
  if (heading_src_wp_) {
    double target_lat = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLat);
    double target_lon = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLon);
    target_heading = propagate.GetLocation().GetHeadingTo(target_lon, target_lat);
  }
  return NormalizeRad(target_heading - current_heading);
}

double Autopilot::GetAltitudeAGLM() const {
  return adapter_.GetProperty(adapter::property::kHaglFt) * kFtToM;
}

double Autopilot::GetAltitudeASLM() const {
  return adapter_.GetPropagate().GetLocation().GetGeodAltitude() * kFtToM;
}

void Autopilot::Update(double /*dt_sec*/) {
  RefreshDynamicSpeedEnvelope(adapter_, &control_profile_, false);
  const auto& propagate = adapter_.GetPropagate();

  // kOwnAutopilot: delegate lateral to XML autopilot, but use C++ pitch
  // control for altitude hold (native AP lacks speed protection and can
  // stall the aircraft at high altitude — observed c310 at 2000m: 25° pitch,
  // 70kts, sinking).
  if (control_profile_.lateral_interface == LateralControlInterface::kOwnAutopilot) {
    UpdateOwnAutopilot();
    UpdateDirectHeadingLateral();
    UpdatePitchChannel();
    UpdateEnergyManagement();
    double r = propagate.GetPQR(3);
    ApplyYawDamping(r);
    return;
  }

  // Lateral dispatch by profile — each profile handles all guidance modes uniformly.
  switch (control_profile_.lateral_interface) {
    case LateralControlInterface::kFbwRateCommand:
      if (control_profile_.fbw_subtype == FbwSubtype::kRateIntegratorActuator ||
          lateral_guidance_mode_ == LateralGuidanceMode::kOrbit) {
        UpdateFbwRateCommandLateral();
      } else {
        UpdateDirectHeadingLateral();
      }
      break;
    case LateralControlInterface::kGenericAutopilotBridge:
      UpdateGenericApBridge();
      UpdateRollAnglePD();
      break;
    case LateralControlInterface::kDirectSurface:
      UpdateDirectHeadingLateral();
      break;
    default:
      break;
  }

  UpdateEnergyManagement();
  UpdatePitchChannel();

  double r = propagate.GetPQR(3);
  ApplyYawDamping(r);
}

void Autopilot::UpdateOwnAutopilot() {
  if (heading_hold_) {
    ApplyNativeHeadingSetpoint();
  }
  adapter_.SetProperty(adapter::property::kApHeadingHold, heading_hold_ ? 1.0 : 0.0);
  adapter_.SetProperty(adapter::property::kApAttitudeHold, heading_hold_ ? 1.0 : 0.0);
  adapter_.SetProperty(adapter::property::kApAltitudeHold, altitude_hold_ ? 1.0 : 0.0);
}

void Autopilot::UpdateGenericApBridge() {
  if (heading_hold_) {
    ApplyNativeHeadingSetpoint();
  }
}

void Autopilot::UpdateFbwRateCommandLateral() {
  const auto& propagate = adapter_.GetPropagate();
  const double roll_rad = propagate.GetEuler(1);
  const double bank_limit_rad =
      control_profile_.max_roll_angle_deg * 0.01745329;

  double roll_rate_cmd = 0.0;
  if (heading_hold_) {
    const double heading_err = GetAngleToHeadingRad();
    roll_rate_cmd = Clamp(0.45 * heading_err, -0.60, 0.60);

    // Bank angle limiting: when current bank approaches structural limit,
    // apply opposing rate command to prevent overshoot regardless of FBW type.
    double bank_ratio = std::abs(roll_rad) / bank_limit_rad;
    if (bank_ratio > 0.80) {
      double roll_sign = (roll_rad >= 0.0) ? 1.0 : -1.0;
      double excess = std::min((bank_ratio - 0.80) / 0.20, 1.0);
      // Blend from heading-driven command to roll-recovery command.
      roll_rate_cmd = roll_rate_cmd * (1.0 - excess) - roll_sign * excess * 0.60;
    }
  }
  if (roll_ap_on_ || heading_hold_ || roll_mode_ == 0) {
    adapter_.SetProperty("fcs/aileron-cmd-norm", roll_rate_cmd);
  }
}

void Autopilot::UpdateDirectHeadingLateral() {
  const auto& propagate = adapter_.GetPropagate();
  const double roll = propagate.GetEuler(1);
  const double p = propagate.GetPQR(1);
  double v_fps = propagate.GetInertialVelocityMagnitude();
  if (v_fps < 10.0) v_fps = 10.0;

  const double speed_ratio = Clamp(kRefSpeedFps / v_fps, 1.4, 1.8);
  double target_roll = 0.0;
  if (heading_hold_) {
    const double heading_err = GetAngleToHeadingRad();
    const double heading_gain = 0.6 * speed_ratio;
    const double roll_limit =
        control_profile_.lateral_interface == LateralControlInterface::kFbwRateCommand ? 1.60
                                                                                       : 0.52;
    target_roll = Clamp(heading_gain * heading_err, -roll_limit, roll_limit);
  }

  const double roll_err = target_roll - roll;
  const double kp_roll = 2.0 * speed_ratio;
  constexpr double kRollDamping = 0.8;
  double aileron = kp_roll * roll_err - kRollDamping * p;
  aileron = Clamp(aileron, -1.0, 1.0);

  if (roll_ap_on_ || heading_hold_ || roll_mode_ == 0) {
    adapter_.SetProperty(adapter::property::kAileronCmd, aileron);
  }
}

void Autopilot::UpdateRollAnglePD() {
  const auto& propagate = adapter_.GetPropagate();
  double roll = propagate.GetEuler(1);
  double p = propagate.GetPQR(1);

  double target_roll = 0.0;
  if (heading_hold_) {
    double heading_err = GetAngleToHeadingRad();
    target_roll = 1.2 * heading_err;
    target_roll = Clamp(target_roll, -0.785, 0.785);
  }

  double roll_err = target_roll - roll;
  double aileron = 3.0 * roll_err + 0.5 * roll_int_ - 0.3 * p;
  if (std::abs(aileron) < 1.0) {
    roll_int_ += 0.005 * roll_err;
    roll_int_ = Clamp(roll_int_, -0.4, 0.4);
  }
  aileron = Clamp(aileron, -1.0, 1.0);
  adapter_.SetProperty(adapter::property::kAileronCmd, aileron);
}

void Autopilot::UpdatePitchChannel() {
  const auto& propagate = adapter_.GetPropagate();
  double pitch = propagate.GetEuler(2);
  double q = propagate.GetPQR(2);

  double target_pitch = 0.0;
  bool pitch_control_active = false;

  if (altitude_hold_) {
    pitch_control_active = true;
    double target_alt_ft = target_altitude_m_ * kMToFt;
    double current_alt_ft = propagate.GetLocation().GetGeodAltitude();
    double alt_err_ft = target_alt_ft - current_alt_ft;

    // Base pitch: altitude PD
    target_pitch = 0.0005 * alt_err_ft;

    // Speed protection: reduce climb pitch if speed is too low.
    double current_speed_mps = GetTrueSpeedMps();
    double min_speed = control_profile_.min_speed_mps;
    if (min_speed > 0.0 && current_speed_mps < min_speed * 1.15 && target_pitch > 0.0) {
      double speed_deficit = Clamp((min_speed * 1.15 - current_speed_mps) / (min_speed * 0.2), 0.0, 1.0);
      target_pitch *= (1.0 - speed_deficit);
    }

    double max_pitch_rad = control_profile_.max_pitch_command_deg * M_PI / 180.0;
    target_pitch = Clamp(target_pitch, -max_pitch_rad, max_pitch_rad);
  } else if (pitch_hold_) {
    pitch_control_active = true;
    target_pitch = target_pitch_deg_ * M_PI / 180.0;

    // Speed protection (L2): reduce pitch if speed is too low.
    // This prevents stall when thrust is insufficient to maintain speed
    // at the commanded pitch angle despite the energy management (L1)
    // commanding max throttle.  The equilibrium between L1 throttle and
    // L2 pitch relief is the aircraft's physical pitch limit at the
    // current altitude/speed combination.
    double current_speed_mps = GetTrueSpeedMps();
    double min_speed = control_profile_.min_speed_mps;
    if (min_speed > 0.0 && current_speed_mps < min_speed * 1.10 && target_pitch > 0.0) {
      double speed_deficit = Clamp(
          (min_speed * 1.10 - current_speed_mps) / (min_speed * 0.15), 0.0, 1.0);
      target_pitch *= (1.0 - speed_deficit);
    }
  }

  if (pitch_control_active) {
    double pitch_err = target_pitch - pitch;
    double elevator = -(2.0 * pitch_err - 0.2 * q);
    elevator = Clamp(elevator, -1.0, 1.0);
    if (control_profile_.fbw_subtype != FbwSubtype::kNone) {
      adapter_.SetProperty("fcs/pitch-trim-cmd-norm", elevator);
    } else {
      adapter_.SetProperty(adapter::property::kElevatorCmd, elevator);
    }
  }
}

void Autopilot::UpdateEnergyManagement() {
  if (!altitude_hold_ && !speed_hold_ && !pitch_hold_) return;

  const double current_speed_mps = GetTrueSpeedMps();

  // ref_speed normalizes the speed error term.  Use profile ref_speed if
  // available, otherwise cruise_speed, otherwise current speed (last resort).
  const double ref_speed = control_profile_.ref_speed_mps > 0.0
                               ? control_profile_.ref_speed_mps
                           : control_profile_.cruise_speed_mps > 0.0
                               ? control_profile_.cruise_speed_mps
                               : current_speed_mps;

  // Altitude error (potential energy proxy)
  const auto& propagate = adapter_.GetPropagate();
  const double current_alt_m = propagate.GetLocation().GetGeodAltitude() * kFtToM;
  double alt_err_m = altitude_hold_ ? (target_altitude_m_ - current_alt_m) : 0.0;

  // Speed error (kinetic energy proxy), clamped to envelope.
  double target_spd = target_speed_mps_;
  if (control_profile_.min_speed_mps > 0.0 && target_spd < control_profile_.min_speed_mps) {
    target_spd = control_profile_.min_speed_mps;
  }
  double speed_err_mps = speed_hold_ ? (target_spd - current_speed_mps) : 0.0;

  // Combined energy error: throttle manages total energy.
  double energy_err = alt_err_m / 500.0;
  if (ref_speed > 1.0) {
    energy_err += speed_err_mps / ref_speed * 0.3;
  }

  // ── Pitch hold energy bias ─────────────────────────────────────────
  // Proportional to pitch target: higher pitch → more drag → need more
  // thrust.  Blends into the existing energy formula so all speed /
  // overspeed protections remain active (no early return).
  //   +5° → +0.08  (gentle boost)
  //   +15°→ +0.24  (strong boost)
  //   +25°→ +0.40  (near full throttle, matches max energy_err clamp)
  // Negative pitch relies on normal energy management + overspeed guard.
  if (pitch_hold_ && !altitude_hold_ && target_pitch_deg_ > 0.0) {
    constexpr double kPitchBiasGain = 0.40;
    constexpr double kPitchBiasRefDeg = 25.0;
    energy_err += kPitchBiasGain * (target_pitch_deg_ / kPitchBiasRefDeg);
  }

  // Speed protection: if below min_speed, override energy demand to recover speed.
  const double min_speed = control_profile_.min_speed_mps;
  if (min_speed > 0.0 && current_speed_mps < min_speed * 1.1) {
    const double urgency = Clamp((min_speed * 1.1 - current_speed_mps) / (min_speed * 0.3), 0.0, 1.0);
    if (control_profile_.speed_energy_priority) {
      energy_err += urgency * 0.5;
    } else {
      energy_err += urgency * 0.25;
    }
  }

  // Speed protection: if above max_speed, reduce throttle aggressively.
  const double max_speed = control_profile_.max_speed_mps;
  if (max_speed > 0.0 && current_speed_mps > max_speed * 0.95) {
    const double excess = Clamp((current_speed_mps - max_speed * 0.95) / (max_speed * 0.10), 0.0, 1.0);
    energy_err -= excess * 0.5;
  }

  double throttle = Clamp(0.70 + Clamp(energy_err, -0.40, 0.40),
                          control_profile_.min_throttle,
                          control_profile_.max_throttle);
  SetThrottleCmdNorm(throttle);
}

void Autopilot::ApplyNativeHeadingSetpoint() {
  double heading_rad = target_heading_rad_;
  if (heading_src_wp_) {
    const auto& propagate = adapter_.GetPropagate();
    double target_lat = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLat);
    double target_lon = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLon);
    heading_rad = propagate.GetLocation().GetHeadingTo(target_lon, target_lat);
  }
  adapter_.SetProperty(adapter::property::kApHeadingSetpoint, RadToDeg360(heading_rad));
}

void Autopilot::ApplyYawDamping(double yaw_rate_rad_sec) {
  const std::string yaw_property = control_profile_.yaw_input_property.empty()
                                       ? std::string(adapter::property::kRudderCmd)
                                       : control_profile_.yaw_input_property;

  const auto& propagate = adapter_.GetPropagate();
  const double roll_rad = propagate.GetEuler(1);

  // Turn coordination: for a banked turn, the aircraft needs rudder to
  // produce the yaw rate for a coordinated turn.  Without this, the yaw
  // damper fights the sustained yaw rate and the turn radius balloons.
  double rudder = 0.0;
  if (std::abs(roll_rad) > 0.05) {
    // Coordinated turn requires r ∝ sin(φ).  Apply rudder proportional
    // to bank angle to assist the turn, then damp residual oscillations.
    double vt_fps = propagate.GetInertialVelocityMagnitude();
    if (vt_fps < 10.0) vt_fps = 10.0;
    double vt_mps = vt_fps * 0.3048;
    double coord_yaw_rate = 9.80665 * std::sin(roll_rad) / vt_mps;
    double yaw_err = yaw_rate_rad_sec - coord_yaw_rate;
    rudder = Clamp(-0.3 * yaw_err, -1.0, 1.0);
  } else {
    // Wings level: pure yaw damping to suppress Dutch roll.
    rudder = Clamp(-0.15 * yaw_rate_rad_sec, -1.0, 1.0);
  }
  adapter_.SetProperty(yaw_property, rudder);
}

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq
