#ifndef ONEQ_FLIGHT_DYNAMIC_ADAPTER_PROPERTYNAMES_H_
#define ONEQ_FLIGHT_DYNAMIC_ADAPTER_PROPERTYNAMES_H_

namespace oneq {
namespace flight_dynamic {
namespace adapter {
namespace property {

// FCS commands
constexpr const char* kAileronCmd = "fcs/aileron-cmd-norm";
constexpr const char* kElevatorCmd = "fcs/elevator-cmd-norm";
constexpr const char* kRudderCmd = "fcs/rudder-cmd-norm";
constexpr const char* kThrottleCmd = "fcs/throttle-cmd-norm";
constexpr const char* kPitchTrimCmd = "fcs/pitch-trim-cmd-norm";
constexpr const char* kRollTrimCmd = "fcs/roll-trim-cmd-norm";
constexpr const char* kYawTrimCmd = "fcs/yaw-trim-cmd-norm";

// FCS state
constexpr const char* kPitchRateIntegrator = "fcs/pitch-rate-integrator";
constexpr const char* kRollRateIntegrator = "fcs/roll-rate-integrator";
constexpr const char* kYawRateIntegrator = "fcs/yaw-rate-integrator";
constexpr const char* kElevatorPosNorm = "fcs/elevator-pos-norm";
constexpr const char* kElevatorPosRad = "fcs/elevator-pos-rad";
constexpr const char* kLeftAileronPosRad = "fcs/left-aileron-pos-rad";
constexpr const char* kRightAileronPosRad = "fcs/right-aileron-pos-rad";
constexpr const char* kRudderPosRad = "fcs/rudder-pos-rad";
constexpr const char* kThrottlePosNorm = "fcs/throttle-pos-norm";
constexpr const char* kRollRateCmd = "fcs/roll-rate-cmd";
constexpr const char* kRollRateError = "fcs/roll-rate-error";
constexpr const char* kRollRateCommand = "fcs/roll-rate-command";
constexpr const char* kRollCmd = "fcs/roll-cmd";
constexpr const char* kAileronAct = "fcs/aileron-act";

// Gear
constexpr const char* kGearCmdNorm = "gear/gear-cmd-norm";
constexpr const char* kGearPosNorm = "gear/gear-pos-norm";

// Autopilot
constexpr const char* kApHeadingHold = "ap/heading_hold";
constexpr const char* kApAltitudeHold = "ap/altitude_hold";
constexpr const char* kApAttitudeHold = "ap/attitude_hold";
constexpr const char* kApRollOn = "ap/autopilot-roll-on";
constexpr const char* kApHeadingSetpoint = "ap/heading_setpoint";
constexpr const char* kApAltitudeSetpoint = "ap/altitude_setpoint";
constexpr const char* kApPitchTarget = "ap/pitch-target-deg";
constexpr const char* kApPitchHold = "ap/pitch-hold";
constexpr const char* kApRollAttitudeMode = "ap/roll-attitude-mode";
constexpr const char* kApYawDamper = "ap/yaw_damper";
constexpr const char* kApFbwOverride = "fcs/fbw-override";

// Guidance
constexpr const char* kGuidanceHeadingRad = "guidance/specified-heading-rad";
constexpr const char* kGuidanceAngleToHeading = "guidance/angle-to-heading-rad";
constexpr const char* kGuidanceHeadingSwitch = "guidance/heading-selector-switch";
constexpr const char* kGuidanceRollAngleLimit = "guidance/roll-angle-limit";
constexpr const char* kGuidanceRollRateLimit = "guidance/roll-rate-limit";
constexpr const char* kGuidanceTargetWpLat = "guidance/target_wp_latitude_rad";
constexpr const char* kGuidanceTargetWpLon = "guidance/target_wp_longitude_rad";

// Simulation
constexpr const char* kIntegratorRateRot = "simulation/integrator/rate/rotational";
constexpr const char* kIntegratorRateTrans = "simulation/integrator/rate/translational";
constexpr const char* kIntegratorPosRot = "simulation/integrator/position/rotational";
constexpr const char* kIntegratorPosTrans = "simulation/integrator/position/translational";
constexpr const char* kGravityModel = "simulation/gravity-model";

// Velocities
constexpr const char* kVelU = "velocities/u-fps";
constexpr const char* kVelV = "velocities/v-fps";
constexpr const char* kVelW = "velocities/w-fps";
constexpr const char* kVelNorth = "velocities/v-north-fps";
constexpr const char* kVelEast = "velocities/v-east-fps";
constexpr const char* kVelDown = "velocities/v-down-fps";
constexpr const char* kVelPAero = "velocities/p-aero-rad_sec";
constexpr const char* kVcFps = "velocities/vc-fps";
constexpr const char* kVtrueFps = "velocities/vtrue-fps";
constexpr const char* kMach = "velocities/mach";

// Position
constexpr const char* kLatGcDeg = "position/lat-gc-deg";
constexpr const char* kLongGcDeg = "position/long-gc-deg";
constexpr const char* kHSlFt = "position/h-sl-ft";
constexpr const char* kHaglFt = "position/h-agl-ft";

// Inertia
constexpr const char* kWeightLbs = "inertia/weight-lbs";
constexpr const char* kMassSlugs = "inertia/mass-slugs";

// Aero
constexpr const char* kAlphaDeg = "aero/alpha-deg";
constexpr const char* kQbarPsf = "aero/qbar-psf";
constexpr const char* kRhoSlugsFt3 = "atmosphere/rho-slugs_ft3";

// Propulsion
constexpr const char* kEngineThrustLbs = "propulsion/engine/thrust-lbs";

// Flight path
constexpr const char* kGammaRad = "flight-path/gamma-rad";

}  // namespace property
}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_ADAPTER_PROPERTYNAMES_H_
