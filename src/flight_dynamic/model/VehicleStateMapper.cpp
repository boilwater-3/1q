#include "flight_dynamic/model/VehicleStateMapper.h"

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropagate.h"
#include "models/FGAccelerations.h"
#include "models/FGAuxiliary.h"
#include "models/FGMassBalance.h"
#include "math/FGLocation.h"

namespace oneq {
namespace flight_dynamic {
namespace model {

VehicleState VehicleStateMapper::Map(
    const JSBSim::FGPropagate& propagate,
    const JSBSim::FGAccelerations& accelerations,
    JSBSim::FGFDMExec& fdm_exec,
    double sim_time_sec) {
  VehicleState s;
  s.sim_time_sec = sim_time_sec;

  // Position
  const auto& loc = propagate.GetLocation();
  s.latitude_rad = loc.GetLatitude();
  s.longitude_rad = loc.GetLongitude();
  s.altitude_geod_m = loc.GetGeodAltitude() * kFtToM;
  s.altitude_agl_m = fdm_exec.GetPropertyValue(adapter::property::kHaglFt) * kFtToM;

  // Body velocities (UVW = body frame, fps)
  const auto& uvw = propagate.GetUVW();
  s.u_mps = uvw(1) * kFtToM;
  s.v_mps = uvw(2) * kFtToM;
  s.w_mps = uvw(3) * kFtToM;

  // Inertial speed
  s.v_inertial_mps = propagate.GetInertialVelocityMagnitude() * kFtToM;

  // Airspeeds
  s.vc_mps = fdm_exec.GetPropertyValue(adapter::property::kVcFps) * kFtToM;
  s.vtrue_mps = fdm_exec.GetPropertyValue(adapter::property::kVtrueFps) * kFtToM;
  s.mach = fdm_exec.GetPropertyValue(adapter::property::kMach);

  // Attitude (Euler, rad)
  const auto euler = propagate.GetEuler();
  s.phi_rad = euler(1);
  s.theta_rad = euler(2);
  s.psi_rad = euler(3);

  // Body rates (rad/s)
  const auto& pqr = propagate.GetPQR();
  s.p_rad_s = pqr(1);
  s.q_rad_s = pqr(2);
  s.r_rad_s = pqr(3);

  // Body accelerations (m/s^2)
  const auto& accel = accelerations.GetBodyAccel();
  s.ax_mps2 = accel(1) * kFtToM;
  s.ay_mps2 = accel(2) * kFtToM;
  s.az_mps2 = accel(3) * kFtToM;

  // Angular accelerations (rad/s^2)
  const auto& pqr_dot = accelerations.GetPQRdot();
  s.pdot_rad_s2 = pqr_dot(1);
  s.qdot_rad_s2 = pqr_dot(2);
  s.rdot_rad_s2 = pqr_dot(3);

  // Mass properties
  s.weight_lbs = fdm_exec.GetPropertyValue(adapter::property::kWeightLbs);
  s.mass_kg = fdm_exec.GetPropertyValue(adapter::property::kMassSlugs) * kSlugToKg;

  // Atmosphere
  s.rho_kg_m3 = fdm_exec.GetPropertyValue(adapter::property::kRhoSlugsFt3) * kSlugToKg / (kFtToM * kFtToM * kFtToM);
  s.qbar_pa = fdm_exec.GetPropertyValue(adapter::property::kQbarPsf) * kPsfToPa;

  return s;
}

void VehicleStateMapper::ApplyInitialConditions(
    JSBSim::FGFDMExec& fdm_exec,
    const coordinate::ExternalKinematics& kinematics,
    config::InitialVelocityFrame velocity_frame) {
  auto ic = fdm_exec.GetIC();
  coordinate::LlaPositionDegM initial_lla;
  bool has_initial_lla = false;
  if (kinematics.position_frame == coordinate::PositionFrame::kLla) {
    initial_lla = kinematics.position_lla_deg_m;
    has_initial_lla = true;
  } else if (kinematics.position_frame == coordinate::PositionFrame::kEcef) {
    has_initial_lla = coordinate::TryEcefToLla(kinematics.position_ecef_m, &initial_lla);
  }
  if (has_initial_lla) {
    ic->SetLatitudeDegIC(initial_lla.latitude_deg);
    ic->SetLongitudeDegIC(initial_lla.longitude_deg);
    // Zero altitude means "ground start": let reset00.xml's AGL value
    // (loaded earlier by JsbsimAdapter) place the gear at runway level.
    // Non-zero altitude means air-start or explicit altitude.
    if (initial_lla.altitude_m != 0.0) {
      ic->SetAltitudeASLFtIC(initial_lla.altitude_m * kMToFt);
    }
  }

  bool velocity_applied = false;
  if (velocity_frame == config::InitialVelocityFrame::kEcef && has_initial_lla) {
    coordinate::NedVelocityMps ned_velocity;
    if (coordinate::TryEcefToNedVelocity(kinematics.velocity_mps, initial_lla, &ned_velocity)) {
      ic->SetVNorthFpsIC(ned_velocity.north_mps * kMToFt);
      ic->SetVEastFpsIC(ned_velocity.east_mps * kMToFt);
      ic->SetVDownFpsIC(ned_velocity.down_mps * kMToFt);
      velocity_applied = true;
    }
  }
  if (!velocity_applied) {
    ic->SetUBodyFpsIC(kinematics.velocity_mps.x_mps * kMToFt);
    ic->SetVBodyFpsIC(kinematics.velocity_mps.y_mps * kMToFt);
    ic->SetWBodyFpsIC(kinematics.velocity_mps.z_mps * kMToFt);
  }

  ic->SetPhiRadIC(kinematics.attitude_deg.roll_deg * kDegToRad);
  ic->SetThetaRadIC(kinematics.attitude_deg.pitch_deg * kDegToRad);
  ic->SetPsiRadIC(kinematics.attitude_deg.yaw_deg * kDegToRad);
}

}  // namespace model
}  // namespace flight_dynamic
}  // namespace oneq
