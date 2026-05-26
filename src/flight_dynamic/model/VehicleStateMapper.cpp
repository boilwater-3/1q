#include "flight_dynamic/model/VehicleStateMapper.h"

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
  s.altitude_agl_m = fdm_exec.GetPropertyValue("position/h-agl-ft") * kFtToM;

  // Body velocities (UVW = body frame, fps)
  const auto& uvw = propagate.GetUVW();
  s.u_mps = uvw(1) * kFtToM;
  s.v_mps = uvw(2) * kFtToM;
  s.w_mps = uvw(3) * kFtToM;

  // Inertial speed
  s.v_inertial_mps = propagate.GetInertialVelocityMagnitude() * kFtToM;

  // Airspeeds
  s.vc_mps = fdm_exec.GetPropertyValue("velocities/vc-fps") * kFtToM;
  s.vtrue_mps = fdm_exec.GetPropertyValue("velocities/vtrue-fps") * kFtToM;
  s.mach = fdm_exec.GetPropertyValue("velocities/mach");

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
  s.weight_lbs = fdm_exec.GetPropertyValue("inertia/weight-lbs");
  s.mass_kg = fdm_exec.GetPropertyValue("inertia/mass-slugs") * kSlugToKg;

  // Atmosphere
  s.rho_kg_m3 = fdm_exec.GetPropertyValue("atmosphere/rho-slugs_ft3") * kSlugToKg / (kFtToM * kFtToM * kFtToM);
  s.qbar_pa = fdm_exec.GetPropertyValue("aero/qbar-psf") * kPsfToPa;

  return s;
}

void VehicleStateMapper::ApplyInitialConditions(
    JSBSim::FGFDMExec& fdm_exec,
    const coordinate::ExternalKinematics& kinematics) {
  auto ic = fdm_exec.GetIC();
  if (kinematics.position_frame == coordinate::PositionFrame::kLla) {
    ic->SetLatitudeDegIC(kinematics.position_lla_deg_m.latitude_deg);
    ic->SetLongitudeDegIC(kinematics.position_lla_deg_m.longitude_deg);
    ic->SetAltitudeASLFtIC(kinematics.position_lla_deg_m.altitude_m * kMToFt);
  }
  ic->SetUBodyFpsIC(kinematics.velocity_mps.x_mps * kMToFt);
  ic->SetVBodyFpsIC(kinematics.velocity_mps.y_mps * kMToFt);
  ic->SetWBodyFpsIC(kinematics.velocity_mps.z_mps * kMToFt);
  ic->SetPhiRadIC(kinematics.attitude_deg.roll_deg * kDegToRad);
  ic->SetThetaRadIC(kinematics.attitude_deg.pitch_deg * kDegToRad);
  ic->SetPsiRadIC(kinematics.attitude_deg.yaw_deg * kDegToRad);
}

}  // namespace model
}  // namespace flight_dynamic
}  // namespace oneq
