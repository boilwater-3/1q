#ifndef ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATE_H_
#define ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATE_H_

#include "1q/coordinate/types.h"

namespace oneq {
namespace flight_dynamic {
namespace model {

struct VehicleState {
  double sim_time_sec = 0.0;

  // WGS84 position
  double latitude_rad = 0.0;
  double longitude_rad = 0.0;
  double altitude_geod_m = 0.0;
  double altitude_agl_m = 0.0;

  // Velocity (body frame, m/s)
  double u_mps = 0.0;
  double v_mps = 0.0;
  double w_mps = 0.0;

  // Velocity (inertial magnitude, m/s)
  double v_inertial_mps = 0.0;

  // Airspeed
  double vc_mps = 0.0;
  double vtrue_mps = 0.0;
  double mach = 0.0;

  // Attitude (Euler, rad)
  double phi_rad = 0.0;
  double theta_rad = 0.0;
  double psi_rad = 0.0;

  // Body angular rates (rad/s)
  double p_rad_s = 0.0;
  double q_rad_s = 0.0;
  double r_rad_s = 0.0;

  // Body accelerations (m/s^2)
  double ax_mps2 = 0.0;
  double ay_mps2 = 0.0;
  double az_mps2 = 0.0;

  // Angular accelerations (rad/s^2)
  double pdot_rad_s2 = 0.0;
  double qdot_rad_s2 = 0.0;
  double rdot_rad_s2 = 0.0;

  // Mass properties
  double mass_kg = 0.0;
  double weight_lbs = 0.0;

  // Atmosphere
  double rho_kg_m3 = 0.0;
  double qbar_pa = 0.0;
};

}  // namespace model
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATE_H_
