#ifndef ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATEMAPPER_H_
#define ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATEMAPPER_H_

#include "1q/flight_dynamic/model/VehicleState.h"

namespace JSBSim {
class FGPropagate;
class FGAccelerations;
class FGFDMExec;
}  // namespace JSBSim

namespace oneq {
namespace flight_dynamic {
namespace config {
enum class InitialVelocityFrame;
}
namespace model {

class VehicleStateMapper {
 public:
  static VehicleState Map(const JSBSim::FGPropagate& propagate,
                          const JSBSim::FGAccelerations& accelerations, JSBSim::FGFDMExec& fdm_exec,
                          double sim_time_sec);

  static void ApplyInitialConditions(JSBSim::FGFDMExec& fdm_exec,
                                     const coordinate::ExternalKinematics& kinematics,
                                     config::InitialVelocityFrame velocity_frame);

 private:
  static constexpr double kFtToM = 0.3048;
  static constexpr double kMToFt = 1.0 / kFtToM;
  static constexpr double kKnotsToMps = 0.514444;
  static constexpr double kSlugToKg = 14.5939;
  static constexpr double kLbfToN = 4.44822;
  static constexpr double kPsfToPa = 47.8803;
  static constexpr double kRadToDeg = 57.2957795;
  static constexpr double kDegToRad = 1.0 / kRadToDeg;
};

}  // namespace model
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATEMAPPER_H_
