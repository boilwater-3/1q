#ifndef ONEQ_FLIGHT_DYNAMIC_PROPULSION_ENGINEMANAGER_H_
#define ONEQ_FLIGHT_DYNAMIC_PROPULSION_ENGINEMANAGER_H_

#include <string>

namespace JSBSim {
class FGFDMExec;
}

namespace oneq {
namespace flight_dynamic {

namespace adapter {
class JsbsimAdapter;
}

namespace propulsion {

enum class EngineType {
  kPiston,
  kTurbine,
  kTurboprop,
  kRocket,
  kElectric,
  kUnknown,
};

class EngineManager {
 public:
  explicit EngineManager(adapter::JsbsimAdapter& adapter);

  EngineType GetType() const { return type_; }
  int GetCount() const { return count_; }

  bool HasMagneto() const { return has_magneto_; }
  bool HasStarter() const { return has_starter_; }
  bool HasMixture() const { return has_mixture_; }
  bool HasGroundContact() const { return has_wow_; }

  // Performance parameters derived from aircraft physics.
  double GetRotationSpeedKts() const;
  double GetClimbPitchDeg() const;
  double GetDefaultApproachSpeedMps() const;

  // Total static thrust from all engines (lbs).  Reads JSBSim property
  // propulsion/engine[n]/thrust-lbs for each engine.
  double GetTotalThrustLbs() const;

  // Thrust-to-weight ratio.  Returns 0.0 if weight is unavailable.
  double GetThrustToWeight() const;

  // Engine start: magneto+starter for piston, InitRunning for others.
  void Start();

  // Throttle for all engines [0, 1].
  void SetThrottle(double value);

  // Brakes: left, right, center.
  void SetBrakes(bool on);

  // Flaps [0, 1].
  void SetFlaps(double value);

  // Landing gear.
  void SetGearDown(bool down);

  // Weight on wheels: true if aircraft is on the ground.
  bool IsWeightOnWheels() const;

  // Fuel mixture for piston engines.
  void SetMixture(double value);

 private:
  void DetectType();
  void MeasureRatedThrust();
  void SetIndexedProperty(const std::string& base, int index, double value);
  double GetProperty(const std::string& name) const;

  adapter::JsbsimAdapter& adapter_;
  JSBSim::FGFDMExec& exec_;
  EngineType type_ = EngineType::kUnknown;
  int count_ = 0;
  double rated_thrust_lbs_ = 0.0;
  bool has_magneto_ = false;
  bool has_starter_ = false;
  bool has_mixture_ = false;
  bool has_wow_ = false;
};

}  // namespace propulsion
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_PROPULSION_ENGINEMANAGER_H_
