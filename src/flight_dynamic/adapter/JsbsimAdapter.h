#ifndef ONEQ_FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_
#define ONEQ_FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_

#include <memory>
#include <string>

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "FGFDMExec.h"

namespace oneq {
namespace flight_dynamic {
namespace adapter {

class JsbsimAdapter {
 public:
  explicit JsbsimAdapter(const config::FlightDynamicConfig& config);
  ~JsbsimAdapter();

  JsbsimAdapter(const JsbsimAdapter&) = delete;
  JsbsimAdapter& operator=(const JsbsimAdapter&) = delete;
  JsbsimAdapter(JsbsimAdapter&&) = default;
  JsbsimAdapter& operator=(JsbsimAdapter&&) = default;

  bool Run();
  bool RunIC();
  void SetDeltaT(double dt_sec);
  double GetDeltaT() const;

  double GetProperty(const std::string& name) const;
  void SetProperty(const std::string& name, double value);
  bool HasProperty(const std::string& name) const;

  JSBSim::FGPropagate& GetPropagate();
  const JSBSim::FGPropagate& GetPropagate() const;
  JSBSim::FGAccelerations& GetAccelerations();
  const JSBSim::FGAccelerations& GetAccelerations() const;
  JSBSim::FGFDMExec& GetFdmExec();
  const JSBSim::FGFDMExec& GetFdmExec() const;

  bool IsValid() const { return fdm_exec_ != nullptr; }
  bool TrimAttempted() const { return trim_attempted_; }
  bool TrimSucceeded() const { return trim_succeeded_; }

 private:
  bool LoadAircraft(const config::FlightDynamicConfig& config);
  void ConfigureIntegrators(const config::FlightDynamicConfig& config);

  std::unique_ptr<JSBSim::FGFDMExec> fdm_exec_;
  bool trim_attempted_ = false;
  bool trim_succeeded_ = false;
};

}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_
