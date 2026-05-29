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

  struct InitDiagnostics {
    bool model_loaded = false;
    bool ic_applied = false;
    bool run_ic_ok = false;
    bool engines_started = false;
    bool gear_retracted = false;
    bool trim_attempted = false;
    bool trim_succeeded = false;
    bool trim_recovery_applied = false;
  };

  const InitDiagnostics& GetInitDiagnostics() const { return init_diag_; }
  bool TrimAttempted() const { return init_diag_.trim_attempted; }
  bool TrimSucceeded() const { return init_diag_.trim_succeeded; }

 private:
  bool LoadAircraft(const config::FlightDynamicConfig& config);
  void ConfigureIntegrators(const config::FlightDynamicConfig& config);

  std::unique_ptr<JSBSim::FGFDMExec> fdm_exec_;
  InitDiagnostics init_diag_;
};

}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_
