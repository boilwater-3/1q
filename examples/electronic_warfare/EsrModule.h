/** @file EsrModule.h @brief 外部仿真引擎使用 ESR 单周期门面的最小集成包装。 */

#ifndef EXAMPLES_ESR_MODULE_H_
#define EXAMPLES_ESR_MODULE_H_

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "config_loader.h"

namespace esr_config = electronic_surveillance_radar::config;
namespace esr_session = electronic_surveillance_radar::session;

class EsrModule {
 public:
  using ConfigPatchCallback = std::function<void(esr_config::EsrRuntimeConfigPatch&)>;

  bool initialize(const esr_config::EsrSessionConfig& config = {}) {
    session_ = esr_session::EsrSession::Create(config);
    initialized_ = true;
    return true;
  }

  bool preStart(const std::string& config_path) {
    esr_config::EsrSessionConfig config;
    std::string error;
    if (!examples::LoadEsrSessionConfigFromFile(config_path.c_str(), &config, &error)) {
      return false;
    }
    return initialize(config);
  }

  void stepImp(const esr_session::EsrCycleInput& input) {
    if (!initialized_) {
      (void)initialize();
    }
    esr_config::EsrRuntimeConfigPatch patch;
    for (const ConfigPatchCallback& callback : callbacks_) {
      callback(patch);
    }
    (void)session_.TryApplyRuntimeConfig(patch);
    last_result_ = session_.StepWithResult(input);
  }

  void registerConfigPatchCallback(ConfigPatchCallback callback) {
    callbacks_.push_back(std::move(callback));
  }

  const esr_session::EsrCycleResult& lastResult() const { return last_result_; }

 private:
  esr_session::EsrSession session_{};
  esr_session::EsrCycleResult last_result_{};
  std::vector<ConfigPatchCallback> callbacks_{};
  bool initialized_{false};
};

#endif  // EXAMPLES_ESR_MODULE_H_
