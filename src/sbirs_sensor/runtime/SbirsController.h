/**
 * @file SbirsController.h
 * @brief SBIRS-inspired 周期控制器。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"

namespace sbirs_sensor {
namespace runtime {

class SbirsController {
 public:
  explicit SbirsController(const config::SbirsInternalExecutionConfig& config);

  void ApplyConfig(const config::SbirsInternalExecutionConfig& config);
  session::SbirsCycleResult RunOnce(const session::SbirsCycleInput& input);

 private:
  pipeline::SbirsPipeline pipeline_;
  bool has_latest_output_{false};
  session::SbirsOutputFrame latest_output_{};
};

}  // namespace runtime
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_
