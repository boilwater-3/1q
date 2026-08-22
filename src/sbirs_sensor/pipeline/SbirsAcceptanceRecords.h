/**
 * @file SbirsAcceptanceRecords.h
 * @brief SBIRS 验收行拼装（安装矩阵、轨道角抽样累计、生命周期、性能暂无项）。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"

namespace sbirs_sensor {
namespace pipeline {

void WriteSbirsInstallMatrices(const oneq::coordinate::EulerAnglesDeg& mount_deg,
                               const oneq::coordinate::EulerAnglesDeg& misalignment_deg);

void WriteSbirsOrbitSample(float sim_time_sec, std::uint32_t cycle, float orbit_sigma_deg,
                           double reference_range_m);

void WriteSbirsAngleError(float sim_time_sec, std::uint32_t cycle, std::uint64_t target_id,
                          double az_error_deg, double el_error_deg, double measured_az_deg,
                          double measured_el_deg, double truth_az_deg, double truth_el_deg);

void WriteSbirsLifecycleEvents(float sim_time_sec, std::uint32_t cycle,
                               const std::vector<session::SbirsDetectionLifecycleEvent>& events,
                               const session::SbirsCycleInput& input);

void WriteSbirsOncePerSession(float sim_time_sec, std::uint32_t cycle);
void WriteSbirsCycleRunCount(float sim_time_sec, std::uint32_t cycle);

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif
