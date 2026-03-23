/**
 * @file EsrOutputManager.h
 * @brief 定义 ESR 输出帧装配器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_CORE_OUTPUT_ESR_OUTPUT_MANAGER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_CORE_OUTPUT_ESR_OUTPUT_MANAGER_H_

#include <cstdint>

#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace core {
namespace output {

/**
 * @brief EsrOutputManager 负责把流水线结果装配为三通道输出帧。
 */
class EsrOutputManager final {
 public:
  /**
   * @brief 装配当前周期三通道输出帧。
   * @param[in] cycle_index 周期号。
   * @param[in] batch_id 批次号。
   * @param[in] cycle_result 流水线结果。
   * @return 组装后的输出帧。
   */
  common::EsrOutputFrame BuildOutputFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
      const pipeline::InterceptCycleResult& cycle_result) const;

  /**
   * @brief 构造空内容输出帧。
   * @param[in] cycle_index 周期号。
   * @param[in] batch_id 批次号。
   * @return 无观测、无假设、无关联的输出帧。
   */
  common::EsrOutputFrame BuildEmptyFrame(std::uint32_t cycle_index,
                                         std::uint64_t batch_id) const;
};

}  // namespace output
}  // namespace core
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_CORE_OUTPUT_ESR_OUTPUT_MANAGER_H_
