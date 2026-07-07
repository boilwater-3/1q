/**
 * @file SarProcessingPipeline.h
 * @brief SAR 单周期处理流水线，编排 raw echo 生成、成像聚焦与质量摘要。
 */

#ifndef ONEQ_SRC_SAR_PIPELINE_SAR_PROCESSING_PIPELINE_H_
#define ONEQ_SRC_SAR_PIPELINE_SAR_PROCESSING_PIPELINE_H_

#include <cstdint>
#include <deque>
#include <memory>

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/runtime/PulseRingBuffer.h"

namespace sar {
namespace pipeline {

/**
 * @brief SAR 处理流水线运行期可序列化状态快照。
 *
 * 用于 replay/checkpoint 的状态捕获与恢复，包含 raw 脉冲 ring buffer、理想与实际轨迹
 * 缓冲、下一个脉冲 ID 及脉冲分数余量等跨周期状态。
 * @note 恢复前应校验 schema_version 与当前实现一致，否则恢复结果未定义。
 */
struct SarProcessingPipelineRuntimeState {
  const void* owner_identity{nullptr};           /**< 用于恢复时校验所有者身份的 opaque 指针 */
  std::uint32_t schema_version{0U};              /**< 状态结构 schema 版本，恢复时用于一致性校验 */
  runtime::PulseRingBufferRuntimeState raw_pulse_buffer_state{}; /**< raw 脉冲 ring buffer 状态 */
  std::deque<geometry::PlatformPulseState> ideal_trajectory_buffer{}; /**< 理想轨迹脉冲缓冲 */
  std::deque<geometry::PlatformPulseState> actual_trajectory_buffer{}; /**< 实际轨迹脉冲缓冲 */
  std::uint64_t next_pulse_id{0U};               /**< 下一个待分配的脉冲 ID */
  double pulse_fraction_carry{0.0};              /**< PRF 重采样累积的脉冲分数余量 */
};

/**
 * @brief SAR 单周期处理流水线。
 *
 * 编排 raw echo 生成、L1 RDA / L3 BP 聚焦成像与图像质量摘要。不可拷贝；持有跨周期
 * 状态（脉冲 ring buffer、轨迹缓冲），通过 CaptureRuntimeState/RestoreRuntimeState
 * 支持 replay 与状态恢复。
 */
class SarProcessingPipeline {
 public:
  /**
   * @brief 用初始会话配置构造流水线。
   * @param[in] initial_config 初始会话配置。
   */
  explicit SarProcessingPipeline(const config::SarSessionConfig& initial_config);
  ~SarProcessingPipeline();

  SarProcessingPipeline(const SarProcessingPipeline&) = delete;
  SarProcessingPipeline& operator=(const SarProcessingPipeline&) = delete;

  /**
   * @brief 执行单周期处理。
   * @param[in] config 本周期会话配置（可与初始配置不同）。
   * @param[in] input 单周期输入载荷。
   * @param[out] result 单周期输出与诊断结果。
   * @return 成功完成处理返回 true；中止（abort）返回 false，错误诊断写入 result。
   */
  bool RunCycle(const config::SarSessionConfig& config, const session::SarCycleInput& input,
                session::SarCycleResult* result);

  /**
   * @brief 捕获当前运行期状态快照。
   * @return 可用于 RestoreRuntimeState 的状态快照。
   */
  SarProcessingPipelineRuntimeState CaptureRuntimeState() const;
  /**
   * @brief 从快照恢复运行期状态。
   * @param[in] state 之前捕获的状态快照。
   * @return schema_version 等校验通过返回 true，否则返回 false。
   * @warning 校验失败时状态可能被部分写入，调用方应避免继续使用本实例。
   */
  bool RestoreRuntimeState(const SarProcessingPipelineRuntimeState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pipeline
}  // namespace sar

#endif  // ONEQ_SRC_SAR_PIPELINE_SAR_PROCESSING_PIPELINE_H_
