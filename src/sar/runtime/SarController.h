/**
 * @file SarController.h
 * @brief SAR 运行期控制器，驱动处理流水线单周期执行与运行期配置热更新。
 */

#ifndef ONEQ_SRC_SAR_RUNTIME_SAR_CONTROLLER_H_
#define ONEQ_SRC_SAR_RUNTIME_SAR_CONTROLLER_H_

#include <cstdint>
#include <memory>

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "sar/pipeline/SarProcessingPipeline.h"

namespace sar {
namespace pipeline {
class SarProcessingPipeline;
}
namespace extension {

/**
 * @brief SarController 运行期可序列化状态快照。
 *
 * 包含当前运行期配置、上一周期输出、最近一次周期结果及其底层流水线状态，用于
 * replay/checkpoint 的捕获与恢复。
 * @note 恢复前应校验 schema_version 与当前实现一致。
 */
struct SarControllerRuntimeState {
  const void* owner_identity{nullptr};          /**< 用于恢复时校验所有者身份的 opaque 指针 */
  std::uint32_t schema_version{0U};             /**< 状态结构 schema 版本，恢复时用于一致性校验 */
  config::SarSessionConfig runtime_config{};    /**< 当前运行期配置 */
  session::SarOutputFrame previous_output{};    /**< 上一周期输出帧 */
  bool has_previous_output{false};              /**< 是否存在有效的上一周期输出 */
  session::SarCycleResult latest_result{};      /**< 最近一次周期结果 */
  pipeline::SarProcessingPipelineRuntimeState pipeline_state{}; /**< 底层流水线运行期状态 */
};

/**
 * @brief SAR 运行期控制器。
 *
 * 持有一个 pipeline 引用并维护运行期配置与最近输出，对外暴露单周期执行、结果构造、
 * 运行期配置热更新与状态捕获/恢复。不可拷贝。
 * @note 非线程安全；调用方需保证对同一实例的访问串行化。
 */
class SarController {
 public:
  /**
   * @brief 绑定流水线并以初始配置构造控制器。
   * @param[in] pipeline 被驱动的处理流水线引用（调用方负责保活）。
   * @param[in] initial_config 初始会话配置。
   */
  SarController(pipeline::SarProcessingPipeline& pipeline,
                const config::SarSessionConfig& initial_config);
  ~SarController();

  SarController(const SarController&) = delete;
  SarController& operator=(const SarController&) = delete;

  /**
   * @brief 执行单周期处理并缓存结果。
   * @param[in] input 单周期输入载荷。
   */
  void RunOnce(const session::SarCycleInput& input);
  /**
   * @brief 基于当前输入构造单周期结果（不修改内部缓存状态）。
   * @param[in] input 单周期输入载荷。
   * @return 构造出的单周期结果。
   */
  session::SarCycleResult BuildCycleResult(const session::SarCycleInput& input) const;

  /**
   * @brief 尝试以补丁热更新运行期配置。
   * @param[in] patch 运行期配置补丁。
   * @return 补丁合法且成功应用返回 true；否则返回 false，配置保持不变。
   */
  bool TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);
  /**
   * @brief 捕获当前运行期状态快照。
   * @return 可用于 RestoreRuntimeState 的状态快照。
   */
  SarControllerRuntimeState CaptureRuntimeState() const;
  /**
   * @brief 从快照恢复运行期状态。
   * @param[in] state 之前捕获的状态快照。
   * @return schema_version 等校验通过返回 true，否则返回 false。
   * @warning 校验失败时状态可能被部分写入，调用方应避免继续使用本实例。
   */
  bool RestoreRuntimeState(const SarControllerRuntimeState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension
}  // namespace sar

#endif  // ONEQ_SRC_SAR_RUNTIME_SAR_CONTROLLER_H_
