/**
 * @file EsrEmitterLifecycleRecorder.h
 * @brief 定义 ESR 辐射源观测生命周期记录器。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EMITTER_LIFECYCLE_RECORDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EMITTER_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"

namespace electronic_surveillance_radar {
namespace session {

// 前向声明：Update 参数为 const 引用，header 无需完整类型，避免拉入 EsrCycleInput 重依赖。
struct EsrCycleInput;

/**
 * @brief EsrEmitterLifecycleEventKind 表示辐射源观测生命周期事件类型。
 */
enum class EsrEmitterLifecycleEventKind {
  kFirstObserved = 0, /**< 首次被观测到 */
  kUpdated = 1,       /**< 已知辐射源在本周期再次被观测到 */
  kLost = 2,          /**< 先前观测到的辐射源本周期丢失 */
  kNotObserved = 3    /**< 从未观测到的辐射源未命中（仅在开启时产生） */
};

/**
 * @brief EsrEmitterLifecycleReason 表示未观测/丢失事件的推断原因。
 */
enum class EsrEmitterLifecycleReason {
  kNone = 0,                /**< 无原因占位值（事件为观测/更新时使用） */
  kNotEmitting = 1,         /**< 辐射源本周期未发射 */
  kNoMatchedObservation = 2,/**< 执行了 pipeline 但无匹配观测 */
  kValidationRejected = 3,  /**< 因输入校验 Error 未执行计算 */
  kCycleNotExecuted = 4,    /**< 本周期未执行核心 pipeline */
  kUnknown = 5              /**< 未知原因占位值 */
};

/**
 * @brief EsrEmitterLifecycleEvent 描述单条辐射源生命周期事件。
 */
struct ONEQ_API EsrEmitterLifecycleEvent {
  std::uint32_t cycle_index{0U};                                  /**< 触发周期号（取自 result.input_cycle_index） */
  std::uint64_t emitter_id{0U};                                   /**< 辐射源标识 */
  std::string emitter_name{};                                     /**< 辐射源名称，仅用于人读 */
  EsrEmitterLifecycleEventKind kind{EsrEmitterLifecycleEventKind::kUpdated}; /**< 事件类型 */
  EsrEmitterLifecycleReason reason{EsrEmitterLifecycleReason::kNone};         /**< 未观测/丢失原因 */
  std::uint64_t observation_id{0U};                               /**< 关联观测记录标识（未命中时为 0） */
  float confidence{0.0f};                                         /**< 关联置信度，范围 [0, 1] */
};

/**
 * @brief EsrEmitterLifecycleRecorderConfig 描述生命周期记录器配置。
 */
struct ONEQ_API EsrEmitterLifecycleRecorderConfig {
  bool emit_not_observed_events{false}; /**< 是否为“从未观测到”的辐射源产生 kNotObserved 事件 */
};

/**
 * @brief 记录辐射源首次观测/更新/丢失/未观测事件；未观测原因需显式开启。
 *
 * 私有状态(含 unordered_map)与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 */
class ONEQ_API EsrEmitterLifecycleRecorder {
 public:
  explicit EsrEmitterLifecycleRecorder(
      EsrEmitterLifecycleRecorderConfig config = EsrEmitterLifecycleRecorderConfig{});
  ~EsrEmitterLifecycleRecorder();

  EsrEmitterLifecycleRecorder(const EsrEmitterLifecycleRecorder&) = delete;
  EsrEmitterLifecycleRecorder& operator=(const EsrEmitterLifecycleRecorder&) = delete;
  // 移动操作声明在 header、定义在 .cpp：unique_ptr<Impl> 析构需要完整类型，
  // 不能内联定义否则破坏 PImpl 不透明性。
  EsrEmitterLifecycleRecorder(EsrEmitterLifecycleRecorder&&) noexcept;
  EsrEmitterLifecycleRecorder& operator=(EsrEmitterLifecycleRecorder&&) noexcept;

  /**
   * @brief 基于单周期输入与结果更新各辐射源生命周期状态并产出事件。
   *
   * 遍历 `input.scene` 中的辐射源，结合真值评估关联判断本周期是否观测到。
   * 对首次观测/再次观测/丢失（含推断原因）产生对应事件；“从未观测”事件仅在
   * 配置开启时产生。内部状态（已观测标记）会被原地更新。
   *
   * @param[in] input 单周期输入（仅消费 scene 辐射源表）。
   * @param[in] result 单周期聚合结果（消费周期号与真值评估通道）。
   * @return 本周期产生的生命周期事件列表。
   */
  std::vector<EsrEmitterLifecycleEvent> Update(const EsrCycleInput& input, const EsrCycleResult& result);

  /**
   * @brief 清空内部辐射源观测状态，使所有辐射源回到“未观测”初始态。
   */
  void Reset();

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EMITTER_LIFECYCLE_RECORDER_H_
