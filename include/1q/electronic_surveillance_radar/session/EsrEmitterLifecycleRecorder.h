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

enum class EsrEmitterLifecycleEventKind {
  kFirstObserved = 0,
  kUpdated = 1,
  kLost = 2,
  kNotObserved = 3
};

enum class EsrEmitterLifecycleReason {
  kNone = 0,
  kNotEmitting = 1,
  kNoMatchedObservation = 2,
  kValidationRejected = 3,
  kCycleNotExecuted = 4,
  kUnknown = 5
};

struct ONEQ_API EsrEmitterLifecycleEvent {
  std::uint32_t cycle_index{0U};
  std::uint64_t emitter_id{0U};
  std::string emitter_name{};
  EsrEmitterLifecycleEventKind kind{EsrEmitterLifecycleEventKind::kUpdated};
  EsrEmitterLifecycleReason reason{EsrEmitterLifecycleReason::kNone};
  std::uint64_t observation_id{0U};
  float confidence{0.0f};
};

struct ONEQ_API EsrEmitterLifecycleRecorderConfig {
  bool emit_not_observed_events{false};
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

  std::vector<EsrEmitterLifecycleEvent> Update(const EsrCycleInput& input, const EsrCycleResult& result);

  void Reset();

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EMITTER_LIFECYCLE_RECORDER_H_
