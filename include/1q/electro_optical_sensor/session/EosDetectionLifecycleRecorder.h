/**
 * @file EosDetectionLifecycleRecorder.h
 * @brief 定义 EOS 目标探测生命周期记录器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace session {

// 前向声明：Update 参数为 const 引用，header 无需完整类型，避免拉入 EosCycleInput 重依赖。
struct EosCycleInput;

enum class EosDetectionLifecycleEventKind {
  kFirstDetected = 0,
  kUpdated = 1,
  kLost = 2,
  kNotDetected = 3
};

enum class EosDetectionLifecycleReason {
  kNone = 0,
  kOutOfFov = 1,
  kBelowSnrThreshold = 2,
  kValidationRejected = 3,
  kCycleNotExecuted = 4,
  kUnknown = 5
};

struct ONEQ_API EosDetectionLifecycleEvent {
  std::uint32_t cycle_index{0U};
  std::uint64_t target_id{0U};
  std::string target_name{};
  EosDetectionLifecycleEventKind kind{EosDetectionLifecycleEventKind::kUpdated};
  EosDetectionLifecycleReason reason{EosDetectionLifecycleReason::kNone};
  float fused_snr_db{0.0f};
  float range_m{0.0f};
};

struct ONEQ_API EosDetectionLifecycleRecorderConfig {
  bool emit_not_detected_events{false};
};

/**
 * @brief 记录目标首次发现/更新/丢失/未发现事件；未发现原因需显式开启。
 *
 * 私有状态(含 unordered_map)与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 */
class ONEQ_API EosDetectionLifecycleRecorder {
 public:
  explicit EosDetectionLifecycleRecorder(
      EosDetectionLifecycleRecorderConfig config = EosDetectionLifecycleRecorderConfig{});
  ~EosDetectionLifecycleRecorder();

  EosDetectionLifecycleRecorder(const EosDetectionLifecycleRecorder&) = delete;
  EosDetectionLifecycleRecorder& operator=(const EosDetectionLifecycleRecorder&) = delete;
  // 移动操作声明在 header、定义在 .cpp：unique_ptr<Impl> 析构需要完整类型，
  // 不能内联定义否则破坏 PImpl 不透明性。
  EosDetectionLifecycleRecorder(EosDetectionLifecycleRecorder&&) noexcept;
  EosDetectionLifecycleRecorder& operator=(EosDetectionLifecycleRecorder&&) noexcept;

  std::vector<EosDetectionLifecycleEvent> Update(const EosCycleInput& input, const EosCycleResult& result);

  void Reset();

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_
